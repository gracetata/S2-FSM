from importlib.util import find_spec
import json
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch


HAS_RUNTIME_DEPENDENCIES = (
    find_spec("numpy") is not None
    and find_spec("unitree_sdk2py") is not None
)


@unittest.skipUnless(
    HAS_RUNTIME_DEPENDENCIES,
    "NumPy and Unitree SDK2 are required",
)
class UnitreeModeGuardTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from locomotion_controller import unitree_controller

        cls.module = unitree_controller

    def test_releases_active_mode_and_confirms_debug_mode(self):
        events = []
        motion_client = MagicMock()
        motion_results = iter(
            (
                (0, {"name": "ai", "form": "0"}),
                (0, {"name": "", "form": "0"}),
            )
        )

        def check_mode():
            events.append("check_mode")
            return next(motion_results)

        def release_mode():
            events.append("release_mode")
            return 0, None

        motion_client.CheckMode.side_effect = check_mode
        motion_client.ReleaseMode.side_effect = release_mode
        with patch.object(
            self.module,
            "MotionSwitcherClient",
            return_value=motion_client,
        ):
            self.module.enter_debug_mode(1.0)

        self.assertEqual(
            events,
            [
                "check_mode",
                "release_mode",
                "check_mode",
            ],
        )

    def test_releases_non_ai_mode(self):
        motion_client = MagicMock()
        motion_client.CheckMode.side_effect = (
            (0, {"name": "normal", "form": "0"}),
            (0, {"name": "", "form": "0"}),
        )
        motion_client.ReleaseMode.return_value = 0, None
        with patch.object(
            self.module,
            "MotionSwitcherClient",
            return_value=motion_client,
        ):
            self.module.enter_debug_mode(1.0)

        motion_client.ReleaseMode.assert_called_once_with()

    def test_accepts_existing_debug_mode_without_release(self):
        motion_client = MagicMock()
        motion_client.CheckMode.return_value = (
            0,
            {"name": "", "form": "0"},
        )
        with patch.object(
            self.module,
            "MotionSwitcherClient",
            return_value=motion_client,
        ):
            self.module.enter_debug_mode(1.0)

        motion_client.ReleaseMode.assert_not_called()

    def test_rejects_malformed_motion_mode_response(self):
        motion_client = MagicMock()
        motion_client.CheckMode.return_value = 0, {"name": "ai"}
        with patch.object(
            self.module,
            "MotionSwitcherClient",
            return_value=motion_client,
        ):
            with self.assertRaisesRegex(RuntimeError, "invalid mode result"):
                self.module.enter_debug_mode(1.0)

        motion_client.ReleaseMode.assert_not_called()

    def test_inference_log_contains_effective_inputs_and_raw_output(self):
        import numpy as np

        from locomotion_controller.protocol import ArmCommand
        from locomotion_controller.state_machine import ControlSelection

        controller = self.module.UnitreeController.__new__(
            self.module.UnitreeController
        )
        controller._inference_frame_index = 7
        selection = ControlSelection(
            model_name="arm_walk",
            command_semantics="velocity",
            command=(0.2, 0.0, -0.1),
            arm_command=ArmCommand(
                sequence=9,
                positions=(0.1,) * 14,
                velocities=(0.2,) * 14,
                weight=0.8,
            ),
            high_mode=3,
            low_mode=1,
            is_standing_transition=False,
        )
        with patch("builtins.print") as print_mock:
            controller._log_inference_frame(
                selection,
                np.asarray((0.15, 0.0, -0.05), dtype=np.float32),
                np.arange(29, dtype=np.float32),
            )

        log_line = print_mock.call_args.args[0]
        payload = json.loads(log_line.removeprefix("[INFERENCE] "))
        self.assertEqual(payload["frame"], 7)
        self.assertEqual(payload["model"], "arm_walk")
        self.assertEqual(
            payload["navigation_input"]["selected"],
            [0.2, 0.0, -0.1],
        )
        self.assertEqual(payload["navigation_input"]["semantics"], "velocity")
        self.assertEqual(payload["arm_output_override"]["sequence"], 9)
        self.assertEqual(payload["arm_output_override"]["weight"], 0.8)
        self.assertEqual(payload["model_output"], list(range(29)))
        self.assertEqual(controller._inference_frame_index, 8)

    def test_velocity_command_is_applied_without_a_ramp(self):
        import numpy as np

        from locomotion_controller.state_machine import ControlSelection

        controller = self.module.UnitreeController.__new__(
            self.module.UnitreeController
        )
        controller._max_velocity_command = np.asarray(
            (0.8, 0.5, 1.57),
            dtype=np.float32,
        )
        selection = ControlSelection(
            model_name="free_walk",
            command_semantics="velocity",
            command=(0.4, -0.2, 0.6),
            arm_command=None,
            high_mode=1,
            low_mode=1,
            is_standing_transition=False,
        )

        np.testing.assert_allclose(
            controller._command_for(selection),
            np.asarray(selection.command, dtype=np.float32),
        )

    def test_whole_body_positions_follow_configured_motor_order(self):
        controller = self.module.UnitreeController.__new__(
            self.module.UnitreeController
        )
        controller._config = SimpleNamespace(motor_indices=(2, 0, 1))
        controller._low_state = SimpleNamespace(
            motor_state=(
                SimpleNamespace(q=0.1),
                SimpleNamespace(q=0.2),
                SimpleNamespace(q=0.3),
            )
        )

        self.assertEqual(
            controller.whole_body_positions(),
            (0.3, 0.1, 0.2),
        )

    def test_arm_modes_share_dedicated_gains(self):
        from locomotion_controller.state_machine import (
            MODEL_ARM_STAND,
            MODEL_ARM_WALK,
        )

        controller = self.module.UnitreeController.__new__(
            self.module.UnitreeController
        )
        common = ("common_angles", "common_kp", "common_kd")
        standing = ("standing_angles", "standing_kp", "standing_kd")
        (
            controller._default_angles,
            controller._kps,
            controller._kds,
        ) = common
        (
            controller._arm_stand_default_angles,
            controller._arm_stand_kps,
            controller._arm_stand_kds,
        ) = standing

        arm_stand = SimpleNamespace(model_name=MODEL_ARM_STAND)
        arm_walk = SimpleNamespace(model_name=MODEL_ARM_WALK)
        self.assertEqual(controller._parameters_for(arm_stand), standing)
        self.assertEqual(
            controller._parameters_for(arm_walk),
            ("common_angles", "standing_kp", "standing_kd"),
        )

    def test_previous_action_uses_post_override_executed_action(self):
        import numpy as np

        controller = self.module.UnitreeController.__new__(
            self.module.UnitreeController
        )
        controller._config = SimpleNamespace(action_scale=1.0)
        target_after_override = np.asarray(
            (1.0, 20.0, 3.0, 40.0),
            dtype=np.float32,
        )

        previous_action = controller._previous_action_for(
            target_after_override,
            np.zeros(4, dtype=np.float32),
        )

        np.testing.assert_allclose(
            previous_action,
            target_after_override,
        )

    def test_model_switch_blend_never_interpolates_arm_joints(self):
        import numpy as np

        controller = self.module.UnitreeController.__new__(
            self.module.UnitreeController
        )
        controller._config = MagicMock(model_switch_blend_s=1.0)
        controller._switch_started_at = 1.0
        controller._switch_from_target = np.zeros(4, dtype=np.float32)
        controller._arm_indices = np.asarray((1, 3), dtype=np.int64)
        target = np.asarray((2.0, 4.0, 6.0, 8.0), dtype=np.float32)

        blended = controller._blend_model_switch(target, now=1.5)

        np.testing.assert_allclose(
            blended,
            np.asarray((1.0, 4.0, 3.0, 8.0), dtype=np.float32),
        )
