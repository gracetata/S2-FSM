from importlib.util import find_spec
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

    def test_releases_only_after_ai_and_zero_torque(self):
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
        loco_client = MagicMock()

        def get_fsm_id():
            events.append("get_fsm_id")
            return 0, 0

        loco_client.GetFsmId.side_effect = get_fsm_id
        with (
            patch.object(
                self.module,
                "MotionSwitcherClient",
                return_value=motion_client,
            ),
            patch.object(
                self.module,
                "LocoClient",
                return_value=loco_client,
            ),
        ):
            self.module.release_motion_mode(1.0)

        self.assertEqual(
            events,
            [
                "check_mode",
                "get_fsm_id",
                "release_mode",
                "check_mode",
            ],
        )

    def test_rejects_non_ai_mode_without_release(self):
        motion_client = MagicMock()
        motion_client.CheckMode.return_value = (
            0,
            {"name": "normal", "form": "0"},
        )
        loco_client = MagicMock()
        with (
            patch.object(
                self.module,
                "MotionSwitcherClient",
                return_value=motion_client,
            ),
            patch.object(
                self.module,
                "LocoClient",
                return_value=loco_client,
            ),
        ):
            with self.assertRaisesRegex(RuntimeError, "mode=ai"):
                self.module.release_motion_mode(1.0)

        loco_client.GetFsmId.assert_not_called()
        motion_client.ReleaseMode.assert_not_called()

    def test_rejects_nonzero_fsm_without_release(self):
        motion_client = MagicMock()
        motion_client.CheckMode.return_value = (
            0,
            {"name": "ai", "form": "0"},
        )
        loco_client = MagicMock()
        loco_client.GetFsmId.return_value = 0, 1
        with (
            patch.object(
                self.module,
                "MotionSwitcherClient",
                return_value=motion_client,
            ),
            patch.object(
                self.module,
                "LocoClient",
                return_value=loco_client,
            ),
        ):
            with self.assertRaisesRegex(RuntimeError, "FSM=0"):
                self.module.release_motion_mode(1.0)

        motion_client.ReleaseMode.assert_not_called()

    def test_rejects_malformed_motion_mode_response(self):
        motion_client = MagicMock()
        motion_client.CheckMode.return_value = 0, {"name": "ai"}
        loco_client = MagicMock()
        with (
            patch.object(
                self.module,
                "MotionSwitcherClient",
                return_value=motion_client,
            ),
            patch.object(
                self.module,
                "LocoClient",
                return_value=loco_client,
            ),
        ):
            with self.assertRaisesRegex(RuntimeError, "invalid mode result"):
                self.module.release_motion_mode(1.0)

        loco_client.GetFsmId.assert_not_called()
        motion_client.ReleaseMode.assert_not_called()
