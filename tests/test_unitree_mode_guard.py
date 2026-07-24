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
