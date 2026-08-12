from importlib.util import find_spec
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock


HAS_ROS_DEPENDENCIES = (
    find_spec("ament_index_python") is not None
    and find_spec("rclpy") is not None
)
if HAS_ROS_DEPENDENCIES:
    from locomotion_controller.simulator_node import (
        PARAMETER_PUBLISHING_ENABLED_AT_STARTUP,
        SimulatorNode,
    )


@unittest.skipUnless(HAS_ROS_DEPENDENCIES, "ROS 2 Python packages are required")
class SimulatorNodeTest(unittest.TestCase):
    def test_parameter_publishing_starts_disabled(self):
        self.assertFalse(PARAMETER_PUBLISHING_ENABLED_AT_STARTUP)

    def test_parameter_publishing_can_be_stopped_and_resumed(self):
        node = SimpleNamespace(
            _is_parameter_publishing_enabled=True,
            _stop_navigation=MagicMock(),
            get_logger=MagicMock(return_value=MagicMock()),
        )

        SimulatorNode._toggle_parameter_publishing(node)

        self.assertFalse(node._is_parameter_publishing_enabled)
        node._stop_navigation.assert_called_once_with()
        node.get_logger().warning.assert_called_once()

        SimulatorNode._toggle_parameter_publishing(node)

        self.assertTrue(node._is_parameter_publishing_enabled)
        node.get_logger().info.assert_called_once()

    def test_disabled_cycle_publishes_no_navigation_or_arm_parameters(self):
        node = SimpleNamespace(
            _read_keys=MagicMock(return_value=False),
            _is_controller_initialized=True,
            _is_parameter_publishing_enabled=False,
            _update_navigation=MagicMock(),
            _navigation_publisher=MagicMock(),
            _arm_publisher=MagicMock(),
        )

        SimulatorNode._run_cycle(node)

        node._update_navigation.assert_not_called()
        node._navigation_publisher.publish.assert_not_called()
        node._arm_publisher.publish.assert_not_called()

    def test_mode_key_still_publishes_while_parameters_are_disabled(self):
        node = SimpleNamespace(
            _high_mode=None,
            _is_controller_initialized=True,
            _is_parameter_publishing_enabled=False,
            _high_mode_publisher=MagicMock(),
            _publish_mode=MagicMock(),
            _log_current_state=MagicMock(),
            get_logger=MagicMock(return_value=MagicMock()),
        )

        should_quit = SimulatorNode._handle_key(node, "4")

        self.assertFalse(should_quit)
        self.assertEqual(node._high_mode, 4)
        node._publish_mode.assert_called_once_with(
            node._high_mode_publisher,
            4,
        )

    def test_velocity_key_activates_low_mode_one_and_starts_from_zero(self):
        node = SimpleNamespace(
            _velocity_trajectory=object(),
            _low_mode=2,
            _navigation_command=(0.3, 0.0, 0.0),
            _is_controller_initialized=True,
            _low_mode_publisher=MagicMock(),
            _publish_mode=MagicMock(),
            _log_current_state=MagicMock(),
            _config=SimpleNamespace(
                controller=SimpleNamespace(
                    max_velocity_command=(0.8, 0.5, 1.57),
                )
            ),
            get_logger=MagicMock(return_value=MagicMock()),
        )

        SimulatorNode._adjust_velocity(node, "w")

        self.assertEqual(node._low_mode, 1)
        self.assertEqual(node._navigation_command, (0.05, 0.0, 0.0))
        self.assertIsNone(node._velocity_trajectory)
        node._publish_mode.assert_called_once_with(
            node._low_mode_publisher,
            1,
        )

    def test_space_cycles_recommended_arm_poses_only_in_modes_two_three(self):
        poses = (object(), object(), object(), object())
        node = SimpleNamespace(
            _high_mode=2,
            _arm_pose_index=0,
            _catalog=SimpleNamespace(arm_poses=poses),
            _set_arm_pose=MagicMock(),
            get_logger=MagicMock(return_value=MagicMock()),
        )

        SimulatorNode._cycle_arm_pose(node)

        node._set_arm_pose.assert_called_once_with(poses[1], 1)

        node._set_arm_pose.reset_mock()
        node._high_mode = 1
        SimulatorNode._cycle_arm_pose(node)
        node._set_arm_pose.assert_not_called()

    def test_current_state_log_contains_modes_velocity_arm_and_publishing(self):
        logger = MagicMock()
        node = SimpleNamespace(
            _high_mode=3,
            _low_mode=1,
            _navigation_command=(0.25, -0.1, 0.35),
            _arm_pose=SimpleNamespace(name="down"),
            _velocity_trajectory=None,
            _is_parameter_publishing_enabled=True,
            get_logger=MagicMock(return_value=logger),
        )

        SimulatorNode._log_current_state(node, "velocity key -> W")

        message = logger.info.call_args.args[0]
        self.assertIn("[KEYBOARD_STATE]", message)
        self.assertIn("publishing=on", message)
        self.assertIn("high_mode=3", message)
        self.assertIn("low_mode=1", message)
        self.assertIn("navigation=[0.25, -0.10, 0.35]", message)
        self.assertIn("arm_pose=down", message)


if __name__ == "__main__":
    unittest.main()
