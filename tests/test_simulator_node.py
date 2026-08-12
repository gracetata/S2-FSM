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
            _is_navigation_publishing_enabled=True,
            _is_arm_publishing_enabled=True,
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
            _is_navigation_publishing_enabled=False,
            _is_arm_publishing_enabled=False,
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
            _print_velocity_state=MagicMock(),
            _is_navigation_publishing_enabled=True,
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
            _enable_navigation_publishing=MagicMock(),
            _log_current_state=MagicMock(),
            _print_velocity_state=MagicMock(),
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
        node._print_velocity_state.assert_called_once_with("W")
        node._enable_navigation_publishing.assert_called_once_with()

    def test_mode_five_disables_keyboard_navigation_before_publish(self):
        node = SimpleNamespace(
            _high_mode=1,
            _is_controller_initialized=True,
            _high_mode_publisher=MagicMock(),
            _publish_mode=MagicMock(),
            _disable_navigation_publishing=MagicMock(),
            _log_current_state=MagicMock(),
        )

        SimulatorNode._handle_key(node, "5")

        node._disable_navigation_publishing.assert_called_once_with()
        node._publish_mode.assert_called_once_with(node._high_mode_publisher, 5)

    def test_position_mode_key_hands_navigation_to_totarget(self):
        node = SimpleNamespace(
            _low_mode=1,
            _is_controller_initialized=True,
            _low_mode_publisher=MagicMock(),
            _publish_mode=MagicMock(),
            _stop_navigation=MagicMock(),
            _disable_navigation_publishing=MagicMock(),
            _log_current_state=MagicMock(),
        )

        SimulatorNode._handle_key(node, "p")

        self.assertEqual(node._low_mode, 2)
        node._disable_navigation_publishing.assert_called_once_with()
        node._publish_mode.assert_called_once_with(node._low_mode_publisher, 2)

    def test_velocity_terminal_line_states_if_command_is_actually_sent(self):
        node = SimpleNamespace(
            _navigation_command=(0.25, -0.1, 0.35),
            _is_controller_initialized=True,
            _is_navigation_publishing_enabled=False,
        )
        with unittest.mock.patch("builtins.print") as print_mock:
            SimulatorNode._print_velocity_state(node, "W")

        line = print_mock.call_args.args[0]
        self.assertIn("[KEYBOARD_VELOCITY]", line)
        self.assertIn("vx=0.25 m/s", line)
        self.assertIn("vy=-0.10 m/s", line)
        self.assertIn("yaw_rate=0.35 rad/s", line)
        self.assertIn("NOT_PUBLISHED_PRESS_N", line)

    def test_navigation_only_toggle_never_enables_arm_publishing(self):
        node = SimpleNamespace(
            _is_navigation_publishing_enabled=False,
            _is_arm_publishing_enabled=False,
            _is_parameter_publishing_enabled=False,
            _stop_navigation=MagicMock(),
        )

        SimulatorNode._toggle_navigation_publishing(node)

        self.assertTrue(node._is_navigation_publishing_enabled)
        self.assertFalse(node._is_arm_publishing_enabled)

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
            _is_navigation_publishing_enabled=True,
            _is_arm_publishing_enabled=True,
            get_logger=MagicMock(return_value=logger),
        )

        SimulatorNode._log_current_state(node, "velocity key -> W")

        message = logger.info.call_args.args[0]
        self.assertIn("[KEYBOARD_STATE]", message)
        self.assertIn("publishing=on", message)
        self.assertIn("navigation_publishing=on", message)
        self.assertIn("arm_publishing=on", message)
        self.assertIn("high_mode=3", message)
        self.assertIn("low_mode=1", message)
        self.assertIn("navigation=[0.25, -0.10, 0.35]", message)
        self.assertIn("arm_pose=down", message)


if __name__ == "__main__":
    unittest.main()
