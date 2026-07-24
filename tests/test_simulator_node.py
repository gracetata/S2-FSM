from importlib.util import find_spec
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock


HAS_ROS_DEPENDENCIES = (
    find_spec("ament_index_python") is not None
    and find_spec("rclpy") is not None
)
if HAS_ROS_DEPENDENCIES:
    from locomotion_controller.simulator_node import SimulatorNode


@unittest.skipUnless(HAS_ROS_DEPENDENCIES, "ROS 2 Python packages are required")
class SimulatorNodeTest(unittest.TestCase):
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
            get_logger=MagicMock(return_value=MagicMock()),
        )

        should_quit = SimulatorNode._handle_key(node, "3")

        self.assertFalse(should_quit)
        self.assertEqual(node._high_mode, 3)
        node._publish_mode.assert_called_once_with(
            node._high_mode_publisher,
            3,
        )


if __name__ == "__main__":
    unittest.main()
