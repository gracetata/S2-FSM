from importlib.util import find_spec
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock


HAS_ROS_DEPENDENCIES = (
    find_spec("rclpy") is not None
    and find_spec("std_msgs") is not None
)
if HAS_ROS_DEPENDENCIES:
    from std_msgs.msg import Float32MultiArray, UInt8

    from locomotion_controller.totarget_bridge_node import (
        ToTargetNavigationBridge,
    )


@unittest.skipUnless(HAS_ROS_DEPENDENCIES, "ROS 2 Python packages are required")
class ToTargetNavigationBridgeTest(unittest.TestCase):
    def test_only_forwards_target_error_in_low_mode_two(self):
        node = SimpleNamespace(
            _target_forwarding_enabled=False,
            _navigation_publisher=MagicMock(),
            _error_count=0,
            get_logger=MagicMock(return_value=MagicMock()),
        )
        error = Float32MultiArray()
        error.data = [0.3, -0.1, 0.2]

        ToTargetNavigationBridge._receive_totarget_error(node, error)
        node._navigation_publisher.publish.assert_not_called()

        low_mode = UInt8()
        low_mode.data = 2
        ToTargetNavigationBridge._receive_low_mode(node, low_mode)
        ToTargetNavigationBridge._receive_totarget_error(node, error)

        forwarded = node._navigation_publisher.publish.call_args.args[0]
        for actual, expected in zip(forwarded.data, (0.3, -0.1, 0.2)):
            self.assertAlmostEqual(actual, expected, places=6)

        low_mode.data = 1
        ToTargetNavigationBridge._receive_low_mode(node, low_mode)
        node._navigation_publisher.reset_mock()
        ToTargetNavigationBridge._receive_totarget_error(node, error)
        node._navigation_publisher.publish.assert_not_called()


if __name__ == "__main__":
    unittest.main()
