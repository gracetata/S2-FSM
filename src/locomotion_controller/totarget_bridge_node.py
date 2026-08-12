"""ROS-only bridge between Kairui ToTarget navigation and S2-FSM.

This node never initializes Unitree SDK, MotionSwitcher, or LowCmd.  It only
relays the measured whole-body state toward Kairui ego-motion and validated
pelvis-frame ToTarget errors toward the S2-FSM navigation input.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, UInt8


class ToTargetNavigationBridge(Node):
    def __init__(self) -> None:
        super().__init__("totarget_navigation_bridge")
        self.declare_parameter(
            "totarget_error_topic",
            "/kairui/totarget/shadow_pelvis_error",
        )
        self.declare_parameter(
            "fsm_navigation_topic",
            "/hecbot/locomotion/navigation_command",
        )
        self.declare_parameter("fsm_whole_body_topic", "/hecbot/whole_body_state")
        self.declare_parameter(
            "kairui_whole_body_topic",
            "/kairui/whole_body_state",
        )
        self.declare_parameter(
            "fsm_low_mode_topic",
            "/hecbot/locomotion/low_level_mode",
        )

        error_topic = str(self.get_parameter("totarget_error_topic").value)
        navigation_topic = str(self.get_parameter("fsm_navigation_topic").value)
        fsm_state_topic = str(self.get_parameter("fsm_whole_body_topic").value)
        kairui_state_topic = str(
            self.get_parameter("kairui_whole_body_topic").value
        )
        low_mode_topic = str(self.get_parameter("fsm_low_mode_topic").value)
        if error_topic == navigation_topic:
            raise ValueError("ToTarget source and FSM navigation target must differ")
        if fsm_state_topic == kairui_state_topic:
            raise ValueError("FSM and Kairui whole-body topics must differ")

        self._navigation_publisher = self.create_publisher(
            Float32MultiArray,
            navigation_topic,
            10,
        )
        self._whole_body_publisher = self.create_publisher(
            String,
            kairui_state_topic,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            error_topic,
            self._receive_totarget_error,
            10,
        )
        self.create_subscription(
            String,
            fsm_state_topic,
            self._receive_whole_body_state,
            10,
        )
        self.create_subscription(
            UInt8,
            low_mode_topic,
            self._receive_low_mode,
            10,
        )
        self._target_forwarding_enabled = False
        self._error_count = 0
        self._state_count = 0
        self.get_logger().info(
            "ROS-only bridge ready (never publishes LowCmd): "
            f"{error_topic} -> {navigation_topic}; "
            f"{fsm_state_topic} -> {kairui_state_topic}"
        )

    def _receive_totarget_error(self, message: Float32MultiArray) -> None:
        if not self._target_forwarding_enabled:
            return
        values = tuple(float(value) for value in message.data)
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            self.get_logger().error(
                "rejected ToTarget error: expected exactly three finite values"
            )
            return
        forwarded = Float32MultiArray()
        forwarded.data = list(values)
        self._navigation_publisher.publish(forwarded)
        self._error_count += 1
        if self._error_count == 1 or self._error_count % 100 == 0:
            self.get_logger().info(
                "forwarded ToTarget error "
                f"#{self._error_count}: [{values[0]:+.4f}, "
                f"{values[1]:+.4f}, {values[2]:+.4f}]"
            )

    def _receive_whole_body_state(self, message: String) -> None:
        forwarded = String()
        forwarded.data = message.data
        self._whole_body_publisher.publish(forwarded)
        self._state_count += 1
        if self._state_count == 1:
            self.get_logger().info("forwarding S2-FSM whole-body state to Kairui")

    def _receive_low_mode(self, message: UInt8) -> None:
        should_enable = int(message.data) == 2
        if should_enable == self._target_forwarding_enabled:
            return
        self._target_forwarding_enabled = should_enable
        state = "enabled" if should_enable else "disabled"
        self.get_logger().info(f"ToTarget forwarding {state} by low mode")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ToTargetNavigationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
