"""ROS 2 boundary for the four upstream messages."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, String, UInt8

from .config import PACKAGE_NAME, load_config
from .protocol import parse_arm_command, parse_navigation_command
from .runtime_client import RuntimeClient


class LocomotionControllerNode(Node):
    def __init__(self) -> None:
        super().__init__(PACKAGE_NAME)
        package_root = Path(get_package_share_directory(PACKAGE_NAME))
        default_config = package_root / "config" / "locomotion_controller.yaml"
        self.declare_parameter("config_file", str(default_config))
        config_file = str(self.get_parameter("config_file").value)
        self._config = load_config(config_file, package_root)

        initialized_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._initialized_publisher = self.create_publisher(
            Bool,
            self._config.topics.initialized,
            initialized_qos,
        )
        self._runtime = RuntimeClient(self._config)
        try:
            self._runtime.start()
        except Exception:
            self._runtime.close()
            raise

        queue_depth = self._config.topics.queue_depth
        self._high_mode_subscription = self.create_subscription(
            UInt8,
            self._config.topics.high_level_mode,
            self._receive_high_mode,
            queue_depth,
        )
        self._low_mode_subscription = self.create_subscription(
            UInt8,
            self._config.topics.low_level_mode,
            self._receive_low_mode,
            queue_depth,
        )
        self._navigation_subscription = self.create_subscription(
            Float32MultiArray,
            self._config.topics.navigation_command,
            self._receive_navigation,
            queue_depth,
        )
        self._arm_subscription = self.create_subscription(
            String,
            self._config.topics.arm_command,
            self._receive_arm,
            queue_depth,
        )

        initialized = Bool()
        initialized.data = True
        self._initialized_publisher.publish(initialized)
        self.get_logger().info(
            "four ONNX models are ready; initialization stand is complete"
        )

    def destroy_node(self) -> bool:
        try:
            if hasattr(self, "_runtime"):
                self._runtime.close()
        except Exception as error:
            self.get_logger().error(f"runtime shutdown failed: {error}")
        return super().destroy_node()

    def _receive_high_mode(self, message: UInt8) -> None:
        try:
            if self._runtime.set_high_mode(message.data):
                self.get_logger().info(
                    f"模式切换为 high mode {message.data}"
                )
        except (ConnectionError, OSError, RuntimeError, ValueError) as error:
            self.get_logger().error(f"high-level mode rejected: {error}")

    def _receive_low_mode(self, message: UInt8) -> None:
        try:
            if self._runtime.set_low_mode(message.data):
                self.get_logger().info(
                    f"模式切换为 low mode {message.data}"
                )
        except (ConnectionError, OSError, RuntimeError, ValueError) as error:
            self.get_logger().error(f"low-level mode rejected: {error}")

    def _receive_navigation(self, message: Float32MultiArray) -> None:
        try:
            command = parse_navigation_command(message.data)
            self._runtime.set_navigation(command)
        except (ConnectionError, OSError, RuntimeError, ValueError) as error:
            self.get_logger().error(f"navigation command rejected: {error}")

    def _receive_arm(self, message: String) -> None:
        try:
            command = parse_arm_command(message.data)
            self._runtime.set_arm(command)
        except (
            ConnectionError,
            OSError,
            RuntimeError,
            ValueError,
        ) as error:
            self.get_logger().error(f"arm command rejected: {error}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: LocomotionControllerNode | None = None
    try:
        node = LocomotionControllerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
