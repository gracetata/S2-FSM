"""Interactive ROS 2 publisher for end-to-end controller testing."""

from __future__ import annotations

import json
from pathlib import Path
import select
import sys
import termios
from time import monotonic, monotonic_ns
import tty

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, String, UInt8

from .config import PACKAGE_NAME, load_config
from .protocol import ARM_COMMAND_SCHEMA
from .simulator_presets import (
    ZERO_COMMAND,
    ArmPose,
    PresetCatalog,
    VelocityTrajectory,
    load_preset_catalog,
)


PUBLISH_PERIOD_S = 0.05
VELOCITY_KEYS = ("4", "5", "6")
POSITION_KEYS = ("7", "8", "9")
ARM_POSE_KEYS = ("z", "x", "c", "b")


class SimulatorNode(Node):
    """Publish deterministic keyboard-selected inputs at 20 Hz."""

    def __init__(self) -> None:
        super().__init__("locomotion_controller_simulator")
        package_root = Path(get_package_share_directory(PACKAGE_NAME))
        default_config = package_root / "config" / "locomotion_controller.yaml"
        default_presets = package_root / "config" / "simulator_presets.json"
        self.declare_parameter("config_file", str(default_config))
        self.declare_parameter("preset_file", str(default_presets))
        config_file = str(self.get_parameter("config_file").value)
        preset_file = str(self.get_parameter("preset_file").value)
        self._config = load_config(config_file, package_root)
        self._catalog = load_preset_catalog(preset_file)
        self._validate_key_capacity(self._catalog)

        topics = self._config.topics
        self._high_mode_publisher = self.create_publisher(
            UInt8,
            topics.high_level_mode,
            topics.queue_depth,
        )
        self._low_mode_publisher = self.create_publisher(
            UInt8,
            topics.low_level_mode,
            topics.queue_depth,
        )
        self._navigation_publisher = self.create_publisher(
            Float32MultiArray,
            topics.navigation_command,
            topics.queue_depth,
        )
        self._arm_publisher = self.create_publisher(
            String,
            topics.arm_command,
            topics.queue_depth,
        )
        initialized_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._initialized_subscription = self.create_subscription(
            Bool,
            topics.initialized,
            self._receive_initialized,
            initialized_qos,
        )

        self._is_controller_initialized = False
        self._is_parameter_publishing_enabled = True
        self._high_mode: int | None = None
        self._low_mode: int | None = None
        self._navigation_command = ZERO_COMMAND
        self._velocity_trajectory: VelocityTrajectory | None = None
        self._velocity_started_at = 0.0
        initial_pose = self._catalog.arm_poses[0]
        self._arm_pose = initial_pose
        self._arm_positions = initial_pose.positions
        self._arm_velocities = (0.0,) * len(initial_pose.positions)
        self._arm_sequence = monotonic_ns()
        if not sys.stdin.isatty():
            raise RuntimeError("simulator must run in an interactive terminal")
        self._terminal_settings = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        self._timer = self.create_timer(PUBLISH_PERIOD_S, self._run_cycle)
        self._print_help()
        self.get_logger().info(
            "waiting for locomotion controller initialized=true"
        )

    def destroy_node(self) -> bool:
        if hasattr(self, "_terminal_settings"):
            termios.tcsetattr(
                sys.stdin.fileno(),
                termios.TCSADRAIN,
                self._terminal_settings,
            )
        return super().destroy_node()

    def _receive_initialized(self, message: Bool) -> None:
        if not message.data or self._is_controller_initialized:
            return
        self._is_controller_initialized = True
        self.get_logger().info(
            "controller initialized; keyboard commands are now active"
        )
        if self._low_mode is not None:
            self._publish_mode(self._low_mode_publisher, self._low_mode)
        if self._high_mode is not None:
            self._publish_mode(self._high_mode_publisher, self._high_mode)

    def _run_cycle(self) -> None:
        if self._read_keys():
            return
        if not self._is_controller_initialized:
            return
        if not self._is_parameter_publishing_enabled:
            return
        now = monotonic()
        self._update_navigation(now)
        navigation_message = Float32MultiArray()
        navigation_message.data = list(self._navigation_command)
        self._navigation_publisher.publish(navigation_message)

        arm_message = String()
        arm_message.data = json.dumps(
            {
                "schema": ARM_COMMAND_SCHEMA,
                "seq": self._arm_sequence,
                "arm_q": list(self._arm_positions),
                "arm_dq": list(self._arm_velocities),
                "weight": self._arm_pose.weight,
            },
            separators=(",", ":"),
        )
        self._arm_publisher.publish(arm_message)
        self._arm_sequence += 1

    def _read_keys(self) -> bool:
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1).lower()
            if self._handle_key(key):
                return True
        return False

    def _handle_key(self, key: str) -> bool:
        if key == "q":
            self.get_logger().info("simulator stopped by keyboard")
            rclpy.shutdown()
            return True
        if key == "h":
            self._print_help()
            return False
        if key == "k":
            self._toggle_parameter_publishing()
            return False
        if key in {"1", "2", "3"}:
            self._high_mode = int(key)
            if self._is_controller_initialized:
                self._publish_mode(self._high_mode_publisher, self._high_mode)
            self.get_logger().info(f"high mode -> {self._high_mode}")
            return False
        if key in {"v", "p"}:
            self._low_mode = 1 if key == "v" else 2
            self._stop_navigation()
            if self._is_controller_initialized:
                self._publish_mode(self._low_mode_publisher, self._low_mode)
            self.get_logger().info(f"low mode -> {self._low_mode}")
            return False
        if key == "0":
            self._stop_navigation()
            self.get_logger().info("navigation -> [0, 0, 0]")
            return False
        if key in VELOCITY_KEYS:
            index = VELOCITY_KEYS.index(key)
            self._start_velocity_trajectory(
                self._catalog.velocity_trajectories[index]
            )
            return False
        if key in POSITION_KEYS:
            index = POSITION_KEYS.index(key)
            target = self._catalog.position_targets[index]
            self._velocity_trajectory = None
            self._navigation_command = target.command
            self.get_logger().info(
                f"position target -> {target.name}: {target.command}"
            )
            return False
        if key in ARM_POSE_KEYS:
            index = ARM_POSE_KEYS.index(key)
            self._set_arm_pose(self._catalog.arm_poses[index])
        return False

    def _start_velocity_trajectory(
        self,
        trajectory: VelocityTrajectory,
    ) -> None:
        self._velocity_trajectory = trajectory
        self._velocity_started_at = monotonic()
        self._navigation_command = ZERO_COMMAND
        self.get_logger().info(
            f"velocity trajectory -> {trajectory.name}: "
            f"{trajectory.description_zh}"
        )

    def _set_arm_pose(self, pose: ArmPose) -> None:
        self._arm_pose = pose
        self._arm_positions = pose.positions
        self._arm_velocities = (0.0,) * len(pose.positions)
        self.get_logger().info(
            f"arm pose -> {pose.name}: {pose.description_zh}"
        )

    def _stop_navigation(self) -> None:
        self._velocity_trajectory = None
        self._navigation_command = ZERO_COMMAND

    def _toggle_parameter_publishing(self) -> None:
        self._is_parameter_publishing_enabled = (
            not self._is_parameter_publishing_enabled
        )
        if self._is_parameter_publishing_enabled:
            self.get_logger().info(
                "navigation and arm parameter publishing resumed"
            )
            return
        self._stop_navigation()
        self.get_logger().warning(
            "navigation and arm parameter publishing stopped; "
            "high/low mode keys remain active"
        )

    def _update_navigation(self, now: float) -> None:
        trajectory = self._velocity_trajectory
        if trajectory is None:
            return
        elapsed_s = now - self._velocity_started_at
        self._navigation_command = trajectory.sample(elapsed_s)
        if not trajectory.should_loop and elapsed_s >= trajectory.duration_s:
            self._velocity_trajectory = None
            self.get_logger().info(
                f"velocity trajectory complete -> {trajectory.name}"
            )

    @staticmethod
    def _publish_mode(publisher: object, value: int) -> None:
        message = UInt8()
        message.data = value
        publisher.publish(message)

    def _print_help(self) -> None:
        velocity_lines = [
            f"  {key}: velocity {trajectory.name} - "
            f"{trajectory.description_zh}"
            for key, trajectory in zip(
                VELOCITY_KEYS,
                self._catalog.velocity_trajectories,
            )
        ]
        position_lines = [
            f"  {key}: position {target.name} - {target.description_zh}"
            for key, target in zip(
                POSITION_KEYS,
                self._catalog.position_targets,
            )
        ]
        arm_lines = [
            f"  {key}: arms {pose.name} - {pose.description_zh}"
            for key, pose in zip(ARM_POSE_KEYS, self._catalog.arm_poses)
        ]
        lines = [
            "",
            "Keyboard controls:",
            "  1/2/3: high mode",
            "  v: low mode 1 (velocity)",
            "  p: low mode 2 (position)",
            "  0: cancel navigation and publish [0,0,0]",
            "  k: stop/resume navigation and arm parameter publishing",
            *velocity_lines,
            *position_lines,
            *arm_lines,
            "  h: print help",
            "  q: quit",
        ]
        self.get_logger().info("\n".join(lines))

    @staticmethod
    def _validate_key_capacity(catalog: PresetCatalog) -> None:
        if len(catalog.velocity_trajectories) != len(VELOCITY_KEYS):
            raise ValueError(
                f"preset file must contain {len(VELOCITY_KEYS)} "
                "velocity trajectories"
            )
        if len(catalog.position_targets) != len(POSITION_KEYS):
            raise ValueError(
                f"preset file must contain {len(POSITION_KEYS)} "
                "position targets"
            )
        if len(catalog.arm_poses) != len(ARM_POSE_KEYS):
            raise ValueError(
                f"preset file must contain {len(ARM_POSE_KEYS)} arm poses"
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: SimulatorNode | None = None
    try:
        node = SimulatorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
