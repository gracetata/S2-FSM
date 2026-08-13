"""Interactive ROS 2 publisher for end-to-end controller testing."""

from __future__ import annotations

import json
from pathlib import Path
import select
import sys
import termios
from time import monotonic, time_ns
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
    ARM_CYCLE_POSE_COUNT,
    KEYBOARD_VELOCITY_DELTAS,
    ZERO_COMMAND,
    ArmPose,
    PresetCatalog,
    VelocityTrajectory,
    adjust_keyboard_velocity,
    load_preset_catalog,
    next_arm_sequence,
    next_arm_cycle_pose_index,
)


PUBLISH_PERIOD_S = 0.05
# W/S, A/D and Q/E are reserved for incremental velocity control. Fixed test
# trajectories use F/G/R. Numeric key 5 is reserved for high mode 5.
VELOCITY_KEYS = ("f", "g", "r")
POSITION_KEYS = ("7", "8", "9")
ARM_POSE_KEYS = ("z", "x", "c", "b")
ARM_CYCLE_HIGH_MODES = {2, 3}
PARAMETER_PUBLISHING_ENABLED_AT_STARTUP = False


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
        # Start in the same state as after pressing k: mode keys work, while
        # navigation and arm parameters remain silent until explicitly enabled.
        self._is_parameter_publishing_enabled = (
            PARAMETER_PUBLISHING_ENABLED_AT_STARTUP
        )
        self._is_navigation_publishing_enabled = (
            PARAMETER_PUBLISHING_ENABLED_AT_STARTUP
        )
        self._is_arm_publishing_enabled = (
            PARAMETER_PUBLISHING_ENABLED_AT_STARTUP
        )
        self._high_mode: int | None = None
        self._low_mode: int | None = None
        self._navigation_command = ZERO_COMMAND
        self._velocity_trajectory: VelocityTrajectory | None = None
        self._velocity_started_at = 0.0
        initial_pose = self._catalog.arm_poses[0]
        self._arm_pose = initial_pose
        self._arm_pose_index = 0
        self._arm_positions = initial_pose.positions
        self._arm_velocities = (0.0,) * len(initial_pose.positions)
        self._arm_sequence = -1
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
        if not (
            self._is_navigation_publishing_enabled
            or self._is_arm_publishing_enabled
        ):
            return
        now = monotonic()
        if self._is_navigation_publishing_enabled:
            self._update_navigation(now)
            navigation_message = Float32MultiArray()
            navigation_message.data = list(self._navigation_command)
            self._navigation_publisher.publish(navigation_message)
        if (
            self._is_arm_publishing_enabled
            and self._high_mode in ARM_CYCLE_HIGH_MODES
        ):
            self._publish_arm_command()

    def _publish_arm_command(self) -> None:
        self._arm_sequence = next_arm_sequence(
            self._arm_sequence,
            time_ns(),
        )
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

    def _read_keys(self) -> bool:
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1).lower()
            if self._handle_key(key):
                return True
        return False

    def _handle_key(self, key: str) -> bool:
        if key == "\x1b":
            self.get_logger().info("simulator stopped by keyboard")
            rclpy.shutdown()
            return True
        if key == "h":
            self._print_help()
            return False
        if key == "k":
            self._toggle_parameter_publishing()
            self._log_current_state("parameter publishing toggled")
            return False
        if key == "n":
            self._toggle_navigation_publishing()
            self._log_current_state("navigation publishing toggled")
            return False
        if key == "m":
            self._toggle_arm_publishing()
            self._log_current_state("arm publishing toggled")
            return False
        if key in {"1", "2", "3", "4", "5", "6"}:
            self._high_mode = int(key)
            if self._high_mode == 5:
                self._disable_navigation_publishing()
            if self._high_mode == 6:
                self._disable_navigation_publishing()
                self._disable_arm_publishing()
            if self._is_controller_initialized:
                self._publish_mode(self._high_mode_publisher, self._high_mode)
            self._log_current_state(f"high mode -> {self._high_mode}")
            return False
        if key in {"v", "p"}:
            self._low_mode = 1 if key == "v" else 2
            self._stop_navigation()
            if key == "p":
                # One-key ownership handoff: stop keyboard navigation before
                # announcing low mode 2, leaving ToTarget as sole publisher.
                self._disable_navigation_publishing()
            if self._is_controller_initialized:
                self._publish_mode(self._low_mode_publisher, self._low_mode)
            self._log_current_state(f"low mode -> {self._low_mode}")
            return False
        if key == "0":
            self._stop_navigation()
            self._log_current_state("navigation zeroed")
            self._print_velocity_state("0")
            return False
        if key in KEYBOARD_VELOCITY_DELTAS:
            self._adjust_velocity(key)
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
            self._log_current_state(f"position target -> {target.name}")
            return False
        if key in ARM_POSE_KEYS:
            index = ARM_POSE_KEYS.index(key)
            self._set_arm_pose(self._catalog.arm_poses[index], index)
            return False
        if key == " ":
            self._cycle_arm_pose()
        return False

    def _adjust_velocity(self, key: str) -> None:
        self._velocity_trajectory = None
        if self._low_mode != 1:
            self._low_mode = 1
            self._navigation_command = ZERO_COMMAND
            if self._is_controller_initialized:
                self._publish_mode(self._low_mode_publisher, self._low_mode)
        self._enable_navigation_publishing()
        self._navigation_command = adjust_keyboard_velocity(
            self._navigation_command,
            key,
            self._config.controller.max_velocity_command,
        )
        self._log_current_state(f"velocity key -> {key.upper()}")
        self._print_velocity_state(key.upper())

    def _start_velocity_trajectory(
        self,
        trajectory: VelocityTrajectory,
    ) -> None:
        self._velocity_trajectory = trajectory
        self._velocity_started_at = monotonic()
        self._navigation_command = ZERO_COMMAND
        self._log_current_state(
            f"velocity trajectory -> {trajectory.name}"
        )

    def _set_arm_pose(self, pose: ArmPose, index: int) -> None:
        self._arm_pose = pose
        self._arm_pose_index = index
        self._arm_positions = pose.positions
        self._arm_velocities = (0.0,) * len(pose.positions)
        self._log_current_state(f"arm pose -> {pose.name}")

    def _cycle_arm_pose(self) -> None:
        if self._high_mode not in ARM_CYCLE_HIGH_MODES:
            self.get_logger().warning(
                "SPACE arm-pose cycle is available only in high modes 2/3"
            )
            return
        next_index = next_arm_cycle_pose_index(self._arm_pose_index)
        self._set_arm_pose(self._catalog.arm_poses[next_index], next_index)

    def _stop_navigation(self) -> None:
        self._velocity_trajectory = None
        self._navigation_command = ZERO_COMMAND

    def _log_current_state(self, event: str) -> None:
        high_mode = getattr(self, "_high_mode", None)
        low_mode = getattr(self, "_low_mode", None)
        command = getattr(self, "_navigation_command", ZERO_COMMAND)
        arm_pose = getattr(getattr(self, "_arm_pose", None), "name", "none")
        trajectory = getattr(self, "_velocity_trajectory", None)
        trajectory_name = (
            getattr(trajectory, "name", "active")
            if trajectory is not None
            else "none"
        )
        navigation_publishing = getattr(
            self,
            "_is_navigation_publishing_enabled",
            False,
        )
        arm_publishing = getattr(
            self,
            "_is_arm_publishing_enabled",
            False,
        )
        if navigation_publishing and arm_publishing:
            publishing = "on"
        elif navigation_publishing or arm_publishing:
            publishing = "mixed"
        else:
            publishing = "off"
        self.get_logger().info(
            "[KEYBOARD_STATE] "
            f"event={event} | publishing={publishing} | "
            f"navigation_publishing={'on' if navigation_publishing else 'off'} | "
            f"arm_publishing={'on' if arm_publishing else 'off'} | "
            f"high_mode={high_mode} | low_mode={low_mode} | "
            f"navigation=[{command[0]:.2f}, {command[1]:.2f}, "
            f"{command[2]:.2f}] | arm_pose={arm_pose} | "
            f"trajectory={trajectory_name}"
        )

    def _print_velocity_state(self, key: str) -> None:
        command = self._navigation_command
        if not self._is_controller_initialized:
            delivery = "WAIT_INITIALIZED"
        elif self._is_navigation_publishing_enabled:
            delivery = "PUBLISHING_20HZ"
        else:
            delivery = "NOT_PUBLISHED_PRESS_N"
        print(
            "[KEYBOARD_VELOCITY] "
            f"key={key} | vx={command[0]:.2f} m/s | "
            f"vy={command[1]:.2f} m/s | "
            f"yaw_rate={command[2]:.2f} rad/s | "
            f"delivery={delivery}",
            flush=True,
        )

    def _toggle_parameter_publishing(self) -> None:
        should_enable = not (
            self._is_navigation_publishing_enabled
            or self._is_arm_publishing_enabled
        )
        self._is_navigation_publishing_enabled = should_enable
        self._is_arm_publishing_enabled = should_enable
        self._is_parameter_publishing_enabled = should_enable
        if should_enable:
            self.get_logger().info(
                "navigation and arm parameter publishing resumed"
            )
            return
        self._stop_navigation()
        self.get_logger().warning(
            "navigation and arm parameter publishing stopped; "
            "high/low mode keys remain active"
        )

    def _toggle_navigation_publishing(self) -> None:
        self._is_navigation_publishing_enabled = (
            not self._is_navigation_publishing_enabled
        )
        if not self._is_navigation_publishing_enabled:
            self._stop_navigation()
        self._is_parameter_publishing_enabled = (
            self._is_navigation_publishing_enabled
            and self._is_arm_publishing_enabled
        )

    def _disable_navigation_publishing(self) -> None:
        self._is_navigation_publishing_enabled = False
        self._stop_navigation()
        self._is_parameter_publishing_enabled = False

    def _enable_navigation_publishing(self) -> None:
        self._is_navigation_publishing_enabled = True
        self._is_parameter_publishing_enabled = self._is_arm_publishing_enabled

    def _toggle_arm_publishing(self) -> None:
        self._is_arm_publishing_enabled = (
            not self._is_arm_publishing_enabled
        )
        self._is_parameter_publishing_enabled = (
            self._is_navigation_publishing_enabled
            and self._is_arm_publishing_enabled
        )

    def _disable_arm_publishing(self) -> None:
        self._is_arm_publishing_enabled = False
        self._is_parameter_publishing_enabled = False

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
            (
                f"  {key}: arms {pose.name} - {pose.description_zh}"
                + (
                    " [recommended arm-walk model preset]"
                    if index < 3
                    else " [extra integration-test pose]"
                )
            )
            for index, (key, pose) in enumerate(
                zip(ARM_POSE_KEYS, self._catalog.arm_poses)
            )
        ]
        lines = [
            "",
            "Keyboard controls:",
            "  1/2: high mode 1/2",
            "  3: high mode 3 (velocity requires low mode 1 / key v)",
            "  4: high mode 4 (stand recovery; zero command; direct switch)",
            "  5: high mode 5 (free walk [0,0,0] stand; stop keyboard navigation)",
            "  6: high mode 6 (suspend S2-FSM LowCmd for external control)",
            "  v: low mode 1 (velocity for high modes 1 and 3)",
            "  p: low mode 2 and stop keyboard navigation (ToTarget handoff)",
            "  W/S: increase/decrease vx by 0.05 m/s",
            "  A/D: increase/decrease vy by 0.05 m/s",
            "  Q/E: increase/decrease yaw rate by 0.05 rad/s",
            "  0: cancel navigation and publish [0,0,0]",
            "  k: start/stop navigation and arm parameter publishing "
            "(starts stopped)",
            "  n: start/stop navigation publishing only (use for W/S tests)",
            "  m: start/stop arm publishing only (high modes 2/3 only)",
            *velocity_lines,
            *position_lines,
            *arm_lines,
            "  SPACE: cycle z/x/c arm poses in high modes 2/3",
            "  Every accepted change prints one [KEYBOARD_STATE] line",
            "  h: print help",
            "  ESC or Ctrl+C: quit",
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
