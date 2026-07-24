"""Strict configuration loader for the locomotion controller."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from numbers import Real
from pathlib import Path
from typing import Any

import yaml


PACKAGE_NAME = "locomotion_controller"
MODEL_NAMES = (
    "free_walk",
    "accurate_arrival",
    "arm_stand",
    "arm_walk",
)
POLICY_JOINT_COUNT = 29
COMMAND_SIZE = 3


@dataclass(frozen=True)
class TopicConfig:
    initialized: str
    high_level_mode: str
    low_level_mode: str
    navigation_command: str
    arm_command: str
    queue_depth: int


@dataclass(frozen=True)
class RuntimeConfig:
    python_executable: Path
    socket_path: Path
    log_root: Path
    startup_timeout_s: float
    request_timeout_s: float
    cyclonedds_home: Path
    network_interface: str
    robot_ip: str
    should_check_robot: bool
    is_real_robot_confirmed: bool


@dataclass(frozen=True)
class StateMachineConfig:
    initialization_stand_duration_s: float
    stand_duration_s: float
    navigation_timeout_s: float
    arm_timeout_s: float


@dataclass(frozen=True)
class ControllerConfig:
    control_dt: float
    imu_type: str
    lowcmd_topic: str
    lowstate_topic: str
    lowstate_startup_timeout_s: float
    lowstate_runtime_timeout_s: float
    motion_release_timeout_s: float
    startup_move_s: float
    first_frame_timeout_s: float
    stop_timeout_s: float
    fault_damping_duration_s: float
    model_switch_blend_s: float
    motor_indices: tuple[int, ...]
    policy_joint_names: tuple[str, ...]
    motor_joint_names: tuple[str, ...]
    default_angles: tuple[float, ...]
    kps: tuple[float, ...]
    kds: tuple[float, ...]
    arm_stand_default_angles: tuple[float, ...]
    arm_stand_kps: tuple[float, ...]
    arm_stand_kds: tuple[float, ...]
    angular_velocity_scale: float
    joint_position_scale: float
    joint_velocity_scale: float
    action_scale: float
    max_velocity_command: tuple[float, float, float]


@dataclass(frozen=True)
class PackageConfig:
    config_path: Path
    package_root: Path
    topics: TopicConfig
    runtime: RuntimeConfig
    state_machine: StateMachineConfig
    models: dict[str, Path]
    controller: ControllerConfig


def load_config(
    config_file: str | Path,
    package_root: str | Path,
) -> PackageConfig:
    """Load every required setting without fallback or implicit defaults."""

    config_path = Path(config_file).expanduser().resolve()
    resolved_package_root = Path(package_root).expanduser().resolve()
    root = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    root_mapping = _mapping(root, "configuration root")
    _require_keys(root_mapping, {PACKAGE_NAME}, "configuration root")
    package = _mapping(root_mapping[PACKAGE_NAME], PACKAGE_NAME)
    _require_keys(
        package,
        {"topics", "runtime", "state_machine", "models", "controller"},
        PACKAGE_NAME,
    )

    topics = _load_topics(_mapping(package["topics"], "topics"))
    runtime = _load_runtime(_mapping(package["runtime"], "runtime"))
    state_machine = _load_state_machine(
        _mapping(package["state_machine"], "state_machine")
    )
    models = _load_models(
        _mapping(package["models"], "models"),
        resolved_package_root,
    )
    controller = _load_controller(_mapping(package["controller"], "controller"))
    return PackageConfig(
        config_path=config_path,
        package_root=resolved_package_root,
        topics=topics,
        runtime=runtime,
        state_machine=state_machine,
        models=models,
        controller=controller,
    )


def _load_topics(settings: dict[str, Any]) -> TopicConfig:
    expected = {
        "initialized",
        "high_level_mode",
        "low_level_mode",
        "navigation_command",
        "arm_command",
        "queue_depth",
    }
    _require_keys(settings, expected, "topics")
    topic_values = {
        key: _absolute_topic(settings[key], f"topics.{key}")
        for key in expected - {"queue_depth"}
    }
    if len(set(topic_values.values())) != len(topic_values):
        raise ValueError("all ROS topics must be unique")
    queue_depth = settings["queue_depth"]
    if not isinstance(queue_depth, int) or isinstance(queue_depth, bool):
        raise ValueError("topics.queue_depth must be an integer")
    if queue_depth <= 0:
        raise ValueError("topics.queue_depth must be positive")
    return TopicConfig(queue_depth=queue_depth, **topic_values)


def _load_runtime(settings: dict[str, Any]) -> RuntimeConfig:
    expected = {
        "python_executable",
        "socket_path",
        "log_root",
        "startup_timeout_s",
        "request_timeout_s",
        "cyclonedds_home",
        "network_interface",
        "robot_ip",
        "check_robot_reachable",
        "confirm_real_robot",
    }
    _require_keys(settings, expected, "runtime")
    python_executable = Path(
        _nonempty_string(
            settings["python_executable"],
            "runtime.python_executable",
        )
    ).expanduser().absolute()
    socket_path = Path(
        _nonempty_string(settings["socket_path"], "runtime.socket_path")
    ).expanduser()
    if not socket_path.is_absolute():
        raise ValueError("runtime.socket_path must be absolute")
    log_root = Path(
        _nonempty_string(settings["log_root"], "runtime.log_root")
    ).expanduser()
    if not log_root.is_absolute():
        raise ValueError("runtime.log_root must be absolute")
    cyclonedds_home = Path(
        _nonempty_string(
            settings["cyclonedds_home"],
            "runtime.cyclonedds_home",
        )
    ).expanduser().resolve()
    startup_timeout_s = _positive_float(
        settings["startup_timeout_s"],
        "runtime.startup_timeout_s",
    )
    request_timeout_s = _positive_float(
        settings["request_timeout_s"],
        "runtime.request_timeout_s",
    )
    network_interface = _nonempty_string(
        settings["network_interface"],
        "runtime.network_interface",
    )
    robot_ip = _nonempty_string(settings["robot_ip"], "runtime.robot_ip")
    should_check_robot = _boolean(
        settings["check_robot_reachable"],
        "runtime.check_robot_reachable",
    )
    is_real_robot_confirmed = _boolean(
        settings["confirm_real_robot"],
        "runtime.confirm_real_robot",
    )
    return RuntimeConfig(
        python_executable=python_executable,
        socket_path=socket_path.resolve(),
        log_root=log_root.resolve(),
        startup_timeout_s=startup_timeout_s,
        request_timeout_s=request_timeout_s,
        cyclonedds_home=cyclonedds_home,
        network_interface=network_interface,
        robot_ip=robot_ip,
        should_check_robot=should_check_robot,
        is_real_robot_confirmed=is_real_robot_confirmed,
    )


def _load_state_machine(settings: dict[str, Any]) -> StateMachineConfig:
    expected = {
        "initialization_stand_duration_s",
        "stand_duration_s",
        "navigation_timeout_s",
        "arm_timeout_s",
    }
    _require_keys(settings, expected, "state_machine")
    return StateMachineConfig(
        initialization_stand_duration_s=_positive_float(
            settings["initialization_stand_duration_s"],
            "state_machine.initialization_stand_duration_s",
        ),
        stand_duration_s=_positive_float(
            settings["stand_duration_s"],
            "state_machine.stand_duration_s",
        ),
        navigation_timeout_s=_positive_float(
            settings["navigation_timeout_s"],
            "state_machine.navigation_timeout_s",
        ),
        arm_timeout_s=_positive_float(
            settings["arm_timeout_s"],
            "state_machine.arm_timeout_s",
        ),
    )


def _load_models(
    settings: dict[str, Any],
    package_root: Path,
) -> dict[str, Path]:
    _require_keys(settings, set(MODEL_NAMES), "models")
    models: dict[str, Path] = {}
    for model_name in MODEL_NAMES:
        path = Path(
            _nonempty_string(settings[model_name], f"models.{model_name}")
        ).expanduser()
        if not path.is_absolute():
            path = package_root / path
        path = path.resolve()
        if not path.is_file() or path.suffix.lower() != ".onnx":
            raise ValueError(f"models.{model_name} is not an ONNX file: {path}")
        models[model_name] = path
    return models


def _load_controller(settings: dict[str, Any]) -> ControllerConfig:
    expected = {
        "control_dt",
        "imu_type",
        "lowcmd_topic",
        "lowstate_topic",
        "lowstate_startup_timeout_s",
        "lowstate_runtime_timeout_s",
        "motion_release_timeout_s",
        "startup_move_s",
        "first_frame_timeout_s",
        "stop_timeout_s",
        "fault_damping_duration_s",
        "model_switch_blend_s",
        "motor_indices",
        "policy_joint_names",
        "motor_joint_names",
        "default_angles",
        "kps",
        "kds",
        "arm_stand_default_angles",
        "arm_stand_kps",
        "arm_stand_kds",
        "angular_velocity_scale",
        "joint_position_scale",
        "joint_velocity_scale",
        "action_scale",
        "max_velocity_command",
    }
    _require_keys(settings, expected, "controller")
    control_dt = _positive_float(settings["control_dt"], "controller.control_dt")
    if control_dt != 0.02:
        raise ValueError("controller.control_dt must be exactly 0.02 seconds")
    startup_move_s = _positive_float(
        settings["startup_move_s"],
        "controller.startup_move_s",
    )
    if startup_move_s < control_dt:
        raise ValueError("controller.startup_move_s must cover at least one frame")
    imu_type = _nonempty_string(settings["imu_type"], "controller.imu_type")
    if imu_type not in {"pelvis", "torso"}:
        raise ValueError("controller.imu_type must be pelvis or torso")
    lowcmd_topic = _nonempty_string(
        settings["lowcmd_topic"],
        "controller.lowcmd_topic",
    )
    lowstate_topic = _nonempty_string(
        settings["lowstate_topic"],
        "controller.lowstate_topic",
    )
    if lowcmd_topic == lowstate_topic:
        raise ValueError("Unitree LowCmd and LowState topics must differ")

    motor_indices = _integer_vector(
        settings["motor_indices"],
        "controller.motor_indices",
        POLICY_JOINT_COUNT,
    )
    if set(motor_indices) != set(range(POLICY_JOINT_COUNT)):
        raise ValueError("controller.motor_indices must contain 0 through 28")
    policy_joint_names = _string_vector(
        settings["policy_joint_names"],
        "controller.policy_joint_names",
        POLICY_JOINT_COUNT,
    )
    motor_joint_names = _string_vector(
        settings["motor_joint_names"],
        "controller.motor_joint_names",
        POLICY_JOINT_COUNT,
    )
    if len(set(policy_joint_names)) != POLICY_JOINT_COUNT:
        raise ValueError("controller.policy_joint_names must be unique")
    if set(policy_joint_names) != set(motor_joint_names):
        raise ValueError("policy and motor joint names must contain the same joints")

    kps = _float_vector(settings["kps"], "controller.kps", POLICY_JOINT_COUNT)
    kds = _float_vector(settings["kds"], "controller.kds", POLICY_JOINT_COUNT)
    arm_stand_kps = _float_vector(
        settings["arm_stand_kps"],
        "controller.arm_stand_kps",
        POLICY_JOINT_COUNT,
    )
    arm_stand_kds = _float_vector(
        settings["arm_stand_kds"],
        "controller.arm_stand_kds",
        POLICY_JOINT_COUNT,
    )
    if any(value < 0.0 for value in (*kps, *kds, *arm_stand_kps, *arm_stand_kds)):
        raise ValueError("controller gains cannot be negative")
    max_velocity_command = _float_vector(
        settings["max_velocity_command"],
        "controller.max_velocity_command",
        COMMAND_SIZE,
    )
    if any(value <= 0.0 for value in max_velocity_command):
        raise ValueError("controller.max_velocity_command must be positive")

    return ControllerConfig(
        control_dt=control_dt,
        imu_type=imu_type,
        lowcmd_topic=lowcmd_topic,
        lowstate_topic=lowstate_topic,
        lowstate_startup_timeout_s=_positive_float(
            settings["lowstate_startup_timeout_s"],
            "controller.lowstate_startup_timeout_s",
        ),
        lowstate_runtime_timeout_s=_positive_float(
            settings["lowstate_runtime_timeout_s"],
            "controller.lowstate_runtime_timeout_s",
        ),
        motion_release_timeout_s=_positive_float(
            settings["motion_release_timeout_s"],
            "controller.motion_release_timeout_s",
        ),
        startup_move_s=startup_move_s,
        first_frame_timeout_s=_positive_float(
            settings["first_frame_timeout_s"],
            "controller.first_frame_timeout_s",
        ),
        stop_timeout_s=_positive_float(
            settings["stop_timeout_s"],
            "controller.stop_timeout_s",
        ),
        fault_damping_duration_s=_nonnegative_float(
            settings["fault_damping_duration_s"],
            "controller.fault_damping_duration_s",
        ),
        model_switch_blend_s=_nonnegative_float(
            settings["model_switch_blend_s"],
            "controller.model_switch_blend_s",
        ),
        motor_indices=motor_indices,
        policy_joint_names=policy_joint_names,
        motor_joint_names=motor_joint_names,
        default_angles=_float_vector(
            settings["default_angles"],
            "controller.default_angles",
            POLICY_JOINT_COUNT,
        ),
        kps=kps,
        kds=kds,
        arm_stand_default_angles=_float_vector(
            settings["arm_stand_default_angles"],
            "controller.arm_stand_default_angles",
            POLICY_JOINT_COUNT,
        ),
        arm_stand_kps=arm_stand_kps,
        arm_stand_kds=arm_stand_kds,
        angular_velocity_scale=_positive_float(
            settings["angular_velocity_scale"],
            "controller.angular_velocity_scale",
        ),
        joint_position_scale=_positive_float(
            settings["joint_position_scale"],
            "controller.joint_position_scale",
        ),
        joint_velocity_scale=_positive_float(
            settings["joint_velocity_scale"],
            "controller.joint_velocity_scale",
        ),
        action_scale=_positive_float(
            settings["action_scale"],
            "controller.action_scale",
        ),
        max_velocity_command=max_velocity_command,
    )


def _mapping(value: object, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    return dict(value)


def _require_keys(
    settings: dict[str, Any],
    expected: set[str],
    label: str,
) -> None:
    actual = set(settings)
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        raise ValueError(f"{label} keys are invalid: missing={missing}, extra={extra}")


def _absolute_topic(value: object, label: str) -> str:
    topic = _nonempty_string(value, label)
    if not topic.startswith("/"):
        raise ValueError(f"{label} must be an absolute ROS topic")
    return topic


def _nonempty_string(value: object, label: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{label} must be a string")
    result = value.strip()
    if not result:
        raise ValueError(f"{label} cannot be empty")
    return result


def _boolean(value: object, label: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{label} must be true or false")
    return value


def _positive_float(value: object, label: str) -> float:
    if not isinstance(value, Real) or isinstance(value, bool):
        raise ValueError(f"{label} must be a number")
    result = float(value)
    if not isfinite(result) or result <= 0.0:
        raise ValueError(f"{label} must be finite and positive")
    return result


def _nonnegative_float(value: object, label: str) -> float:
    if not isinstance(value, Real) or isinstance(value, bool):
        raise ValueError(f"{label} must be a number")
    result = float(value)
    if not isfinite(result) or result < 0.0:
        raise ValueError(f"{label} must be finite and non-negative")
    return result


def _float_vector(
    value: object,
    label: str,
    size: int,
) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{label} must contain exactly {size} values")
    if not all(isinstance(item, Real) and not isinstance(item, bool) for item in value):
        raise ValueError(f"{label} must contain only numbers")
    result = tuple(float(item) for item in value)
    if not all(isfinite(item) for item in result):
        raise ValueError(f"{label} must contain only finite values")
    return result


def _integer_vector(
    value: object,
    label: str,
    size: int,
) -> tuple[int, ...]:
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{label} must contain exactly {size} values")
    if not all(isinstance(item, int) and not isinstance(item, bool) for item in value):
        raise ValueError(f"{label} must contain only integers")
    return tuple(value)


def _string_vector(
    value: object,
    label: str,
    size: int,
) -> tuple[str, ...]:
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{label} must contain exactly {size} values")
    if not all(isinstance(item, str) for item in value):
        raise ValueError(f"{label} must contain only strings")
    result = tuple(item.strip() for item in value)
    if not all(result):
        raise ValueError(f"{label} cannot contain empty names")
    return result
