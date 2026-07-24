"""Strict loader and samplers for the keyboard test presets."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import json
from math import isfinite
from numbers import Real
from pathlib import Path
from typing import Any


PRESET_SCHEMA = "hecbot.locomotion_test_presets.v1"
ARM_JOINTS_PER_SIDE = 7
NAVIGATION_COMMAND_SIZE = 3
ZERO_COMMAND = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class ArmPose:
    name: str
    description_zh: str
    positions: tuple[float, ...]
    weight: float


@dataclass(frozen=True)
class VelocitySegment:
    duration_s: float
    command: tuple[float, float, float]


@dataclass(frozen=True)
class VelocityTrajectory:
    name: str
    description_zh: str
    should_loop: bool
    segments: tuple[VelocitySegment, ...]

    @property
    def duration_s(self) -> float:
        return sum(segment.duration_s for segment in self.segments)

    def sample(self, elapsed_s: float) -> tuple[float, float, float]:
        if elapsed_s < 0.0:
            raise ValueError("trajectory elapsed time cannot be negative")
        if not self.should_loop and elapsed_s >= self.duration_s:
            return ZERO_COMMAND
        sample_time = elapsed_s % self.duration_s
        segment_end = 0.0
        for segment in self.segments:
            segment_end += segment.duration_s
            if sample_time < segment_end:
                return segment.command
        return ZERO_COMMAND


@dataclass(frozen=True)
class PositionTarget:
    name: str
    description_zh: str
    command: tuple[float, float, float]


@dataclass(frozen=True)
class PresetCatalog:
    joint_order_per_arm: tuple[str, ...]
    arm_poses: tuple[ArmPose, ...]
    velocity_trajectories: tuple[VelocityTrajectory, ...]
    position_targets: tuple[PositionTarget, ...]


def load_preset_catalog(path: str | Path) -> PresetCatalog:
    """Load a complete preset file without aliases or implicit values."""

    raw_value = json.loads(Path(path).read_text(encoding="utf-8"))
    root = _mapping(raw_value, "preset root")
    _require_keys(
        root,
        {
            "schema",
            "joint_order_per_arm",
            "arm_poses",
            "velocity_trajectories",
            "position_targets",
        },
        "preset root",
    )
    if root["schema"] != PRESET_SCHEMA:
        raise ValueError(f"preset schema must be {PRESET_SCHEMA}")
    joint_order = _string_vector(
        root["joint_order_per_arm"],
        ARM_JOINTS_PER_SIDE,
        "joint_order_per_arm",
    )
    arm_poses = tuple(
        _load_arm_pose(value, index)
        for index, value in enumerate(
            _nonempty_sequence(root["arm_poses"], "arm_poses")
        )
    )
    velocity_trajectories = tuple(
        _load_velocity_trajectory(value, index)
        for index, value in enumerate(
            _nonempty_sequence(
                root["velocity_trajectories"],
                "velocity_trajectories",
            )
        )
    )
    position_targets = tuple(
        _load_position_target(value, index)
        for index, value in enumerate(
            _nonempty_sequence(root["position_targets"], "position_targets")
        )
    )
    _require_unique_names(arm_poses, "arm_poses")
    _require_unique_names(velocity_trajectories, "velocity_trajectories")
    _require_unique_names(position_targets, "position_targets")
    return PresetCatalog(
        joint_order_per_arm=joint_order,
        arm_poses=arm_poses,
        velocity_trajectories=velocity_trajectories,
        position_targets=position_targets,
    )

def _load_arm_pose(value: object, index: int) -> ArmPose:
    label = f"arm_poses[{index}]"
    pose = _mapping(value, label)
    _require_keys(
        pose,
        {"name", "description_zh", "left", "right", "weight"},
        label,
    )
    left = _finite_vector(
        pose["left"],
        ARM_JOINTS_PER_SIDE,
        f"{label}.left",
    )
    right = _finite_vector(
        pose["right"],
        ARM_JOINTS_PER_SIDE,
        f"{label}.right",
    )
    weight = _finite_float(pose["weight"], f"{label}.weight")
    if not 0.0 <= weight <= 1.0:
        raise ValueError(f"{label}.weight must be within [0, 1]")
    return ArmPose(
        name=_nonempty_string(pose["name"], f"{label}.name"),
        description_zh=_nonempty_string(
            pose["description_zh"],
            f"{label}.description_zh",
        ),
        positions=left + right,
        weight=weight,
    )


def _load_velocity_trajectory(
    value: object,
    index: int,
) -> VelocityTrajectory:
    label = f"velocity_trajectories[{index}]"
    trajectory = _mapping(value, label)
    _require_keys(
        trajectory,
        {"name", "description_zh", "loop", "segments"},
        label,
    )
    should_loop = trajectory["loop"]
    if not isinstance(should_loop, bool):
        raise ValueError(f"{label}.loop must be boolean")
    segments = tuple(
        _load_velocity_segment(segment, label, segment_index)
        for segment_index, segment in enumerate(
            _nonempty_sequence(trajectory["segments"], f"{label}.segments")
        )
    )
    return VelocityTrajectory(
        name=_nonempty_string(trajectory["name"], f"{label}.name"),
        description_zh=_nonempty_string(
            trajectory["description_zh"],
            f"{label}.description_zh",
        ),
        should_loop=should_loop,
        segments=segments,
    )


def _load_velocity_segment(
    value: object,
    trajectory_label: str,
    index: int,
) -> VelocitySegment:
    label = f"{trajectory_label}.segments[{index}]"
    segment = _mapping(value, label)
    _require_keys(segment, {"duration_s", "command"}, label)
    command = _finite_vector(
        segment["command"],
        NAVIGATION_COMMAND_SIZE,
        f"{label}.command",
    )
    return VelocitySegment(
        duration_s=_positive_float(
            segment["duration_s"],
            f"{label}.duration_s",
        ),
        command=(command[0], command[1], command[2]),
    )


def _load_position_target(value: object, index: int) -> PositionTarget:
    label = f"position_targets[{index}]"
    target = _mapping(value, label)
    _require_keys(target, {"name", "description_zh", "command"}, label)
    command = _finite_vector(
        target["command"],
        NAVIGATION_COMMAND_SIZE,
        f"{label}.command",
    )
    return PositionTarget(
        name=_nonempty_string(target["name"], f"{label}.name"),
        description_zh=_nonempty_string(
            target["description_zh"],
            f"{label}.description_zh",
        ),
        command=(command[0], command[1], command[2]),
    )


def _mapping(value: object, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object")
    return value


def _require_keys(
    value: dict[str, Any],
    expected: set[str],
    label: str,
) -> None:
    actual = set(value)
    if actual != expected:
        raise ValueError(
            f"{label} keys are invalid: "
            f"missing={sorted(expected - actual)}, "
            f"extra={sorted(actual - expected)}"
        )


def _nonempty_sequence(value: object, label: str) -> Sequence[object]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) == 0
    ):
        raise ValueError(f"{label} must be a non-empty array")
    return value


def _string_vector(
    value: object,
    size: int,
    label: str,
) -> tuple[str, ...]:
    items = _nonempty_sequence(value, label)
    if len(items) != size:
        raise ValueError(f"{label} must contain exactly {size} values")
    result = tuple(_nonempty_string(item, label) for item in items)
    if len(set(result)) != size:
        raise ValueError(f"{label} values must be unique")
    return result


def _finite_vector(
    value: object,
    size: int,
    label: str,
) -> tuple[float, ...]:
    items = _nonempty_sequence(value, label)
    if len(items) != size:
        raise ValueError(f"{label} must contain exactly {size} values")
    return tuple(_finite_float(item, label) for item in items)


def _finite_float(value: object, label: str) -> float:
    if not isinstance(value, Real) or isinstance(value, bool):
        raise ValueError(f"{label} must contain numbers")
    result = float(value)
    if not isfinite(result):
        raise ValueError(f"{label} must contain finite numbers")
    return result


def _positive_float(value: object, label: str) -> float:
    result = _finite_float(value, label)
    if result <= 0.0:
        raise ValueError(f"{label} must be positive")
    return result


def _nonempty_string(value: object, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label} must be a non-empty string")
    return value


def _require_unique_names(values: Sequence[object], label: str) -> None:
    names = [getattr(value, "name") for value in values]
    if len(set(names)) != len(names):
        raise ValueError(f"{label} names must be unique")
