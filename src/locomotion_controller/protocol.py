"""Message contracts shared by the ROS node and the control runtime."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import json
from math import isfinite
from numbers import Real


ARM_COMMAND_SCHEMA = "hecbot.upper_body_command.v1"
ARM_JOINT_COUNT = 14
NAVIGATION_COMMAND_SIZE = 3


@dataclass(frozen=True)
class ArmCommand:
    sequence: int
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    weight: float

    def to_payload(self) -> dict[str, object]:
        return {
            "sequence": self.sequence,
            "positions": list(self.positions),
            "velocities": list(self.velocities),
            "weight": self.weight,
        }


def parse_navigation_command(values: object) -> tuple[float, float, float]:
    """Parse the three values whose meaning is selected by low-level mode."""

    parsed = _finite_vector(
        values,
        NAVIGATION_COMMAND_SIZE,
        "navigation command",
    )
    return parsed[0], parsed[1], parsed[2]


def parse_arm_command(raw_message: str) -> ArmCommand:
    """Parse the existing manipulation-side JSON without optional fields."""

    value = json.loads(raw_message)
    if not isinstance(value, dict):
        raise ValueError("arm command must be a JSON object")
    expected = {"schema", "seq", "arm_q", "arm_dq", "weight"}
    actual = set(value)
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        raise ValueError(
            f"arm command keys are invalid: missing={missing}, extra={extra}"
        )
    if value["schema"] != ARM_COMMAND_SCHEMA:
        raise ValueError(f"arm command schema must be {ARM_COMMAND_SCHEMA}")
    sequence = value["seq"]
    if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 0:
        raise ValueError("arm command seq must be a non-negative integer")
    positions = _finite_vector(value["arm_q"], ARM_JOINT_COUNT, "arm_q")
    velocities = _finite_vector(value["arm_dq"], ARM_JOINT_COUNT, "arm_dq")
    if not isinstance(value["weight"], Real) or isinstance(value["weight"], bool):
        raise ValueError("arm command weight must be a number")
    weight = float(value["weight"])
    if not isfinite(weight) or not 0.0 <= weight <= 1.0:
        raise ValueError("arm command weight must be within [0, 1]")
    return ArmCommand(
        sequence=sequence,
        positions=positions,
        velocities=velocities,
        weight=weight,
    )


def arm_command_from_payload(value: object) -> ArmCommand:
    """Read the already-normalized IPC representation."""

    if not isinstance(value, dict):
        raise ValueError("IPC arm command must be an object")
    expected = {"sequence", "positions", "velocities", "weight"}
    if set(value) != expected:
        raise ValueError("IPC arm command keys are invalid")
    sequence = value["sequence"]
    if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 0:
        raise ValueError("IPC arm command sequence must be non-negative")
    positions = _finite_vector(value["positions"], ARM_JOINT_COUNT, "positions")
    velocities = _finite_vector(value["velocities"], ARM_JOINT_COUNT, "velocities")
    if not isinstance(value["weight"], Real) or isinstance(value["weight"], bool):
        raise ValueError("IPC arm command weight must be a number")
    weight = float(value["weight"])
    if not isfinite(weight) or not 0.0 <= weight <= 1.0:
        raise ValueError("IPC arm command weight must be within [0, 1]")
    return ArmCommand(sequence, positions, velocities, weight)


def _finite_vector(value: object, size: int, label: str) -> tuple[float, ...]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != size
    ):
        raise ValueError(f"{label} must contain exactly {size} values")
    if not all(isinstance(item, Real) and not isinstance(item, bool) for item in value):
        raise ValueError(f"{label} must contain only numbers")
    result = tuple(float(item) for item in value)
    if not all(isfinite(item) for item in result):
        raise ValueError(f"{label} must contain only finite values")
    return result
