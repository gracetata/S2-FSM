"""Self-describing policy-input packets shared by runtime and ROS 2."""

from __future__ import annotations

from math import isfinite
from numbers import Real
from typing import Any, Sequence

from .config import MODEL_NAMES, POLICY_JOINT_COUNT


POLICY_INPUT_SCHEMA = "hecbot.policy_input.v1"
POLICY_INPUT_SIZE = 96
POLICY_INPUT_LAYOUT = {
    "angular_velocity": [0, 3],
    "gravity": [3, 6],
    "command": [6, 9],
    "joint_position": [9, 38],
    "joint_velocity": [38, 67],
    "previous_action": [67, 96],
}


def make_policy_input_packet(
    *,
    frame: int,
    wall_time: str,
    monotonic_time_s: float,
    model: str,
    high_mode: int | None,
    low_mode: int | None,
    standing_transition: bool,
    command_semantics: str,
    selected_command: Sequence[float],
    model_command: Sequence[float],
    policy_joint_names: Sequence[str],
    observation: Sequence[float],
) -> dict[str, object]:
    """Build the exact, immutable-by-convention packet captured before infer()."""

    packet: dict[str, object] = {
        "schema": POLICY_INPUT_SCHEMA,
        "stage": "pre_inference",
        "frame": frame,
        "wall_time": wall_time,
        "monotonic_time_s": monotonic_time_s,
        "model": model,
        "high_mode": high_mode,
        "low_mode": low_mode,
        "standing_transition": standing_transition,
        "navigation_input": {
            "semantics": command_semantics,
            "selected": list(selected_command),
            "model_input": list(model_command),
        },
        "input": {
            "name": "obs",
            "dtype": "float32",
            "shape": [1, POLICY_INPUT_SIZE],
            "layout": POLICY_INPUT_LAYOUT,
            "policy_joint_names": list(policy_joint_names),
            "observation": list(observation),
        },
    }
    validate_policy_input_packet(packet)
    return packet


def validate_policy_input_packet(packet: object) -> dict[str, object]:
    """Validate a policy-input packet without requiring NumPy or ONNX Runtime."""

    if not isinstance(packet, dict):
        raise RuntimeError("policy input must be a JSON object")
    if packet.get("schema") != POLICY_INPUT_SCHEMA:
        raise RuntimeError("policy input schema is invalid")
    if packet.get("stage") != "pre_inference":
        raise RuntimeError("policy input stage must be pre_inference")
    frame = packet.get("frame")
    if not isinstance(frame, int) or isinstance(frame, bool) or frame < 0:
        raise RuntimeError("policy input frame must be a non-negative integer")
    wall_time = packet.get("wall_time")
    if not isinstance(wall_time, str) or not wall_time:
        raise RuntimeError("policy input wall_time must be a non-empty string")
    _finite_number(packet.get("monotonic_time_s"), "monotonic_time_s")
    if packet.get("model") not in MODEL_NAMES:
        raise RuntimeError("policy input model is invalid")
    _optional_mode(packet.get("high_mode"), "high_mode", {1, 2, 3, 4})
    _optional_mode(packet.get("low_mode"), "low_mode", {1, 2})
    if not isinstance(packet.get("standing_transition"), bool):
        raise RuntimeError("policy input standing_transition must be boolean")

    navigation = packet.get("navigation_input")
    if not isinstance(navigation, dict):
        raise RuntimeError("policy input navigation_input must be an object")
    if not isinstance(navigation.get("semantics"), str):
        raise RuntimeError("policy input command semantics must be a string")
    _finite_vector(navigation.get("selected"), 3, "selected command")
    _finite_vector(navigation.get("model_input"), 3, "model command")

    model_input = packet.get("input")
    if not isinstance(model_input, dict):
        raise RuntimeError("policy input input must be an object")
    if model_input.get("name") != "obs":
        raise RuntimeError("policy input tensor name must be obs")
    if model_input.get("dtype") != "float32":
        raise RuntimeError("policy input dtype must be float32")
    if model_input.get("shape") != [1, POLICY_INPUT_SIZE]:
        raise RuntimeError("policy input shape must be [1, 96]")
    if model_input.get("layout") != POLICY_INPUT_LAYOUT:
        raise RuntimeError("policy input layout is invalid")
    joint_names = model_input.get("policy_joint_names")
    if (
        not isinstance(joint_names, list)
        or len(joint_names) != POLICY_JOINT_COUNT
        or not all(isinstance(name, str) and name for name in joint_names)
        or len(set(joint_names)) != POLICY_JOINT_COUNT
    ):
        raise RuntimeError("policy input must contain 29 unique joint names")
    _finite_vector(
        model_input.get("observation"),
        POLICY_INPUT_SIZE,
        "observation",
    )
    return packet


def _finite_number(value: object, label: str) -> float:
    if (
        not isinstance(value, Real)
        or isinstance(value, bool)
        or not isfinite(float(value))
    ):
        raise RuntimeError(f"policy input {label} must be finite")
    return float(value)


def _finite_vector(value: object, size: int, label: str) -> None:
    if not isinstance(value, list) or len(value) != size:
        raise RuntimeError(f"policy input {label} must contain exactly {size} values")
    for item in value:
        _finite_number(item, label)


def _optional_mode(value: Any, label: str, legal: set[int]) -> None:
    if value is None:
        return
    if not isinstance(value, int) or isinstance(value, bool) or value not in legal:
        raise RuntimeError(f"policy input {label} is invalid")
