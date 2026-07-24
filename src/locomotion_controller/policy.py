"""Preloaded ONNX policy sessions."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import onnxruntime as ort

from .config import MODEL_NAMES, POLICY_JOINT_COUNT


OBSERVATION_SIZE = 96


class Policy:
    def __init__(self, model_path: Path) -> None:
        self._session = ort.InferenceSession(
            str(model_path),
            providers=["CPUExecutionProvider"],
        )
        inputs = self._session.get_inputs()
        outputs = self._session.get_outputs()
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError(f"{model_path} must have one input and one output")
        model_input = inputs[0]
        model_output = outputs[0]
        expected_input = ("obs", "tensor(float)", [1, OBSERVATION_SIZE])
        actual_input = (model_input.name, model_input.type, model_input.shape)
        if actual_input != expected_input:
            raise ValueError(
                f"{model_path} input is {actual_input}, expected {expected_input}"
            )
        expected_output = ("actions", "tensor(float)", [1, POLICY_JOINT_COUNT])
        actual_output = (model_output.name, model_output.type, model_output.shape)
        if actual_output != expected_output:
            raise ValueError(
                f"{model_path} output is {actual_output}, expected {expected_output}"
            )
        self._input_name = model_input.name
        self._output_name = model_output.name

    def infer(self, observation: np.ndarray) -> np.ndarray:
        value = np.asarray(observation, dtype=np.float32)
        if value.shape != (OBSERVATION_SIZE,) or not np.all(np.isfinite(value)):
            raise RuntimeError("policy observation must contain 96 finite values")
        result = self._session.run(
            [self._output_name],
            {self._input_name: np.ascontiguousarray(value.reshape(1, -1))},
        )[0]
        action = np.asarray(result, dtype=np.float32).reshape(-1)
        if action.shape != (POLICY_JOINT_COUNT,) or not np.all(
            np.isfinite(action)
        ):
            raise RuntimeError("policy output must contain 29 finite values")
        return action


class PolicyBank:
    """Load and warm all four models before any Unitree command is sent."""

    def __init__(self, model_paths: dict[str, Path]) -> None:
        if set(model_paths) != set(MODEL_NAMES):
            raise ValueError("policy bank requires exactly the four configured models")
        self._policies = {
            model_name: Policy(model_paths[model_name])
            for model_name in MODEL_NAMES
        }
        zero_observation = np.zeros(OBSERVATION_SIZE, dtype=np.float32)
        for policy in self._policies.values():
            policy.infer(zero_observation)

    def get_policy(self, model_name: str) -> Policy:
        try:
            return self._policies[model_name]
        except KeyError as error:
            raise ValueError(f"unknown policy: {model_name}") from error
