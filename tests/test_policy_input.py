from pathlib import Path
import os
import sys
import unittest
from unittest.mock import patch

from locomotion_controller.config import MODEL_NAMES, load_config
from locomotion_controller.policy_input import (
    POLICY_INPUT_LAYOUT,
    POLICY_INPUT_SCHEMA,
    make_policy_input_packet,
    validate_policy_input_packet,
)
from locomotion_controller.runtime_client import RuntimeClient


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_FILE = PROJECT_ROOT / "config" / "locomotion_controller.yaml"
TEST_RUNTIME_ENVIRONMENT = {
    "LOCOMOTION_RUNTIME_PYTHON": sys.executable,
    "LOCOMOTION_RUNTIME_HOME": sys.prefix,
    "LOCOMOTION_LOG_ROOT": str(PROJECT_ROOT / "log"),
    "LOCOMOTION_SOCKET_PATH": "/tmp/locomotion-controller-test.sock",
    "LOCOMOTION_NETWORK_INTERFACE": "robot0",
    "LOCOMOTION_ROBOT_IP": "192.0.2.1",
}


def packet_for(model: str, frame: int = 4) -> dict[str, object]:
    with patch.dict(os.environ, TEST_RUNTIME_ENVIRONMENT, clear=False):
        config = load_config(CONFIG_FILE, PROJECT_ROOT)
    is_stand_recovery = model == "stand_recovery"
    command = (0.0, 0.0, 0.0) if is_stand_recovery else (0.1, -0.2, 0.3)
    return make_policy_input_packet(
        frame=frame,
        wall_time="2026-08-12T12:00:00.000000+08:00",
        monotonic_time_s=123.5,
        model=model,
        high_mode=4 if is_stand_recovery else 1,
        low_mode=None if is_stand_recovery else 1,
        standing_transition=False,
        command_semantics="velocity",
        selected_command=command,
        model_command=command,
        policy_joint_names=config.controller.policy_joint_names,
        observation=[index / 100.0 for index in range(96)],
    )


class PolicyInputContractTest(unittest.TestCase):
    def test_one_self_describing_packet_contract_covers_all_models(self):
        for model in MODEL_NAMES:
            with self.subTest(model=model):
                packet = packet_for(model)
                self.assertIs(validate_policy_input_packet(packet), packet)
                self.assertEqual(packet["schema"], POLICY_INPUT_SCHEMA)
                self.assertEqual(packet["stage"], "pre_inference")
                model_input = packet["input"]
                self.assertEqual(model_input["shape"], [1, 96])
                self.assertEqual(model_input["layout"], POLICY_INPUT_LAYOUT)
                self.assertEqual(len(model_input["observation"]), 96)
                self.assertEqual(len(model_input["policy_joint_names"]), 29)

    def test_invalid_observation_is_rejected(self):
        packet = packet_for("free_walk")
        packet["input"]["observation"][-1] = float("nan")
        with self.assertRaisesRegex(RuntimeError, "observation must be finite"):
            validate_policy_input_packet(packet)

    def test_mode_five_free_walk_packet_is_valid(self):
        packet = packet_for("free_walk")
        packet["high_mode"] = 5
        packet["low_mode"] = None
        packet["navigation_input"]["selected"] = [0.0, 0.0, 0.0]
        packet["navigation_input"]["model_input"] = [0.0, 0.0, 0.0]

        self.assertIs(validate_policy_input_packet(packet), packet)

    def test_runtime_client_validates_combined_telemetry(self):
        client = RuntimeClient.__new__(RuntimeClient)
        packet = packet_for("arm_walk", frame=12)
        client._require_success = lambda operation, payload: {
            "positions": [index / 10.0 for index in range(29)],
            "policy_inputs": [packet],
            "first_available_frame": 3,
            "latest_frame": 12,
        }

        telemetry = client.get_control_telemetry(11)

        self.assertEqual(len(telemetry.positions), 29)
        self.assertEqual(telemetry.policy_inputs[0]["frame"], 12)
        self.assertEqual(telemetry.first_available_frame, 3)
        self.assertEqual(telemetry.latest_frame, 12)


if __name__ == "__main__":
    unittest.main()
