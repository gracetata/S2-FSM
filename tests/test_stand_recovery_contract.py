from hashlib import sha256
import json
from pathlib import Path
import unittest

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MODEL_FILE = PROJECT_ROOT / "models" / "extreme_stand_recovery.onnx"
FREE_WALK_MODEL_FILE = PROJECT_ROOT / "models" / "free_walk.onnx"
CONTRACT_FILE = (
    PROJECT_ROOT / "models" / "extreme_stand_recovery.contract.json"
)
FREE_WALK_CONTRACT_FILE = PROJECT_ROOT / "models" / "free_walk.contract.json"
CONFIG_FILE = PROJECT_ROOT / "config" / "locomotion_controller.yaml"


class StandRecoveryContractTest(unittest.TestCase):
    def test_free_walk_model_hash_and_static_contract(self):
        contract = json.loads(
            FREE_WALK_CONTRACT_FILE.read_text(encoding="utf-8")
        )
        digest = sha256(FREE_WALK_MODEL_FILE.read_bytes()).hexdigest()

        self.assertEqual(digest, contract["sha256"])
        self.assertEqual(contract["input"], {
            "name": "obs",
            "type": "tensor(float)",
            "shape": [1, 96],
        })
        self.assertEqual(contract["output"], {
            "name": "actions",
            "type": "tensor(float)",
            "shape": [1, 29],
        })
        self.assertEqual(contract["control_frequency_hz"], 50)
        self.assertEqual(contract["action_scale"], 0.25)

    def test_model_hash_and_static_contract(self):
        contract = json.loads(CONTRACT_FILE.read_text(encoding="utf-8"))
        digest = sha256(MODEL_FILE.read_bytes()).hexdigest()

        self.assertEqual(digest, contract["sha256"])
        self.assertEqual(contract["input"], {
            "name": "obs",
            "type": "tensor(float)",
            "shape": [1, 96],
        })
        self.assertEqual(contract["output"], {
            "name": "actions",
            "type": "tensor(float)",
            "shape": [1, 29],
        })
        self.assertEqual(contract["control_frequency_hz"], 50)
        self.assertEqual(contract["action_scale"], 0.25)
        self.assertEqual(contract["command"], [0.0, 0.0, 0.0])

    def test_config_routes_replaced_models_to_packaged_files(self):
        config = yaml.safe_load(
            CONFIG_FILE.read_text(encoding="utf-8")
        )["locomotion_controller"]

        self.assertEqual(
            config["models"]["stand_recovery"],
            "models/extreme_stand_recovery.onnx",
        )
        self.assertEqual(
            config["models"]["free_walk"],
            "models/free_walk.onnx",
        )
        self.assertEqual(config["controller"]["control_dt"], 0.02)
        self.assertEqual(config["controller"]["action_scale"], 0.25)


if __name__ == "__main__":
    unittest.main()
