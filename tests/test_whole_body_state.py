from pathlib import Path
import unittest
from unittest.mock import MagicMock

from locomotion_controller.config import (
    POLICY_JOINT_COUNT,
    WHOLE_BODY_JOINT_NAMES,
    load_config,
)
from locomotion_controller.runtime_client import RuntimeClient


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_FILE = PROJECT_ROOT / "config" / "locomotion_controller.yaml"


class WholeBodyStateContractTest(unittest.TestCase):
    def test_config_uses_required_joint_order_and_impedance_file(self):
        config = load_config(CONFIG_FILE, PROJECT_ROOT)

        self.assertEqual(
            config.topics.whole_body_state,
            "/hecbot/whole_body_state",
        )
        self.assertEqual(
            config.controller.motor_joint_names,
            WHOLE_BODY_JOINT_NAMES,
        )
        self.assertEqual(
            config.controller.impedance_file,
            PROJECT_ROOT / "impedancepara.yaml",
        )
        self.assertNotEqual(
            config.controller.kps,
            config.controller.arm_stand_kps,
        )
        self.assertNotEqual(
            config.controller.kds,
            config.controller.arm_stand_kds,
        )

    def test_runtime_client_accepts_exactly_29_finite_positions(self):
        client = RuntimeClient.__new__(RuntimeClient)
        expected = [index / 10.0 for index in range(POLICY_JOINT_COUNT)]
        client._require_success = MagicMock(
            return_value={
                "success": True,
                "positions": expected,
            }
        )

        actual = client.get_whole_body_positions()

        self.assertEqual(actual, tuple(expected))
        client._require_success.assert_called_once_with("whole_body_state")

    def test_runtime_client_rejects_invalid_position_payload(self):
        client = RuntimeClient.__new__(RuntimeClient)
        client._require_success = MagicMock(
            return_value={
                "success": True,
                "positions": [0.0] * (POLICY_JOINT_COUNT - 1),
            }
        )
        with self.assertRaisesRegex(RuntimeError, "exactly 29"):
            client.get_whole_body_positions()

        client._require_success = MagicMock(
            return_value={
                "success": True,
                "positions": [0.0] * (POLICY_JOINT_COUNT - 1)
                + [float("nan")],
            }
        )
        with self.assertRaisesRegex(RuntimeError, "non-finite"):
            client.get_whole_body_positions()


if __name__ == "__main__":
    unittest.main()
