from pathlib import Path
import unittest

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_FILE = PROJECT_ROOT / "config" / "locomotion_controller.yaml"

EXPECTED_MOTOR_TO_POLICY = (
    0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10,
    16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28,
)
EXPECTED_POLICY_TO_MOTOR = (
    0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18, 2, 5, 8,
    11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28,
)


class JointOrderMappingTest(unittest.TestCase):
    def test_documented_permutations_match_configured_joint_names(self):
        controller = yaml.safe_load(
            CONFIG_FILE.read_text(encoding="utf-8")
        )["locomotion_controller"]["controller"]
        policy_names = controller["policy_joint_names"]
        motor_names = controller["motor_joint_names"]

        motor_to_policy = tuple(
            motor_names.index(name) for name in policy_names
        )
        policy_to_motor = tuple(
            policy_names.index(name) for name in motor_names
        )

        self.assertEqual(motor_to_policy, EXPECTED_MOTOR_TO_POLICY)
        self.assertEqual(policy_to_motor, EXPECTED_POLICY_TO_MOTOR)
        self.assertEqual(len(set(policy_names)), 29)
        self.assertEqual(set(policy_names), set(motor_names))

    def test_right_ankle_indices_are_explicit(self):
        controller = yaml.safe_load(
            CONFIG_FILE.read_text(encoding="utf-8")
        )["locomotion_controller"]["controller"]
        policy_names = controller["policy_joint_names"]
        motor_names = controller["motor_joint_names"]

        self.assertEqual(policy_names.index("right_ankle_pitch_joint"), 14)
        self.assertEqual(policy_names.index("right_ankle_roll_joint"), 18)
        self.assertEqual(motor_names.index("right_ankle_pitch_joint"), 10)
        self.assertEqual(motor_names.index("right_ankle_roll_joint"), 11)


if __name__ == "__main__":
    unittest.main()
