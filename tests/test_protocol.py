import json
import unittest

from locomotion_controller.protocol import (
    ARM_COMMAND_SCHEMA,
    parse_arm_command,
    parse_navigation_command,
)


class ProtocolTest(unittest.TestCase):
    def test_navigation_requires_exactly_three_finite_values(self):
        self.assertEqual(
            parse_navigation_command([0.1, -0.2, 0.3]),
            (0.1, -0.2, 0.3),
        )
        with self.assertRaises(ValueError):
            parse_navigation_command([0.1, 0.2])
        with self.assertRaises(ValueError):
            parse_navigation_command([0.1, float("nan"), 0.3])

    def test_arm_command_requires_the_complete_schema(self):
        payload = {
            "schema": ARM_COMMAND_SCHEMA,
            "seq": 7,
            "arm_q": [0.1] * 14,
            "arm_dq": [0.2] * 14,
            "weight": 0.8,
        }
        command = parse_arm_command(json.dumps(payload))
        self.assertEqual(command.sequence, 7)
        self.assertEqual(command.positions, (0.1,) * 14)
        self.assertEqual(command.velocities, (0.2,) * 14)
        self.assertEqual(command.weight, 0.8)

        del payload["arm_dq"]
        with self.assertRaises(ValueError):
            parse_arm_command(json.dumps(payload))

    def test_arm_command_rejects_unknown_fields_and_invalid_weight(self):
        payload = {
            "schema": ARM_COMMAND_SCHEMA,
            "seq": 0,
            "arm_q": [0.0] * 14,
            "arm_dq": [0.0] * 14,
            "weight": 1.1,
            "unexpected": True,
        }
        with self.assertRaises(ValueError):
            parse_arm_command(json.dumps(payload))


if __name__ == "__main__":
    unittest.main()
