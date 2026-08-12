import json
from pathlib import Path
import tempfile
import unittest

from locomotion_controller.simulator_presets import (
    ARM_CYCLE_POSE_COUNT,
    KEYBOARD_VELOCITY_DELTAS,
    PRESET_SCHEMA,
    ZERO_COMMAND,
    adjust_keyboard_velocity,
    load_preset_catalog,
    next_arm_cycle_pose_index,
)


PRESET_FILE = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "simulator_presets.json"
)
ARM_WALK_POSE_FILE = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "walk_with_object_arm_pose_set.json"
)


class SimulatorPresetsTest(unittest.TestCase):
    def test_project_presets_are_complete(self):
        catalog = load_preset_catalog(PRESET_FILE)

        self.assertEqual(len(catalog.arm_poses), 4)
        self.assertEqual(len(catalog.velocity_trajectories), 3)
        self.assertEqual(len(catalog.position_targets), 3)
        self.assertTrue(
            all(len(pose.positions) == 14 for pose in catalog.arm_poses)
        )

    def test_first_three_arm_poses_match_arm_walk_model_presets(self):
        catalog = load_preset_catalog(PRESET_FILE)
        source = json.loads(ARM_WALK_POSE_FILE.read_text(encoding="utf-8"))

        expected = tuple(
            tuple(pose["left"] + pose["right"])
            for pose in source["poses"]
        )
        actual = tuple(
            pose.positions for pose in catalog.arm_poses[:3]
        )

        self.assertEqual(actual, expected)
        self.assertEqual(
            tuple(pose.name for pose in catalog.arm_poses[:3]),
            ("back", "down", "front"),
        )

    def test_velocity_trajectory_finishes_at_zero(self):
        trajectory = load_preset_catalog(
            PRESET_FILE
        ).velocity_trajectories[0]

        self.assertEqual(trajectory.sample(0.5), ZERO_COMMAND)
        forward_command = trajectory.sample(1.5)
        self.assertGreater(forward_command[0], 0.0)
        self.assertEqual(forward_command[1:], (0.0, 0.0))
        self.assertEqual(
            trajectory.sample(trajectory.duration_s),
            ZERO_COMMAND,
        )

    def test_keyboard_velocity_keys_adjust_all_axes_and_return_to_zero(self):
        limits = (0.8, 0.5, 1.57)
        command = ZERO_COMMAND
        for positive_key, negative_key in (("w", "s"), ("a", "d"), ("q", "e")):
            increased = adjust_keyboard_velocity(
                command,
                positive_key,
                limits,
            )
            self.assertNotEqual(increased, command)
            self.assertEqual(
                adjust_keyboard_velocity(increased, negative_key, limits),
                ZERO_COMMAND,
            )
        self.assertEqual(
            set(KEYBOARD_VELOCITY_DELTAS),
            {"w", "s", "a", "d", "q", "e"},
        )

    def test_keyboard_velocity_is_clamped_per_axis(self):
        limits = (0.8, 0.5, 1.57)
        self.assertEqual(
            adjust_keyboard_velocity((0.8, 0.5, 1.57), "w", limits),
            (0.8, 0.5, 1.57),
        )
        self.assertEqual(
            adjust_keyboard_velocity((-0.8, -0.5, -1.57), "e", limits),
            (-0.8, -0.5, -1.57),
        )

    def test_space_cycle_uses_only_three_recommended_arm_poses(self):
        self.assertEqual(ARM_CYCLE_POSE_COUNT, 3)
        self.assertEqual(next_arm_cycle_pose_index(0), 1)
        self.assertEqual(next_arm_cycle_pose_index(1), 2)
        self.assertEqual(next_arm_cycle_pose_index(2), 0)
        self.assertEqual(next_arm_cycle_pose_index(3), 0)

    def test_unknown_preset_field_is_rejected(self):
        value = json.loads(PRESET_FILE.read_text(encoding="utf-8"))
        self.assertEqual(value["schema"], PRESET_SCHEMA)
        value["unexpected"] = True
        with tempfile.TemporaryDirectory() as directory:
            invalid_file = Path(directory) / "invalid.json"
            invalid_file.write_text(json.dumps(value), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_preset_catalog(invalid_file)


if __name__ == "__main__":
    unittest.main()
