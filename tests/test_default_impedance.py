from pathlib import Path
import unittest

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_IMPEDANCE_FILE = PROJECT_ROOT / "impedancepara_default.yaml"


class DefaultImpedanceTest(unittest.TestCase):
    def test_all_modes_use_identical_default_parameters(self):
        parameters = yaml.safe_load(
            DEFAULT_IMPEDANCE_FILE.read_text(encoding="utf-8")
        )

        self.assertEqual(
            set(parameters),
            {
                "default_angles",
                "kps",
                "kds",
                "default_angles_standing_grasp",
                "kps_standing_grasp",
                "kds_standing_grasp",
            },
        )
        self.assertTrue(
            all(len(values) == 29 for values in parameters.values())
        )
        self.assertEqual(
            parameters["default_angles_standing_grasp"],
            parameters["default_angles"],
        )
        self.assertEqual(
            parameters["kps_standing_grasp"],
            parameters["kps"],
        )
        self.assertEqual(
            parameters["kds_standing_grasp"],
            parameters["kds"],
        )

    def test_default_impedance_file_is_installed(self):
        cmake = (PROJECT_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("impedancepara_default.yaml", cmake)


if __name__ == "__main__":
    unittest.main()
