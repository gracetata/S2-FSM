from importlib.util import find_spec
import os
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_FILE = PROJECT_ROOT / "config" / "locomotion_controller.yaml"


class ProjectConfigTest(unittest.TestCase):
    def test_docs_do_not_bind_deployment_to_a_specific_home_or_nuc(self):
        documentation = [
            PROJECT_ROOT / "README.md",
            *sorted((PROJECT_ROOT / "docs").glob("*.md")),
        ]
        forbidden = (
            "/home/wenduo",
            "/home/hecbot",
            "/home/user",
            "192.168.50.113",
        )
        for path in documentation:
            text = path.read_text(encoding="utf-8")
            for value in forbidden:
                with self.subTest(path=path.name, value=value):
                    self.assertNotIn(value, text)

    def test_portable_default_paths_and_models_resolve(self):
        if find_spec("yaml") is None:
            self.skipTest("PyYAML is required")
        from locomotion_controller.config import load_config

        config = load_config(CONFIG_FILE, PROJECT_ROOT)

        self.assertTrue(config.runtime.python_executable.is_file())
        self.assertTrue(config.runtime.cyclonedds_home.is_dir())
        self.assertEqual(len(config.models), 5)
        self.assertTrue(all(path.is_file() for path in config.models.values()))
        self.assertEqual(
            config.runtime.log_root,
            PROJECT_ROOT / "log",
        )
        self.assertEqual(config.runtime.socket_path.parent, Path("/tmp"))

    def test_each_nuc_can_override_deployment_values_without_editing_yaml(self):
        if find_spec("yaml") is None:
            self.skipTest("PyYAML is required")
        from locomotion_controller.config import load_config

        with TemporaryDirectory() as temporary_directory:
            log_root = Path(temporary_directory) / "runtime-logs"
            environment = {
                "LOCOMOTION_RUNTIME_PYTHON": sys.executable,
                "LOCOMOTION_RUNTIME_HOME": sys.prefix,
                "LOCOMOTION_LOG_ROOT": str(log_root),
                "LOCOMOTION_SOCKET_PATH": "portable.sock",
                "LOCOMOTION_NETWORK_INTERFACE": "robot0",
                "LOCOMOTION_ROBOT_IP": "10.20.30.40",
            }
            with patch.dict(os.environ, environment, clear=False):
                config = load_config(CONFIG_FILE, PROJECT_ROOT)

        self.assertEqual(
            config.runtime.python_executable,
            Path(sys.executable).resolve(),
        )
        self.assertEqual(config.runtime.cyclonedds_home, Path(sys.prefix))
        self.assertEqual(config.runtime.log_root, log_root)
        self.assertEqual(
            config.runtime.socket_path,
            PROJECT_ROOT / "portable.sock",
        )
        self.assertEqual(config.runtime.network_interface, "robot0")
        self.assertEqual(config.runtime.robot_ip, "10.20.30.40")


if __name__ == "__main__":
    unittest.main()
