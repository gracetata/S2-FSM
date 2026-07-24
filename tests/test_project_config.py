from importlib.util import find_spec
from pathlib import Path
import socket
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_FILE = PROJECT_ROOT / "config" / "locomotion_controller.yaml"


class ProjectConfigTest(unittest.TestCase):
    def test_runtime_paths_models_and_network_interface_exist(self):
        if find_spec("yaml") is None:
            self.skipTest("PyYAML is required")
        from locomotion_controller.config import load_config

        config = load_config(CONFIG_FILE, PROJECT_ROOT)

        self.assertTrue(config.runtime.python_executable.is_file())
        self.assertTrue(config.runtime.cyclonedds_home.is_dir())
        self.assertEqual(len(config.models), 4)
        self.assertTrue(all(path.is_file() for path in config.models.values()))
        self.assertEqual(
            config.runtime.log_root,
            PROJECT_ROOT / "log",
        )
        interface_names = {name for _, name in socket.if_nameindex()}
        self.assertIn(config.runtime.network_interface, interface_names)


if __name__ == "__main__":
    unittest.main()
