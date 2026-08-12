from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
LOADER = PROJECT_ROOT / "config" / "load_nuc_env.sh"


class NucEnvironmentLoaderTest(unittest.TestCase):
    def test_missing_nuc_env_fails_before_launch(self):
        with tempfile.TemporaryDirectory() as directory:
            config_directory = Path(directory) / "config"
            config_directory.mkdir()
            shutil.copy2(LOADER, config_directory / LOADER.name)

            result = subprocess.run(
                [
                    "bash",
                    "-c",
                    f'source "{config_directory / LOADER.name}"',
                ],
                capture_output=True,
                text=True,
                check=False,
            )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("missing", result.stderr)
        self.assertIn("nuc.env", result.stderr)

    def test_valid_nuc_env_is_exported_and_checked(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_directory = root / "config"
            runtime_home = root / "runtime"
            log_root = root / "logs"
            config_directory.mkdir()
            runtime_home.mkdir()
            shutil.copy2(LOADER, config_directory / LOADER.name)
            fake_python = runtime_home / "python"
            fake_python.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
            fake_python.chmod(0o755)
            (config_directory / "nuc.env").write_text(
                "\n".join(
                    (
                        f'LOCOMOTION_RUNTIME_PYTHON="{fake_python}"',
                        f'LOCOMOTION_RUNTIME_HOME="{runtime_home}"',
                        'LOCOMOTION_NETWORK_INTERFACE="robot0"',
                        'LOCOMOTION_ROBOT_IP="192.0.2.1"',
                        f'LOCOMOTION_LOG_ROOT="{log_root}"',
                        'LOCOMOTION_SOCKET_PATH="/tmp/s2-fsm-test.sock"',
                    )
                ),
                encoding="utf-8",
            )

            result = subprocess.run(
                [
                    "bash",
                    "-c",
                    (
                        f'source "{config_directory / LOADER.name}" && '
                        'test "$LOCOMOTION_NETWORK_INTERFACE" = robot0'
                    ),
                ],
                capture_output=True,
                text=True,
                check=False,
            )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("NUC environment ready", result.stdout)


if __name__ == "__main__":
    unittest.main()
