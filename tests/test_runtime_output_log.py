from io import BytesIO, StringIO
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

from locomotion_controller.runtime_client import RuntimeClient


class RuntimeOutputLogTest(unittest.TestCase):
    def test_runtime_output_is_written_to_file_and_forwarded_to_terminal(self):
        client = RuntimeClient.__new__(RuntimeClient)
        client._output_error = None
        process = SimpleNamespace(stdout=BytesIO(b"first line\nsecond line\n"))

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "runtime.log"
            output_file = path.open("x", encoding="utf-8", buffering=1)
            terminal = StringIO()
            with patch("sys.stdout", terminal):
                client._forward_runtime_output(process, output_file)
            saved = path.read_text(encoding="utf-8")

        self.assertEqual(saved, "first line\nsecond line\n")
        self.assertEqual(terminal.getvalue(), saved)
        self.assertIsNone(client._output_error)


if __name__ == "__main__":
    unittest.main()
