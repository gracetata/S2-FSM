import json
from pathlib import Path
import tempfile
import unittest

from locomotion_controller.totarget_logger import LOG_SCHEMA, ToTargetLogger


class ToTargetLoggerTest(unittest.TestCase):
    def test_session_contains_header_every_frame_and_footer(self):
        with tempfile.TemporaryDirectory() as directory:
            logger = ToTargetLogger(Path(directory))
            path = logger.start_session(
                (0.5, -0.2, 0.1),
                {
                    "model": "accurate_arrival",
                    "model_sha256": "abc123",
                },
            )
            logger.write_frame(
                {
                    "observation": [0.0] * 96,
                    "model_output": [0.0] * 29,
                }
            )
            logger.write_frame(
                {
                    "observation": [1.0] * 96,
                    "model_output": [1.0] * 29,
                }
            )
            logger.end_session("switched_to_free_walk")
            logger.close()

            records = [
                json.loads(line)
                for line in path.read_text(encoding="utf-8").splitlines()
            ]

        self.assertIn("dx_+0.500000_dy_-0.200000_yaw_+0.100000", path.name)
        self.assertEqual([record["event"] for record in records], [
            "session_start",
            "frame",
            "frame",
            "session_end",
        ])
        self.assertTrue(all(record["schema"] == LOG_SCHEMA for record in records))
        self.assertEqual(records[0]["initial_target"], [0.5, -0.2, 0.1])
        self.assertEqual(records[1]["session_frame"], 0)
        self.assertEqual(records[2]["session_frame"], 1)
        self.assertEqual(records[-1]["frame_count"], 2)
        self.assertEqual(records[-1]["reason"], "switched_to_free_walk")


if __name__ == "__main__":
    unittest.main()
