from importlib.util import find_spec
from pathlib import Path
import unittest


MODEL_DIRECTORY = Path(__file__).resolve().parents[1] / "models"
EXPECTED_MODEL_NAMES = {
    "accurate_arrival.onnx",
    "free_walk.onnx",
    "extreme_stand_recovery.onnx",
    "standing_grasp.onnx",
    "walk_with_object.onnx",
}


class ModelInferenceTest(unittest.TestCase):
    def test_all_five_models_accept_one_observation(self):
        if find_spec("numpy") is None or find_spec("onnxruntime") is None:
            self.skipTest("NumPy and ONNX Runtime are required")
        import numpy as np
        import onnxruntime as ort

        model_paths = tuple(sorted(MODEL_DIRECTORY.glob("*.onnx")))
        self.assertEqual(
            {path.name for path in model_paths},
            EXPECTED_MODEL_NAMES,
        )
        observation = np.zeros((1, 96), dtype=np.float32)
        for model_path in model_paths:
            with self.subTest(model=model_path.name):
                session = ort.InferenceSession(
                    str(model_path),
                    providers=["CPUExecutionProvider"],
                )
                input_name = session.get_inputs()[0].name
                outputs = session.run(None, {input_name: observation})
                self.assertEqual(len(outputs), 1)
                self.assertEqual(outputs[0].shape, (1, 29))


if __name__ == "__main__":
    unittest.main()
