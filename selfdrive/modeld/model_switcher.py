import hashlib
import os
import shutil
from openpilot.common.params import Params

OPENPILOT_PATH = "/data/openpilot"
DESTINATION_PATH = os.path.join(OPENPILOT_PATH, "selfdrive/modeld/models")
MODELS_SOURCE = os.path.join(DESTINATION_PATH, "models")
THNEED_FILE = os.path.join(DESTINATION_PATH, "supercombo.thneed")

MODEL_NAME = {
  0: "night-strike",
  1: "B4+B0",
  2: "farmville",
  3: "new-lemon-pie",
  4: "non-inflatable",
  5: "optimus-prime",
}

def set_model_list_parameter(params):
  """Create a string of all the model names for future comparisons."""

  # Retrieve the previous model list
  previous_model_list = (params.get("ModelList", "") or b"").decode('utf-8')

  # Create a new model list
  model_list = "".join(MODEL_NAME.values())

  if previous_model_list != model_list:
    # Reset the selected model if the model list changed
    params.put_int("Model", 0)
    params.put("ModelList".encode('utf-8'), model_list)

def onnx_already_set(path1, path2):
  """Check if the two files are identical by comparing their SHA-256 hashes."""
  with open(path1, 'rb') as f1, open(path2, 'rb') as f2:
    return hashlib.sha256(f1.read()).hexdigest() == hashlib.sha256(f2.read()).hexdigest()

def copy_model_variant(params):
  # Get the corresponding supercombo variant name
  variant = MODEL_NAME.get(params.get_int("Model"), MODEL_NAME[0])

  # Copy the variant .onnx file to supercombo.onnx in the destination models folder
  onnx_path = os.path.join(MODELS_SOURCE, f"{variant}.onnx")
  destination = os.path.join(DESTINATION_PATH, "supercombo.onnx")
  if not os.path.exists(destination) or not onnx_already_set(onnx_path, destination):
    # Delete the thneed file
    if os.path.exists(THNEED_FILE):
      os.remove(THNEED_FILE)

    # Copy over the onnx file
    shutil.copy(onnx_path, destination)

    # Reset the calibration
    params.remove("CalibrationParams")
    params.remove("LiveTorqueParameters")

if __name__ == "__main__":
  params = Params()

  set_model_list_parameter(params)
  copy_model_variant(params)
