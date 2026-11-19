"""Compatibility helpers for the single village world exporter."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim/webots"))
from configure_sensors import prepare_project


def iris(instance, position):
    coordinates = " ".join(f"{value:.8g}" for value in position)
    return f'''SensorIris {{
  translation {coordinates}
  name "Iris {instance}"
  controller "coverage_vehicle"
  controllerArgs [ "--motors" "m1_motor, m2_motor, m3_motor, m4_motor" "--instance" "{instance}" ]
}}
'''
