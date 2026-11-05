"""Prepare a local Webots project without modifying the ArduPilot checkout."""

import os
from pathlib import Path

from webots_project import prepare_project

project = Path(__file__).resolve().parents[1]
ardupilot = Path(os.environ.get("ARDUPILOT_DIR", project / "projectData/ardupilot")).resolve()
run = Path(os.environ.get("RUN_DIR", project / "projectData/run")).resolve()
output, proto = prepare_project(run, ardupilot)
world = output / "worlds/iris-three.wbt"
template = (project / "sim/webots/iris-three.wbt").read_text()
world.write_text(template.replace("@IRIS_PROTO@", proto.as_posix()))
print(f"Откройте в Webots: {world}\nЗапустите моделирование, затем make swarm-start.")
