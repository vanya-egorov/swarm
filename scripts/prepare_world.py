"""Prepare a local Webots project without modifying the ArduPilot checkout."""

import os
from pathlib import Path

project = Path(__file__).resolve().parent.parent
ardupilot = Path(os.environ.get("ARDUPILOT_DIR", project / "projectData/ardupilot")).resolve()
example = ardupilot / "libraries/SITL/examples/Webots_Python"
proto = example / "protos/Iris.proto"
controllers = example / "controllers"
if not proto.is_file() or not controllers.is_dir():
    raise SystemExit(f"Webots Iris example not found: {example}")

output = Path(os.environ.get("RUN_DIR", project / "projectData/run")).resolve() / "webots"
(output / "worlds").mkdir(parents=True, exist_ok=True)
link = output / "controllers"
if link.is_symlink():
    link.unlink()
link.symlink_to(controllers, target_is_directory=True)
world = output / "worlds/iris-three.wbt"
template = (project / "sim/webots/iris-three.wbt").read_text()
world.write_text(template.replace("@IRIS_PROTO@", proto.as_posix()))
print(f"Откройте в Webots: {world}\nЗапустите моделирование, затем make swarm-start.")
