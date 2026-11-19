"""Attach sensors to the existing village.wbt without generating another world."""

import argparse
import os
from pathlib import Path
import re
import shutil

SOURCE = Path(__file__).resolve().parent
PROJECT = SOURCE.parents[1]
NODE = re.compile(r"(?m)^([ \t]*(?:DEF\s+\w+\s+)?)(?:SensorIris|Iris)(\s*\{)")
PROTO = re.compile(r'(?m)^EXTERNPROTO\s+"[^"\n]*/(?:SensorIris|Iris)\.proto"[ \t]*$')
CONTROLLER = re.compile(r'\bcontroller\s+"(?:ardupilot_vehicle_controller|swarm_ardupilot_vehicle_controller|coverage_vehicle)"')


def prepare_project(run_dir, ardupilot):
    example = ardupilot / "libraries/SITL/examples/Webots_Python"
    vendor = example / "protos/Iris.proto"
    if not vendor.is_file() or not (example / "controllers").is_dir():
        raise ValueError(f"Webots Iris example not found: {example}")
    output = run_dir / "webots"
    controllers = output / "controllers"
    marker = controllers / ".swarm-managed"
    if controllers.is_symlink():
        if controllers.resolve() != (example / "controllers").resolve():
            raise ValueError(f"Unexpected controller symlink: {controllers}")
    elif controllers.exists() and not marker.is_file():
        raise ValueError(f"Controller directory is not managed by this project: {controllers}")
    target = controllers / "coverage_vehicle"
    source = SOURCE / "controllers/coverage_vehicle"
    if target.is_symlink():
        if target.resolve() != source.resolve():
            raise ValueError(f"Foreign controller: {target}")
    elif target.exists():
        raise ValueError(f"Foreign controller: {target}")
    protos = output / "protos"
    if protos.is_symlink():
        raise ValueError(f"Unexpected proto symlink: {protos}")
    for name in ("SwarmSensors.proto", "SensorIris.proto"):
        if (protos / name).is_symlink():
            raise ValueError(f"Unexpected proto symlink: {protos / name}")
    if controllers.is_symlink():
        controllers.unlink()
    controllers.mkdir(parents=True, exist_ok=True)
    marker.touch()
    if not target.is_symlink():
        target.symlink_to(source, target_is_directory=True)
    protos.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(SOURCE / "protos/SwarmSensors.proto", protos / "SwarmSensors.proto")
    template = (SOURCE / "protos/SensorIris.proto.in").read_text()
    proto = protos / "SensorIris.proto"
    proto.write_text(template.replace("@IRIS_PROTO@", vendor.resolve().as_posix()))
    (output / "worlds").mkdir(exist_ok=True)
    return output, proto


def configured_text(world):
    if world.name != "village.wbt" or world.is_symlink():
        raise ValueError("Only the existing village.wbt can be configured")
    text = world.read_text()
    if len(NODE.findall(text)) != 3 or len(CONTROLLER.findall(text)) != 3 or len(PROTO.findall(text)) != 1:
        raise ValueError("Expected three Iris with known controllers and one Iris EXTERNPROTO")
    text = PROTO.sub('EXTERNPROTO "../protos/SensorIris.proto"', text)
    text = NODE.sub(r"\1SensorIris\2", text)
    return CONTROLLER.sub('controller "coverage_vehicle"', text)


def configure(run_dir, ardupilot, check=False):
    world = run_dir / "webots/worlds/village.wbt"
    text = configured_text(world)
    if check:
        if world.read_text() != text:
            raise ValueError("Sensor configuration is missing; run make village-sensors")
        output = run_dir / "webots"
        expected = (SOURCE / "protos/SensorIris.proto.in").read_text().replace(
            "@IRIS_PROTO@", (ardupilot / "libraries/SITL/examples/Webots_Python/protos/Iris.proto").resolve().as_posix())
        if (output / "protos/SensorIris.proto").read_text() != expected:
            raise ValueError("SensorIris proto is outdated")
        if (output / "protos/SwarmSensors.proto").read_bytes() != (SOURCE / "protos/SwarmSensors.proto").read_bytes():
            raise ValueError("SwarmSensors proto is outdated")
        if (output / "controllers/coverage_vehicle").resolve() != (SOURCE / "controllers/coverage_vehicle").resolve():
            raise ValueError("Sensor controller is missing")
        if not (ardupilot / "libraries/SITL/examples/Webots_Python/protos/Iris.proto").is_file():
            raise ValueError("Upstream Iris proto is missing")
    else:
        prepare_project(run_dir, ardupilot)
        if world.read_text() != text:
            world.write_text(text)
    return world


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="validate the current world without writing files")
    args = parser.parse_args()
    run = Path(os.environ.get("RUN_DIR", PROJECT / "projectData/run")).resolve()
    ardupilot = Path(os.environ.get("ARDUPILOT_DIR", PROJECT / "projectData/ardupilot")).resolve()
    try:
        world = configure(run, ardupilot, args.check)
    except (OSError, ValueError) as error:
        parser.exit(1, f"{error}\n")
    print(f"{'Проверен' if args.check else 'Настроен'} единственный мир: {world}")


if __name__ == "__main__":
    main()
