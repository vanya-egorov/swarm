"""Prepare project-owned Webots files while keeping the ArduPilot checkout read only."""

from pathlib import Path


def prepare_project(run_dir, ardupilot):
    example = ardupilot / "libraries/SITL/examples/Webots_Python"
    proto, controllers = example / "protos/Iris.proto", example / "controllers"
    if not proto.is_file() or not controllers.is_dir():
        raise ValueError(f"Webots Iris example not found: {example}")
    output = run_dir / "webots"
    (output / "worlds").mkdir(parents=True, exist_ok=True)
    link = output / "controllers"
    marker = link / ".swarm-managed"
    if link.is_symlink():
        if link.resolve() != controllers.resolve():
            raise ValueError(f"Unexpected controller symlink: {link}")
        link.unlink()
    elif link.exists() and not marker.is_file():
        raise ValueError(f"Controller directory is not managed by this project: {link}")
    link.mkdir(exist_ok=True)
    marker.touch()
    sources = [item for item in controllers.iterdir() if item.is_dir()]
    validator = Path(__file__).resolve().parents[1] / "sim/webots/controllers/village_validator"
    if validator.is_dir():
        sources.append(validator)
    for source in sources:
        target = link / source.name
        if target.is_symlink() and target.resolve() == source.resolve():
            continue
        if target.exists() or target.is_symlink():
            raise ValueError(f"Controller path already exists: {target}")
        target.symlink_to(source.resolve(), target_is_directory=True)
    return output, proto


def iris(instance, position):
    coordinates = " ".join(f"{value:.8g}" for value in position)
    return f'''Iris {{
  translation {coordinates}
  name "Iris {instance}"
  controller "ardupilot_vehicle_controller"
  controllerArgs [ "--motors" "m1_motor, m2_motor, m3_motor, m4_motor" "--instance" "{instance}" ]
}}
'''
