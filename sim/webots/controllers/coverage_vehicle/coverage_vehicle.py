"""Extend the upstream Webots/SITL bridge without changing the ArduPilot checkout."""

import json
import os
from pathlib import Path
import sys
import time
import select
import socket
import struct
import math


def latest_controls(channel, address, size, packet_format):
    latest = None
    # The producer can keep sending while we drain. Bounded work guarantees the next physics step.
    for _ in range(256):
        if not select.select([channel], [], [], 0)[0]:
            break
        data, sender = channel.recvfrom(512)
        if sender == address and len(data) == size:
            values = struct.unpack(packet_format, data)
            if all(math.isfinite(value) for value in values):
                latest = tuple(max(0, min(1, value)) for value in values)
    return latest


def main():
    project = Path(__file__).resolve().parents[4]
    ardupilot = Path(os.environ.get("ARDUPILOT_DIR", project / "projectData/ardupilot")).resolve()
    webots_home = Path(os.environ.get("WEBOTS_HOME", "/Applications/Webots.app"))
    for path in (webots_home / "Contents/lib/controller/python", webots_home / "lib/controller/python"):
        if (path / "controller").is_dir():
            sys.path.insert(0, str(path))
            break
    vendor = ardupilot / "libraries/SITL/examples/Webots_Python/controllers/ardupilot_vehicle_controller"
    sys.path.insert(0, str(vendor))
    from ardupilot_vehicle_controller import get_args
    from webots_vehicle import WebotsArduVehicle
    from sensor_bridge import SensorBridge

    class CoverageVehicle(WebotsArduVehicle):
        def _handle_sitl(self, *args):
            settings = json.loads(self.robot.getCustomData() or "{}")
            system_id = settings.get("system_id", self._instance + 1)
            self.sensor_bridge = SensorBridge(self.robot, self.gps, self.imu, system_id)
            channel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            channel.bind(("127.0.0.1", 9002 + self._instance * 10))
            channel.setblocking(False)
            address = (args[0], args[1] + 1)
            try:
                # One physics step per feedback exchange. Rendering must not build a queue
                # of old motor commands that would destabilize the attitude controller.
                deadline = time.monotonic() + 30
                if self.robot.step(self._timestep) == -1:
                    return
                while True:
                    channel.sendto(self._get_fdm_struct(), address)
                    while not select.select([channel], [], [], 0.05)[0]:
                        if time.monotonic() > deadline:
                            raise RuntimeError("SITL motor feedback timed out")
                        channel.sendto(self._get_fdm_struct(), address)
                    latest = latest_controls(channel, address, self.controls_struct_size, self.controls_struct_format)
                    if latest is None:
                        continue
                    deadline = time.monotonic() + 2
                    self._handle_controls(latest)
                    if self.robot.step(self._timestep) == -1:
                        break
            finally:
                channel.close()
                self.sensor_bridge.close()
                self._webots_connected = False

        def _get_fdm_struct(self):
            self.sensor_bridge.publish()
            return super()._get_fdm_struct()

    args = get_args()
    logs = project / "projectData/run/logs"
    logs.mkdir(parents=True, exist_ok=True)
    log = (logs / f"coverage-controller-I{args.instance}.log").open("a", buffering=1)
    sys.stdout = sys.stderr = log
    vehicle = CoverageVehicle(
        motor_names=[name.strip() for name in args.motors.split(",")],
        reversed_motors=[int(value) for value in args.reversed_motors.split(",")] if args.reversed_motors else [],
        instance=args.instance, sitl_address=args.sitl_address,
        motor_velocity_cap=args.motor_cap, bidirectional_motors=args.bidirectional_motors,
        uses_propellers=args.uses_propellers,
    )
    while vehicle.webots_connected():
        time.sleep(0.1)


if __name__ == "__main__":
    main()
