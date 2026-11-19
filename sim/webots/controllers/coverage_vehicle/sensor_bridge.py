"""Read real Webots devices on the SITL stepping thread and publish one atomic frame."""

import json
import math
import socket
import uuid


class SensorBridge:
    def __init__(self, robot, gps, imu, system_id, target_port=None, source_port=None):
        self.robot, self.gps, self.imu = robot, gps, imu
        self.system_id = system_id
        self.session, self.sequence, self.last_time = uuid.uuid4().hex, 0, -1.0
        self.warned_packet = self.warned_send = False
        period = int(math.ceil(50 / robot.getBasicTimeStep()) * robot.getBasicTimeStep())
        self.period = period / 1000
        self.lidars = [robot.getDevice(f"coverage lidar {i}") for i in range(4)]
        self.lidar = self.lidars[0]
        self.sonars = [robot.getDevice(f"coverage sonar {i}") for i in range(10)]
        if any(device is None for device in [*self.lidars, *self.sonars]):
            raise RuntimeError("SensorIris with lidar and ten sonars is required")
        for device in self.lidars:
            device.enable(period)
        self.columns = []
        for sector, device in enumerate(self.lidars):
            width = device.getHorizontalResolution()
            for pixel in range(width):
                local_angle = math.atan((2 * (pixel + 0.5) / width - 1) * math.tan(device.getFov() / 2))
                angle = sector * math.pi / 2 + local_angle
                angle = (angle + math.pi) % (2 * math.pi) - math.pi
                self.columns.append((angle, sector, pixel, local_angle))
        self.columns.sort()
        for device in self.sonars:
            device.enable(period)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", source_port or 14800 + system_id))
        self.socket.setblocking(False)
        self.target = ("127.0.0.1", target_port or 14700 + system_id)

    def frame(self):
        now = self.robot.getTime()
        if now < self.last_time or now - self.last_time < self.period - 1e-9:
            return None
        images = [device.getRangeImage() for device in self.lidars]
        if any(image is None for image in images):
            return None
        layers = self.lidar.getNumberOfLayers()
        ranges = []
        for layer in range(layers):
            elevation = self.lidar.getVerticalFov() * (0.5 - layer / (layers - 1)) if layers > 1 else 0
            for _, sector, pixel, local_angle in self.columns:
                depth = images[sector][layer * self.lidars[sector].getHorizontalResolution() + pixel]
                ranges.append(depth / (math.cos(local_angle) * math.cos(elevation)))
        pose, attitude = self.gps.getValues(), self.imu.getRollPitchYaw()
        sonar_ranges = [device.getValue() for device in self.sonars]
        if ranges is None or any(math.isnan(value) or value < 0 for value in ranges):
            return None
        if any(not math.isfinite(value) for value in [*pose, *attitude, *sonar_ranges]):
            return None
        minimum, maximum = self.lidar.getMinRange(), self.lidar.getMaxRange()
        encoded_ranges = [-1 if math.isinf(value) or value > maximum else round(value, 2) for value in ranges]
        if any(value != -1 and not minimum <= value <= maximum for value in encoded_ranges):
            return None
        sonars = []
        for i, distance in enumerate(sonar_ranges):
            bearing = i * math.pi / 4 if i < 8 else 0
            sonars.append({
                "BearingRad": bearing,
                "ElevationRad": math.pi / 2 if i == 8 else -math.pi / 2 if i == 9 else 0,
                "ForwardM": 0.3 * math.cos(bearing) if i < 8 else 0,
                "RightM": 0.3 * math.sin(bearing) if i < 8 else 0,
                "UpM": 0.15 if i == 8 else -0.04 if i == 9 else 0,
                "RangeM": distance, "MaxRangeM": 6,
            })
        self.sequence += 1
        self.last_time = now
        return {
            "Version": 1, "SystemID": self.system_id, "Session": self.session,
            "Sequence": self.sequence, "TimeS": now,
            "Pose": {"NorthM": pose[0], "EastM": -pose[1], "UpM": pose[2],
                     "RollRad": attitude[0], "PitchRad": -attitude[1], "YawRad": -attitude[2]},
            "Lidar": {"Resolution": len(self.columns),
                      "Layers": self.lidar.getNumberOfLayers(),
                      "MinRangeM": minimum, "MaxRangeM": maximum,
                      "VerticalFOVRad": self.lidar.getVerticalFov(), "HeightM": 0.18,
                      "BearingsRad": [round(angle, 6) for angle, _, _, _ in self.columns],
                      "RangesM": encoded_ranges},
            "Sonars": sonars,
        }

    def publish(self):
        frame = self.frame()
        if frame is None:
            return
        packet = json.dumps(frame, allow_nan=False, separators=(",", ":")).encode()
        if len(packet) > 8192:
            if not self.warned_packet:
                print(f"sensor packet too large: {len(packet)} bytes", flush=True)
                self.warned_packet = True
            return  # Receiver freshness fails safely; the simulator stepping thread remains alive.
        try:
            self.socket.sendto(packet, self.target)
        except OSError as error:
            if not self.warned_send:
                print(f"sensor datagram failed: {error}", flush=True)
                self.warned_send = True
            # A lost sensor datagram must never interrupt the physics/SITL stepping thread.

    def close(self):
        self.socket.close()
