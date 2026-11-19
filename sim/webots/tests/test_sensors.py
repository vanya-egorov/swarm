"""Sensor bridge, motor exchange and configuration of the existing village world."""

import json
import math
from pathlib import Path
import sys
import struct
import tempfile
import unittest
from unittest.mock import MagicMock, patch

WEBOTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WEBOTS))
sys.path.insert(0, str(WEBOTS / "controllers/coverage_vehicle"))
from configure_sensors import configure, configured_text, prepare_project
from sensor_bridge import SensorBridge
from coverage_vehicle import latest_controls


class SensorTests(unittest.TestCase):
    def setUp(self):
        self.socket_patch = patch("sensor_bridge.socket.socket")
        self.socket = self.socket_patch.start().return_value
        self.addCleanup(self.socket_patch.stop)
        self.robot = MagicMock()
        self.robot.getBasicTimeStep.return_value = 2
        self.robot.getTime.return_value = 1
        self.lidar = MagicMock()
        self.lidar.getRangeImage.return_value = [2.500190973] * 144
        self.lidar.getMinRange.return_value = 0.12
        self.lidar.getMaxRange.return_value = 12
        self.lidar.getNumberOfLayers.return_value = 3
        self.lidar.getHorizontalResolution.return_value = 48
        self.lidar.getFov.return_value = math.pi / 2
        self.lidar.getVerticalFov.return_value = 0.22
        self.sonar = MagicMock()
        self.sonar.getValue.return_value = 6
        self.robot.getDevice.side_effect = lambda name: self.lidar if name.startswith("coverage lidar") else self.sonar
        gps, imu = MagicMock(), MagicMock()
        gps.getValues.return_value = [4, -6, 0.06]
        imu.getRollPitchYaw.return_value = [0.1, 0.2, 0.3]
        self.bridge = SensorBridge(self.robot, gps, imu, 1)

    def test_units_frame_transform_and_macos_packet_size(self):
        self.bridge.publish()
        packet, target = self.socket.sendto.call_args.args
        frame = json.loads(packet)
        self.assertLessEqual(len(packet), 8192)
        self.assertEqual(target, ("127.0.0.1", 14701))
        self.assertEqual(frame["Pose"]["EastM"], 6)
        self.assertEqual(frame["Pose"]["YawRad"], -0.3)
        self.assertGreaterEqual(frame["Lidar"]["RangesM"][0], 2.5)
        self.assertEqual(len(frame["Sonars"]), 10)
        self.assertAlmostEqual(frame["Sonars"][2]["BearingRad"], math.pi / 2)

    def test_paused_invalid_and_missing_measurements_never_renew_freshness(self):
        self.assertIsNotNone(self.bridge.frame())
        self.assertIsNone(self.bridge.frame())
        self.robot.getTime.return_value = 1.1
        self.lidar.getRangeImage.return_value = [math.nan] * 144
        self.assertIsNone(self.bridge.frame())
        self.lidar.getRangeImage.return_value = None
        self.assertIsNone(self.bridge.frame())
        self.lidar.getRangeImage.return_value = [math.inf] * 144
        frame = self.bridge.frame()
        self.assertEqual(frame["Lidar"]["RangesM"], [-1] * 576)

    def test_network_error_does_not_stop_sitl_physics(self):
        self.socket.sendto.side_effect = OSError("network unavailable")
        self.bridge.publish()


class ProjectTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.ardupilot = self.root / "ardupilot"
        self.example = self.ardupilot / "libraries/SITL/examples/Webots_Python"
        vendor = self.example / "protos/Iris.proto"
        vendor.parent.mkdir(parents=True)
        vendor.write_text("original Iris")
        (self.example / "controllers").mkdir()
        self.run = self.root / "run"
        self.world = self.run / "webots/worlds/village.wbt"
        self.world.parent.mkdir(parents=True)
        self.original = '#VRML_SIM R2025b utf8\nEXTERNPROTO "../Iris.proto"\nSolid { name "map" }\n'
        for instance in range(3):
            self.original += f'Iris {{\n  translation {12 * instance} 0 0.06\n  controller "ardupilot_vehicle_controller"\n  controllerArgs [ "--instance" "{instance}" ]\n}}\n'
        self.world.write_text(self.original)

    def test_configuration_preserves_map_starts_and_vendor_without_extra_worlds(self):
        configure(self.run, self.ardupilot)
        expected = self.original.replace('../Iris.proto', '../protos/SensorIris.proto')
        expected = expected.replace('Iris {', 'SensorIris {').replace('"ardupilot_vehicle_controller"', '"coverage_vehicle"')
        self.assertEqual(self.world.read_text(), expected)
        configure(self.run, self.ardupilot)
        configure(self.run, self.ardupilot, check=True)
        self.assertEqual(self.world.read_text(), expected)
        self.assertEqual(list(self.world.parent.glob("*.wbt")), [self.world])
        self.assertEqual((self.example / "protos/Iris.proto").read_text(), "original Iris")
        self.assertEqual(sorted(p.name for p in (self.example / "controllers").iterdir()), [])

    def test_check_is_read_only_and_missing_world_is_not_generated(self):
        with self.assertRaisesRegex(ValueError, "configuration is missing"):
            configure(self.run, self.ardupilot, check=True)
        self.assertFalse((self.run / "webots/protos").exists())
        self.assertEqual(self.world.read_text(), self.original)
        self.world.unlink()
        with self.assertRaises(FileNotFoundError):
            configure(self.run, self.ardupilot)
        self.assertFalse(self.world.exists())

    def test_foreign_controller_and_custom_world_are_preserved(self):
        controllers = self.run / "webots/controllers"
        controllers.mkdir()
        (controllers / "custom.py").write_text("user data")
        with self.assertRaisesRegex(ValueError, "not managed"):
            configure(self.run, self.ardupilot)
        self.assertEqual(self.world.read_text(), self.original)
        self.assertEqual((controllers / "custom.py").read_text(), "user data")
        alternate = self.world.with_name("custom.wbt")
        with self.assertRaisesRegex(ValueError, "Only the existing village"):
            configured_text(alternate)

    def test_previous_vendor_symlink_is_migrated_without_writing_vendor(self):
        controllers = self.run / "webots/controllers"
        controllers.symlink_to(self.example / "controllers", target_is_directory=True)
        prepare_project(self.run, self.ardupilot)
        self.assertFalse(controllers.is_symlink())
        self.assertTrue((controllers / "coverage_vehicle").is_symlink())
        self.assertEqual(list((self.example / "controllers").iterdir()), [])

    def test_unknown_controller_is_not_overwritten(self):
        original = self.original.replace('"ardupilot_vehicle_controller"', '"custom"', 1)
        self.world.write_text(original)
        with self.assertRaisesRegex(ValueError, "known controllers"):
            configure(self.run, self.ardupilot)
        self.assertEqual(self.world.read_text(), original)


class MotorExchangeTests(unittest.TestCase):
    def test_delayed_and_foreign_motor_packets_do_not_accumulate(self):
        address = ("127.0.0.1", 9003)
        packets = [(struct.pack("16f", *([value] * 16)), address) for value in (0.1, 0.2, 0.3)]
        packets += [(b"broken", address), (struct.pack("16f", *([0.9] * 16)), ("127.0.0.1", 9004))]
        channel = MagicMock()
        channel.recvfrom.side_effect = lambda count: packets.pop(0)
        with patch("coverage_vehicle.select.select", side_effect=lambda *args: ([channel] if packets else [], [], [])):
            latest = latest_controls(channel, address, 64, "16f")
        self.assertAlmostEqual(latest[0], 0.3)
        self.assertEqual(packets, [])

    def test_disabled_motors_stay_off_and_nan_is_rejected(self):
        address = ("127.0.0.1", 9003)
        packets = [(struct.pack("16f", *([-1] * 16)), address),
                   (struct.pack("16f", *([math.nan] * 16)), address)]
        channel = MagicMock()
        channel.recvfrom.side_effect = lambda count: packets.pop(0)
        with patch("coverage_vehicle.select.select", side_effect=lambda *args: ([channel] if packets else [], [], [])):
            self.assertEqual(latest_controls(channel, address, 64, "16f"), (0,) * 16)

    def test_continuous_sender_cannot_starve_physics_step(self):
        address = ("127.0.0.1", 9003)
        channel = MagicMock()
        channel.recvfrom.return_value = (struct.pack("16f", *([0.3] * 16)), address)
        with patch("coverage_vehicle.select.select", return_value=([channel], [], [])):
            latest = latest_controls(channel, address, 64, "16f")
        self.assertAlmostEqual(latest[0], 0.3)
        self.assertEqual(channel.recvfrom.call_count, 256)


if __name__ == "__main__":
    unittest.main()
