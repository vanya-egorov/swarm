"""Protect upstream controller files and foreign directories during preparation."""

from pathlib import Path
import tempfile
import unittest

from scripts.webots_project import prepare_project


class WebotsProjectTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.ardupilot = self.root / "ardupilot"
        self.example = self.ardupilot / "libraries/SITL/examples/Webots_Python"
        (self.example / "protos").mkdir(parents=True)
        (self.example / "protos/Iris.proto").write_text("original model")
        (self.example / "controllers/ardupilot_vehicle_controller").mkdir(parents=True)
        (self.example / "controllers/ardupilot_vehicle_controller/controller.py").write_text("original controller")
        self.run = self.root / "run"

    def test_repeated_preparation_keeps_upstream_untouched(self):
        original = sorted(str(path.relative_to(self.ardupilot)) for path in self.ardupilot.rglob("*"))
        output, proto = prepare_project(self.run, self.ardupilot)
        prepare_project(self.run, self.ardupilot)
        self.assertEqual(proto.read_text(), "original model")
        link = output / "controllers/ardupilot_vehicle_controller"
        self.assertTrue(link.is_symlink())
        self.assertEqual((link / "controller.py").read_text(), "original controller")
        self.assertEqual(sorted(str(path.relative_to(self.ardupilot)) for path in self.ardupilot.rglob("*")), original)

    def test_previous_whole_directory_symlink_is_migrated(self):
        (self.run / "webots").mkdir(parents=True)
        link = self.run / "webots/controllers"
        link.symlink_to(self.example / "controllers", target_is_directory=True)
        prepare_project(self.run, self.ardupilot)
        self.assertFalse(link.is_symlink())
        self.assertTrue((link / "ardupilot_vehicle_controller").is_symlink())

    def test_foreign_controller_directory_is_preserved(self):
        foreign = self.run / "webots/controllers"
        foreign.mkdir(parents=True)
        (foreign / "custom.py").write_text("user data")
        with self.assertRaisesRegex(ValueError, "not managed"):
            prepare_project(self.run, self.ardupilot)
        self.assertEqual((foreign / "custom.py").read_text(), "user data")


if __name__ == "__main__":
    unittest.main()
