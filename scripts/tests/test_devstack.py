"""Exercise process supervision with fake SITL/monitor executables."""

import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest

PROJECT = Path(__file__).resolve().parents[2]
MOCK = '''#!/usr/bin/env python3
import os, signal, sys, time
from pathlib import Path
args = sys.argv[1:]
name = Path(sys.argv[0]).name
with open(os.environ["CALLS"], "a") as log:
    log.write(name + " " + " ".join(args) + "\\n")
if os.environ.get("IGNORE_TERM") == name:
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
else:
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
if os.environ.get("FAIL_INSTANCE") in args:
    time.sleep(0.2)
    sys.exit(1)
if "-sitl-group-flight-check" in args or "-sitl-group-route-check" in args:
    time.sleep(0.5)
    sys.exit(0)
if "-sitl-arm-check" in args:
    vehicle = int(args[args.index("-system-id") + 1])
    time.sleep(vehicle * 0.4)
    with open(os.environ["CALLS"], "a") as log:
        log.write(f"completed {vehicle}\\n")
    sys.exit(0)
while True:
    time.sleep(0.05)
'''


class DevstackTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "bin").mkdir()
        for path in (self.root / "sitl", self.root / "bin/swarm"):
            path.write_text(MOCK)
            path.chmod(0o755)
        params = self.root / "iris.parm"
        params.touch()
        self.env = dict(os.environ, RUN_DIR=str(self.root), SITL_BIN=str(self.root / "sitl"),
                        SITL_PARAMS=str(params), CALLS=str(self.root / "calls"), DRONE_COUNT="3")

    def start(self, command="start", children=6):
        process = subprocess.Popen(["bash", str(PROJECT / "scripts/devstack.sh"), command],
                                   env=self.env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.addCleanup(self.finish, process)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            calls = self.root / "calls"
            if calls.exists() and len(calls.read_text().splitlines()) == children:
                return process
            if process.poll() is not None:
                self.fail(process.communicate()[0].decode())
            time.sleep(0.02)
        self.fail("six child processes did not start")

    def finish(self, process):
        if process.poll() is None:
            process.terminate()
        process.communicate(timeout=15)

    def assert_stopped(self, process, code):
        output = process.communicate(timeout=15)[0].decode()
        self.assertEqual(process.returncode, code, output)
        for row in (self.root / "children-copy").read_text().splitlines():
            fields = dict(field.split("=") for field in row.split())
            for key in ("sitl", "monitor"):
                with self.assertRaises(ProcessLookupError):
                    os.kill(int(fields[key]), 0)
        self.assertFalse((self.root / "stack.pid").exists())

    def snapshot_children(self):
        rows = (self.root / "children").read_text()
        (self.root / "children-copy").write_text(rows)
        return [dict(field.split("=") for field in row.split()) for row in rows.splitlines()]

    def test_three_instances_stop_together(self):
        process = self.start()
        rows = self.snapshot_children()
        self.assertEqual([r["system_id"] for r in rows], ["1", "2", "3"])
        self.assertEqual([r["port"] for r in rows], ["14560", "14570", "14580"])
        calls = (self.root / "calls").read_text()
        for i in range(3):
            self.assertIn(f"--sysid {i+1} -I{i}", calls)
            self.assertIn(f"-listen 127.0.0.1:{14560+10*i} -system-id {i+1}", calls)
        subprocess.run(["bash", str(PROJECT / "scripts/devstack.sh"), "stop"],
                       env=self.env, check=True, capture_output=True, timeout=5)
        self.assert_stopped(process, 0)

    def test_arm_check_waits_for_every_vehicle(self):
        self.env["ARM_CHECK_PARAMS"] = self.env["SITL_PARAMS"]
        process = self.start("arm-check")
        self.snapshot_children()
        self.assert_stopped(process, 0)
        calls = (self.root / "calls").read_text()
        for vehicle in (1, 2, 3):
            self.assertIn(f"completed {vehicle}", calls)

    def test_selective_arm_check_keeps_other_monitors_passive(self):
        self.env.update(ARM_CHECK_PARAMS=self.env["SITL_PARAMS"], CHECK_INSTANCE="1")
        process = self.start("arm-check")
        self.snapshot_children()
        self.assert_stopped(process, 0)
        calls = (self.root / "calls").read_text()
        self.assertIn("completed 2", calls)
        self.assertNotIn("completed 1", calls)
        self.assertNotIn("completed 3", calls)

    def test_stop_is_bounded_when_sitl_ignores_term(self):
        self.env["IGNORE_TERM"] = "sitl"
        process = self.start()
        self.snapshot_children()
        subprocess.run(["bash", str(PROJECT / "scripts/devstack.sh"), "stop"],
                       env=self.env, check=True, capture_output=True, timeout=5)
        self.assert_stopped(process, 0)

    def test_group_flight_uses_one_shared_controller(self):
        self.env["ARM_CHECK_PARAMS"] = self.env["SITL_PARAMS"]
        process = self.start("group-flight-check", children=4)
        rows = self.snapshot_children()
        self.assertEqual(len({r["monitor"] for r in rows}), 1)
        self.assert_stopped(process, 0)

    def test_group_route_uses_one_shared_controller(self):
        self.env["ARM_CHECK_PARAMS"] = self.env["SITL_PARAMS"]
        process = self.start("group-route-check", children=4)
        rows = self.snapshot_children()
        self.assertEqual(len({r["monitor"] for r in rows}), 1)
        self.assert_stopped(process, 0)
        calls = (self.root / "calls").read_text()
        self.assertEqual(calls.count("-sitl-group-route-check"), 1)

    def test_group_route_rejects_partial_group(self):
        for overrides in ({"DRONE_COUNT": "1"}, {"CHECK_INSTANCE": "1"}):
            with self.subTest(overrides=overrides):
                result = subprocess.run(["bash", str(PROJECT / "scripts/devstack.sh"), "group-route-check"],
                                        env=dict(self.env, **overrides), capture_output=True, timeout=5)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse((self.root / "calls").exists())

    def test_child_failure_stops_entire_stack(self):
        self.env["FAIL_INSTANCE"] = "-I1"
        process = self.start()
        self.snapshot_children()
        self.assert_stopped(process, 1)


if __name__ == "__main__":
    unittest.main()
