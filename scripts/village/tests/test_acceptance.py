"""Require complete routes and landing evidence before accepting physical runs."""

import json
from pathlib import Path
import tempfile
import time
import unittest

from scripts.village.acceptance import validate_agents, validate_physics


class AcceptanceEvidenceTest(unittest.TestCase):
    def test_stopped_observer_and_physical_violations_are_rejected(self):
        physics = {"samples": 1000, "airborneContacts": {str(i): 0 for i in range(1, 4)},
                   "outsideRegion": {str(i): 0 for i in range(1, 4)},
                   "maxHeightM": {str(i): 1.5 for i in range(1, 4)}, "minSeparationM": 12}
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            path = output / "physics-summary.json"
            path.write_text(json.dumps(physics))
            self.assertEqual(validate_physics(output, time.time()), physics)
            with self.assertRaisesRegex(RuntimeError, "observer stopped"):
                validate_physics(output, path.stat().st_mtime+3)
            physics["airborneContacts"]["2"] = 1
            path.write_text(json.dumps(physics))
            with self.assertRaisesRegex(RuntimeError, "criteria violated"):
                validate_physics(output, time.time())

    def write_agents(self, output, failed=None, missing_landing=None, partial=None):
        if failed:
            (output / "fault.json").write_text(json.dumps({"leader": failed, "term": 1}))
        for i in range(1, 4):
            records = [{"msg": "coverage agent started", "waypoints": 6}]
            if failed and i != failed:
                records.append({"msg": "coordination leader changed", "leaderId": 2, "term": 2})
            count = 1 if i in (failed, partial) else 6
            records.extend({"msg": "waypoint reached", "index": j} for j in range(1, count+1))
            if i != missing_landing:
                records.append({"msg": "landed"})
            if failed and i != failed:
                records.append({"msg": "stopped", "error": "coverage incomplete: unavailable vehicles [1]"})
            (output / f"agent-{i}.log").write_text("\n".join(json.dumps(r) for r in records))

    def test_all_routes_and_landings_are_required(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            self.write_agents(output)
            self.assertIsNone(validate_agents(output, None))
            self.write_agents(output, partial=2)
            with self.assertRaisesRegex(RuntimeError, "planned waypoints"):
                validate_agents(output, None)
            self.write_agents(output, missing_landing=3)
            with self.assertRaisesRegex(RuntimeError, "confirm landing"):
                validate_agents(output, None)

    def test_failover_requires_a_new_term_and_survivor_routes(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            self.write_agents(output, failed=1)
            self.assertEqual(validate_agents(output, 1), 2)
            path = output / "agent-3.log"
            path.write_text(path.read_text().replace('"term": 2', '"term": 1'))
            with self.assertRaisesRegex(RuntimeError, "replacement leader"):
                validate_agents(output, 1)


if __name__ == "__main__":
    unittest.main()
