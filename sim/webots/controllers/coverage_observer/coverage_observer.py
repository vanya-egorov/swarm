"""Record physical bounds, separation and airborne contacts during acceptance runs."""

import json
import os
from pathlib import Path

from controller import Supervisor


robot = Supervisor()
period = int(robot.getBasicTimeStep())
mission_path = Path(robot.getCustomData())
mission = json.loads(mission_path.read_text())
output = Path(os.environ["COVERAGE_EVIDENCE_DIR"])
children = robot.getRoot().getField("children")
vehicles = {}
for i in range(children.getCount()):
    node = children.getMFNode(i)
    name = node.getField("name")
    if name and name.getSFString() in [f"Iris {j}" for j in range(3)]:
        vehicles[int(name.getSFString().split()[1])+1] = node
if len(vehicles) != 3:
    raise RuntimeError("Observer requires all three Iris")

starts = {system_id: node.getPosition()[2] for system_id, node in vehicles.items()}
area = mission["Area"]
low, high = area["Min"], area["Max"]
summary = {"samples": 0, "airborneContacts": {str(i): 0 for i in vehicles},
           "outsideRegion": {str(i): 0 for i in vehicles}, "minSeparationM": 1e9,
           "maxHeightM": {str(i): 0 for i in vehicles}}
last_write = -1
with (output / "physics.jsonl").open("w", buffering=1) as stream:
    while robot.step(period) != -1:
        positions = {system_id: node.getPosition() for system_id, node in vehicles.items()}
        summary["samples"] += 1
        for system_id, node in vehicles.items():
            x, y, z = positions[system_id]
            height = z-starts[system_id]
            key = str(system_id)
            summary["maxHeightM"][key] = max(summary["maxHeightM"][key], height)
            summary["airborneContacts"][key] += int(height > 0.35 and bool(node.getContactPoints(True)))
            north_min = low["NorthM"]+(system_id-1)*(high["NorthM"]-low["NorthM"])/3
            north_max = north_min+(high["NorthM"]-low["NorthM"])/3
            summary["outsideRegion"][key] += int(not (north_min <= x <= north_max and
                                                     low["EastM"] <= -y <= high["EastM"]))
            for other, p in positions.items():
                if other > system_id:
                    distance = ((x-p[0])**2+(y-p[1])**2)**0.5
                    summary["minSeparationM"] = min(summary["minSeparationM"], distance)
        if robot.getTime()-last_write >= 0.1:
            stream.write(json.dumps({"timeS": robot.getTime(), "positions": positions})+"\n")
            (output / "physics-summary.json").write_text(json.dumps(summary, indent=2)+"\n")
            last_write = robot.getTime()
