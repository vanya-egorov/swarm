"""Move the three Iris onto dry ground and prepare coverage in the main world."""

import json
import os
from pathlib import Path

from village.coverage import prepare


if __name__ == "__main__":
    project = Path(__file__).resolve().parents[1]
    output = Path(os.environ.get("RUN_DIR", project / "projectData/run")) / "webots"
    result = prepare(output / "worlds/village.wbt", output / "coverage-mission.json")
    print(json.dumps(result, ensure_ascii=False, indent=2))
