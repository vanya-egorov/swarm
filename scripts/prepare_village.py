"""Export the saved Unity village into Webots without starting Unity."""

import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path

from village.geometry import convert_model
from village.coverage import prepare
from village.scene import include_instance, read_scene
from village.spawn import choose_start
from village.terrain import read_terrain
from village.world import tile_bounds, write_world
from webots_project import prepare_project

PROJECT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=PROJECT / "проект_карта_деревни")
    parser.add_argument("--map-size", type=float, default=1500)
    parser.add_argument("--spawn", type=float, nargs=2, metavar=("X", "Z"))
    args = parser.parse_args()
    source = args.source.resolve()
    run = Path(os.environ.get("RUN_DIR", PROJECT / "projectData/run")).resolve()
    ardupilot = Path(os.environ.get("ARDUPILOT_DIR", PROJECT / "projectData/ardupilot")).resolve()
    output, iris = prepare_project(run, ardupilot)
    proto_dir = output / "protos/village"
    proto_dir.mkdir(parents=True, exist_ok=True)
    print("Чтение сохранённой сцены и рельефа...", flush=True)
    all_instances, terrain_path, assets = read_scene(source)
    instances = [item for item in all_instances if include_instance(item)]
    if not instances:
        raise ValueError("The source scene has no selected instances")
    terrain = read_terrain(terrain_path, assets)
    if not args.map_size <= min(terrain.size):
        raise ValueError("map-size exceeds the source terrain")
    models = {}
    for path in sorted({item.asset for item in instances}):
        print(f"Конвертация {path.name}", flush=True)
        models[path] = convert_model(path, source, proto_dir)
    print("Выбор места старта...", flush=True)
    origin, pad_tops, variation = choose_start(terrain, instances, models, args.map_size, args.spawn)
    print(f"Старт в исходной карте: {origin}; перепад у стартов: {variation:.3f} м", flush=True)
    full = tile_bounds(terrain, origin, args.map_size)
    worlds = [write_world(output, iris, terrain, instances, models, assets, origin,
                          pad_tops, full, 2048)]
    parameters = output / "village-flight.parm"
    base_params = PROJECT / "sim/ardupilot/iris-arm-check.parm"
    village_params = PROJECT / "sim/ardupilot/village-flight.parm"
    parameters.write_text(base_params.read_text()+"\n"+village_params.read_text())
    sources = sorted(path for path in (source / "Assets").rglob("*") if path.is_file())
    manifest = {
        "source": str(source), "unityRequired": False, "sourceTerrainSizeM": terrain.size,
        "mapSizeM": [args.map_size, args.map_size],
        "terrainHeightRangeM": [float(terrain.heights.min()), float(terrain.heights.max())],
        "sourceInstances": len(all_instances), "filteredInstances": len(instances), "exportedInstances": worlds[0]["instances"],
        "croppedOutInstances": len(instances)-worlds[0]["instances"],
        "excludedInstances": len(all_instances)-len(instances), "objectFilter": "exclude houses and PP_Forest_Mountain_Moss_01/02",
        "filteredSourceCounts": dict(Counter(i.asset.stem for i in instances)),
        "originUnityXZ": origin.tolist(), "originAltitudeM": float(pad_tops[0]),
        "padTopsM": pad_tops.tolist(), "maxFlightAreaReliefM": variation,
        "collisions": "conservative boxes for trees, rocks and bridges; original height grid",
        "sourceSHA256": {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        "worlds": worlds, "flightParameters": str(parameters),
        "flightParametersSHA256": hashlib.sha256(parameters.read_bytes()).hexdigest(),
    }
    (output / "village-manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n")
    launches = prepare(output / "worlds/village.wbt", output / "coverage-mission.json")
    print(f"Сухие старты покрытия: {launches['startsWebots']}")
    for result in worlds:
        print(f"Мир: {result['world']} ({result['instances']} объектов)")
    print("Откройте village.wbt в Webots; включите моделирование, затем make village-route-check.")


if __name__ == "__main__":
    main()
