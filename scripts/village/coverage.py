"""Prepare dry, aligned launches and a small mission in the existing village world."""

import json
from pathlib import Path
import re
import uuid

import numpy as np
from PIL import Image

from .terrain import Terrain

OFFSETS = np.array([[0, 0], [12, 0], [24, 0]], dtype=float)
NUMBER = r"[-+\d.eE]+"
ROOT = re.compile(r"(?m)^(?:Solid|SensorIris) \{")


def root_nodes(text):
    for match in ROOT.finditer(text):
        depth, end = 1, match.end()
        while depth:
            if text[end] == "{":
                depth += 1
            elif text[end] == "}":
                depth -= 1
            end += 1
        yield match.start(), end, text[match.start():end]


def vector(node, field, count=3):
    match = re.search(rf"\b{field}\s+((?:{NUMBER}\s+){{{count-1}}}{NUMBER})", node)
    if match is None:
        raise ValueError(f"Missing {field} in village world")
    return np.fromstring(match[1], sep=" ")


def read_world(text, texture):
    nodes = list(root_nodes(text))
    ground = next(node for _, _, node in nodes if 'name "Village terrain"' in node)
    translation = vector(ground, "translation")
    width = int(vector(ground, "xDimension", 1)[0])
    height = int(vector(ground, "yDimension", 1)[0])
    spacing = (vector(ground, "xSpacing", 1)[0], vector(ground, "ySpacing", 1)[0])
    values = re.search(r"\bheight\s*\[([^]]+)\]", ground)[1].replace(",", " ")
    heights = np.fromstring(values, sep=" ").reshape(height, width)
    terrain = Terrain(heights + translation[2], spacing, [], np.zeros((0, 0, 0)))
    boxes = []
    for _, _, node in nodes:
        if 'name "Village collision tile ' not in node:
            continue
        tile = vector(node, "translation")
        for pose in re.findall(r"Pose\s*\{(.*?)children\s*\[\s*Box\s*\{(.*?)\}", node, re.S):
            center = tile + vector(pose[0], "translation")
            # A horizontal bounding circle also encloses boxes rotated around any axis.
            radius = np.linalg.norm(vector(pose[1], "size")) / 2
            boxes.append([*center[:2], radius])
    if not boxes:
        raise ValueError("Village collision boxes are missing")
    return terrain, translation[:2], np.asarray(Image.open(texture).convert("RGB")), np.asarray(boxes)


def dry_rectangle(image, terrain, low, high):
    size = np.array(terrain.size)
    pixels = np.array([image.shape[1], image.shape[0]])
    a = np.floor(low / size * pixels).astype(int)
    b = np.ceil(high / size * pixels).astype(int)
    if np.any(a < 0) or np.any(b >= pixels):
        return False
    # The exported source paints its river blue; include blended shoreline pixels.
    region = image[image.shape[0]-b[1]-1:image.shape[0]-a[1], a[0]:b[0]+1].astype(float)
    water = (region[:, :, 2] > region[:, :, 0]*1.2) & (region[:, :, 2] > region[:, :, 1]*0.8)
    return region.size > 0 and not water.any()


def choose_launches(terrain, translation, image, boxes):
    candidates = np.array([(x, y) for x in range(-650, 651, 5) for y in range(-650, 651, 5)])
    candidates = candidates[np.argsort(np.linalg.norm(candidates+[12, 0], axis=1), kind="stable")]
    samples = np.array([(x, y) for x in np.linspace(-4.5, 4.5, 13) for y in np.linspace(-4.5, 4.5, 13)])
    for candidate in candidates:
        low, high = candidate+[-6, -6], candidate+[30, 6]
        if not dry_rectangle(image, terrain, low-translation-2, high-translation+2):
            continue
        starts = candidate+OFFSETS
        if any(np.any(np.linalg.norm(boxes[:, :2]-point, axis=1) < boxes[:, 2]+5) for point in starts):
            continue
        points = starts[:, None, :]+samples-translation
        heights = terrain.sample(points[:, :, 0], points[:, :, 1])
        if np.max(np.ptp(heights, axis=1)) > 0.20:
            continue
        return starts, heights.max(axis=1)+0.03
    raise ValueError("No dry, flat and obstacle-free coverage area in the existing village world")


def prepare(world, mission_path):
    text = world.read_text()
    if 'name "Coverage observer"' in text:
        raise ValueError("Stop the active coverage check before changing launch positions")
    terrain, translation, image, boxes = read_world(text, world.parent / "village-assets/village-terrain.png")
    starts, tops = choose_launches(terrain, translation, image, boxes)
    replacements = []
    found = set()
    for begin, end, node in root_nodes(text):
        match = re.search(r'name "(Landing pad|Iris) ([0-2])"', node)
        if match is None:
            continue
        kind, index = match[1], int(match[2])
        found.add((kind, index))
        z = tops[index] + (0.06 if kind == "Iris" else -0.075)
        position = " ".join(f"{v:.8g}" for v in [*starts[index], z])
        node = re.sub(rf"\btranslation\s+(?:{NUMBER}\s+){{2}}{NUMBER}", f"translation {position}", node, count=1)
        if kind == "Iris":
            node = re.sub(r"(?m)^\s*hidden [^\n]+\n", "\n", node)
            node = re.sub(rf"\brotation\s+(?:{NUMBER}\s+){{3}}{NUMBER}", "rotation 0 0 1 0", node, count=1)
        replacements.append((begin, end, node))
    if len(found) != 6:
        raise ValueError("Expected all three Iris and landing pads")
    for begin, end, node in reversed(replacements):
        text = text[:begin]+node+text[end:]
    center = np.array([*starts.mean(axis=0), float(tops.max())+1.5])
    camera = center+[20, -40, 35]
    # Fixed view towards the formation from a [20, -40, 35] metre offset.
    rotation = "-0.3195410632 0.1974872378 0.9267644252 2.101276249"
    position = " ".join(f"{v:.8g}" for v in camera)
    text = re.sub(r"Viewpoint\s*\{[^}]+\}",
                  f"Viewpoint {{ orientation {rotation} position {position} far 600 }}", text, count=1)
    launch = starts[0]
    mission = {
        "RunID": f"village-{uuid.uuid4().hex}",
        "Origin": {"LatitudeDeg": -35.363261, "LongitudeDeg": 149.165237},
        "Area": {"Min": {"NorthM": float(launch[0]-6), "EastM": float(-launch[1]-6)},
                 "Max": {"NorthM": float(launch[0]+30), "EastM": float(-launch[1]+6)}},
        "Coverage": {"SwathWidthM": 8, "LaneSpacingM": 4, "AltitudeM": 1.5, "Direction": 1},
        "Navigation": {"Enabled": True, "SensorMaxAge": 500000000, "MaxSpeedMS": 0.4},
        "Launches": [{"SystemID": i+1, "Position": {"NorthM": float(p[0]), "EastM": float(-p[1])}}
                     for i, p in enumerate(starts)],
        "Network": {"Peers": [{"SystemID": i, "Address": f"127.0.0.1:{14600+i}"} for i in range(1, 4)]},
    }
    result = {"startsWebots": starts.tolist(), "padTopsM": tops.tolist(), "mission": str(mission_path)}
    manifest_path = world.parent.parent / "village-manifest.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else None
    world.write_text(text)
    mission_path.write_text(json.dumps(mission, indent=2)+"\n")
    if manifest is not None:
        manifest["coverage"] = result
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n")
    return result
