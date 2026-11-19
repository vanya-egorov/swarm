"""Select a flat, obstacle-free starting area near the center of the forest map."""

import numpy as np

OFFSETS = np.array([[0, 0], [12, 0], [0, 12]], dtype=float)


def choose_start(terrain, instances, models, map_size, requested=None):
    obstacles = [item for item in instances if models[item.asset].collider]
    centers = np.array([item.position[[0, 2]] for item in obstacles])
    radii = np.array([np.linalg.norm(np.maximum(abs(models[item.asset].minimum),
                      abs(models[item.asset].maximum))) * max(abs(item.scale)) for item in obstacles])
    if requested is not None:
        candidates = np.array([requested], dtype=float)
    else:
        margin = map_size/2
        candidates = np.array([(x, y) for x in np.arange(margin, terrain.size[0]-margin, 10)
                               for y in np.arange(margin, terrain.size[1]-margin, 10)])
        middle = np.array(terrain.size)/2
        candidates = candidates[np.argsort(np.linalg.norm(candidates-middle, axis=1))]
    best = None
    samples = np.array([(x, y) for x in np.linspace(-4.5, 4.5, 5) for y in np.linspace(-4.5, 4.5, 5)])
    for candidate in candidates:
        if np.any(candidate-map_size/2 < 0) or np.any(candidate+map_size/2 > terrain.size):
            continue
        starts = candidate+OFFSETS
        points = starts[:, None, :]+samples
        heights = terrain.sample(points[:, :, 0], points[:, :, 1])
        variation = float(np.max(np.ptp(heights, axis=1)))
        if variation > 0.20:
            continue
        local = centers-candidate
        nearby = np.linalg.norm(local-[6, 6], axis=1) < 40+radii
        if any(np.any(np.linalg.norm(centers[nearby]-point, axis=1) < radii[nearby]+5) for point in starts):
            continue
        # Prefer a central flat clearing, without consulting discarded houses or decor.
        distance = float(np.linalg.norm(candidate-np.array(terrain.size)/2))
        score = variation + distance*0.0002
        if best is None or score < best[0]:
            best = (score, candidate, heights.max(axis=1)+0.03, variation)
    if best is None:
        raise ValueError("No flat obstacle-free start area found; adjust --spawn or --map-size")
    _, origin, pad_tops, variation = best
    return origin, pad_tops, variation
