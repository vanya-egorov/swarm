"""Decode the original TerrainData height samples and splat textures."""

from dataclasses import dataclass

import numpy as np
import UnityPy


@dataclass
class Terrain:
    heights: np.ndarray
    spacing: tuple
    layers: list
    alpha: np.ndarray

    @property
    def size(self):
        return ((self.heights.shape[1]-1)*self.spacing[0], (self.heights.shape[0]-1)*self.spacing[1])

    def sample(self, x, y):
        x = np.clip(np.asarray(x)/self.spacing[0], 0, self.heights.shape[1]-1)
        y = np.clip(np.asarray(y)/self.spacing[1], 0, self.heights.shape[0]-1)
        ix, iy = x.astype(int), y.astype(int)
        nx = np.minimum(ix+1, self.heights.shape[1]-1)
        ny = np.minimum(iy+1, self.heights.shape[0]-1)
        fx, fy = x-ix, y-iy
        return ((1-fx)*(1-fy)*self.heights[iy, ix] + fx*(1-fy)*self.heights[iy, nx]
                + (1-fx)*fy*self.heights[ny, ix] + fx*fy*self.heights[ny, nx])


def read_terrain(path, assets):
    environment = UnityPy.load(str(path))
    objects = {obj.path_id: obj for obj in environment.objects}
    terrains = [obj for obj in objects.values() if obj.type.name == "TerrainData"]
    if len(terrains) != 1:
        raise ValueError("Expected exactly one TerrainData asset")
    obj = terrains[0]
    data = obj.parse_as_dict()
    heightmap = data["m_Heightmap"]
    resolution = heightmap["m_Resolution"]
    # Unity's serialized Terrain heights use 15 significant bits, not full-range uint16.
    heights = np.asarray(heightmap["m_Heights"], dtype=float).reshape(resolution, resolution)
    heights *= heightmap["m_Scale"]["y"] / 32767
    if not np.all(np.isfinite(heights)):
        raise ValueError("Nonfinite terrain heights")
    if heightmap.get("m_Holes"):
        raise ValueError("Terrain holes require separate collision geometry")
    splat = data["m_SplatDatabase"]
    alpha = np.concatenate([np.asarray(objects[p["m_PathID"]].parse_as_object().image.convert("RGBA"))
                            for p in splat["m_AlphaTextures"]], axis=2).astype(float) / 255
    layers = []
    for pointer in splat["m_TerrainLayers"]:
        external = obj.assets_file.externals[pointer["m_FileID"]-1]
        # Unity GUID text stores the low nibble first within each serialized byte.
        guid = "".join(f"{byte & 15:x}{byte >> 4:x}" for byte in external.guid)
        if guid not in assets:
            raise ValueError(f"Unresolved terrain layer {guid}")
        layers.append(assets[guid])
    # Texture images have a top-left origin, whereas height rows start at Unity z=0.
    alpha = np.flipud(alpha[:, :, :len(layers)])
    return Terrain(heights, (heightmap["m_Scale"]["x"], heightmap["m_Scale"]["z"]), layers, alpha)
