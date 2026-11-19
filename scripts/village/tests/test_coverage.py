"""Check dry launches and the geometry read from the existing Webots world."""

from pathlib import Path
import tempfile
import unittest

import numpy as np
from PIL import Image

from scripts.village.coverage import dry_rectangle, prepare, read_world, root_nodes
from scripts.village.terrain import Terrain


class CoveragePreparationTest(unittest.TestCase):
    def test_prepare_preserves_map_and_produces_repeatable_launches(self):
        ground = '''Solid {
  translation -500 -500 0
  name "Village terrain"
  children [ Shape { geometry DEF TERRAIN ElevationGrid {
    xDimension 2 yDimension 2 xSpacing 1000 ySpacing 1000 height [ 0 0 0 0 ]
  } } ]
}
Solid {
  translation 400 400 0
  name "Village collision tile 4 4"
  boundingObject Group { children [ Pose {
    translation 1 2 3 rotation 0 0 1 0
    children [ Box { size 2 4 6 } ]
  } ] }
}
'''
        drones = "\n".join(f'''Solid {{ translation 0 0 0 name "Landing pad {i}" }}
SensorIris {{
  hidden linearVelocity_0 1 2 3
  translation 0 0 0.06 rotation 0 0 1 1.5 name "Iris {i}"
}}
''' for i in range(3))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            assets = root / "worlds/village-assets"
            assets.mkdir(parents=True)
            Image.new("RGB", (100, 100), (80, 130, 40)).save(assets / "village-terrain.png")
            world, mission = root / "worlds/village.wbt", root / "coverage-mission.json"
            world.write_text(ground+drones)
            first = prepare(world, mission)
            original = world.read_text()
            self.assertIn(ground, original)
            self.assertNotIn("hidden linearVelocity", original)
            second = prepare(world, mission)
            self.assertEqual(first["startsWebots"], second["startsWebots"])
            self.assertEqual(original, world.read_text())

    def test_water_and_blended_shore_are_rejected(self):
        terrain = Terrain(np.zeros((2, 2)), (100, 100), [], np.zeros((0, 0, 0)))
        image = np.full((100, 100, 3), [80, 130, 40], dtype=np.uint8)
        self.assertTrue(dry_rectangle(image, terrain, np.array([40, 40]), np.array([60, 60])))
        image[50, 50] = [60, 100, 105]
        self.assertFalse(dry_rectangle(image, terrain, np.array([40, 40]), np.array([60, 60])))
        self.assertFalse(dry_rectangle(image, terrain, np.array([-1, 40]), np.array([60, 60])))

    def test_texture_y_axis_is_opposite_world_y(self):
        terrain = Terrain(np.zeros((2, 2)), (100, 100), [], np.zeros((0, 0, 0)))
        image = np.full((100, 100, 3), [80, 130, 40], dtype=np.uint8)
        image[:20] = [40, 90, 140]
        self.assertFalse(dry_rectangle(image, terrain, np.array([40, 85]), np.array([60, 95])))
        self.assertTrue(dry_rectangle(image, terrain, np.array([40, 5]), np.array([60, 15])))

    def test_world_terrain_and_collision_translation(self):
        world = '''Solid {
  translation -10 -20 -30
  name "Village terrain"
  children [ Shape { geometry DEF TERRAIN ElevationGrid {
    xDimension 2 yDimension 2 xSpacing 10 ySpacing 20 height [ 30, 32, 34, 36 ]
  } } ]
}
Solid {
  translation 100 200 0
  name "Village collision tile 1 2"
  boundingObject Group { children [ Pose {
    translation 1 2 3 rotation 0 0 1 0
    children [ Box { size 2 4 6 } ]
  } ] }
}
'''
        with tempfile.TemporaryDirectory() as directory:
            texture = Path(directory) / "terrain.png"
            Image.new("RGB", (10, 10), "green").save(texture)
            terrain, translation, _, boxes = read_world(world, texture)
        np.testing.assert_array_equal(translation, [-10, -20])
        self.assertEqual(terrain.sample(5, 10), 3)
        np.testing.assert_allclose(boxes[0], [101, 202, np.sqrt(56)/2])
        self.assertEqual(len(list(root_nodes(world))), 2)


if __name__ == "__main__":
    unittest.main()
