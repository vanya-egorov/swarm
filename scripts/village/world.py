"""Write the single village world using shared model geometry."""

import numpy as np

from .geometry import matrix_axis_angle, numbers
from .spawn import OFFSETS
from .static_scene import write_static_objects
from .textures import bake


def tile_bounds(terrain, center, size):
    spacing = np.array(terrain.spacing)
    count = np.ceil(size/spacing).astype(int)
    lower = np.rint(center/spacing-count/2).astype(int)
    lower = np.clip(lower, 0, np.array([terrain.heights.shape[1]-1, terrain.heights.shape[0]-1])-count)
    upper = lower+count
    return (*lower, *upper)


def camera_rotation(position, target):
    forward = np.array(target, dtype=float)-position
    forward /= np.linalg.norm(forward)
    left = np.cross([0, 0, 1], forward)
    left /= np.linalg.norm(left)
    up = np.cross(forward, left)
    return matrix_axis_angle(np.column_stack([forward, left, up]))


def write_world(output, iris_proto, terrain, instances, models, assets, origin, pad_tops,
                bounds, texture_resolution):
    from webots_project import iris
    x0, y0, x1, y1 = bounds
    spacing = np.array(terrain.spacing)
    low, high = np.array([x0, y0])*spacing, np.array([x1, y1])*spacing
    altitude = float(pad_tops[0])
    selected = [item for item in instances if np.all(item.position[[0, 2]] >= low)
                and np.all(item.position[[0, 2]] <= high)]
    used = sorted({item.asset for item in selected})
    texture = output / "worlds/village-assets" / "village-terrain.png"
    bake(terrain, assets, (*low, *high), texture, texture_resolution)
    world = output / "worlds" / "village.wbt"
    with world.open("w") as stream:
        stream.write(f'''#VRML_SIM R2025b utf8
# Generated from the saved Unity village; original files are never modified.
EXTERNPROTO "{iris_proto.as_posix()}"
''')
        stream.write(f'''
WorldInfo {{ title "Village — three Iris" basicTimeStep 2 FPS 20 }}
Viewpoint {{
  orientation {numbers(camera_rotation(np.array([20, -40, 35]), [6, 6, 1.5]))}
  position 20 -40 35
  far 600
}}
Background {{ skyColor [ 0.65 0.8 0.95 ] }}
DirectionalLight {{ direction -0.5 -0.5 -1 ambientIntensity 0.5 castShadows FALSE }}
Solid {{
  translation {numbers([*(low-origin), -altitude])}
  name "Village terrain"
  children [ Shape {{
    appearance PBRAppearance {{
      baseColorMap ImageTexture {{ url [ "village-assets/{texture.name}" ] }}
      roughness 1 metalness 0
    }}
    geometry DEF TERRAIN ElevationGrid {{
      xDimension {x1-x0+1} yDimension {y1-y0+1}
      xSpacing {spacing[0]} ySpacing {spacing[1]}
      height [
''')
        for row in terrain.heights[y0:y1+1, x0:x1+1]:
            stream.write("        "+numbers(row)+"\n")
        stream.write("      ]\n    }\n  } ]\n  boundingObject USE TERRAIN\n}\n")
        write_static_objects(stream, selected, models, origin, altitude)
        for index, (offset, top) in enumerate(zip(OFFSETS, pad_tops)):
            stream.write(f'''Solid {{
  translation {numbers([*offset, top-altitude-0.075])}
  name "Landing pad {index}"
  children [ Shape {{
    appearance PBRAppearance {{ baseColor 0.9 0.7 0.1 roughness 1 metalness 0 }}
    geometry DEF PAD_{index} Box {{ size 3 3 0.15 }}
  }} ]
  boundingObject USE PAD_{index}
}}
''')
            stream.write(iris(index, [*offset, top-altitude+0.06]))
    return {"world": str(world), "instances": len(selected), "models": len(used), "combinedStaticScene": True,
            "colliders": sum(models[item.asset].collider for item in selected),
            "boundsUnityXZ": [*low.tolist(), *high.tolist()], "heightSamples": [int(x1-x0+1), int(y1-y0+1)]}
