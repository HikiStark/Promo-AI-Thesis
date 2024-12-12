from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid, DynamicCuboid
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.sensor import Camera

# Create a World
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Create a "Table" using a Static Cuboid
# Let's say our table surface is 1 meter by 1 meter, and 0.05m thick.
class add_table():
    table = VisualCuboid(
        prim_path="/World/Table",
        name="table",
        position=(0.0, 0.0, 0.5),  # This elevates the table top at z=0.5m
        size=(1.0, 1.0, 0.05),  # (X, Y, Z) size
        color=(0.6, 0.3, 0.0),  # Brownish color
    )

    world.scene.add(table)

# Add some cubes on top of the table
# Each cube is small, say 0.05m on a side.
class add_cubes():
    cube_positions = [(0.0, 0.0, 0.525), (0.1, 0.1, 0.525), (-0.1, 0.1, 0.525)]
    for i, pos in enumerate(cube_positions):
        cube = DynamicCuboid(
            prim_path=f"/World/Cube_{i}",
            name=f"cube_{i}",
            position=pos,
            size=(0.05, 0.05, 0.05),
            color=(1.0, 0.0, 0.0),  # Red cubes
        )
        world.scene.add(cube)

# Add a Camera pointing downward at the table
# Let's place it at (0,0,1.5) so it's above the table.
# We need to rotate the camera to look straight down.
# If the camera initially looks along the +X axis, a rotation of -90 degrees about the Y-axis
# will align it to look down the -Z axis.
class add_camera():
    camera_orientation = euler_angles_to_quat([0, -90, 0])  # [roll, pitch, yaw] in degrees
    camera = Camera(
        prim_path="/World/OverheadCamera",
        translation=(0.0, 0.0, 1.5),
        orientation=camera_orientation,
        resolution=(1920, 1080),
    )
    world.scene.add(camera)

add_table()
add_cubes()
add_camera()

camera = add_camera.camera

world.reset()
for _ in range(10):
    world.step(render=True)
    camera.get_rgb_image()
