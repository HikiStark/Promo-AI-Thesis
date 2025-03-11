# example_add_object.py

################################################################
# 1. Import and launch the SimulationApp
################################################################
# from omni.isaac.kit import SimulationApp

# Launch Isaac Sim (headless=False opens the UI; set to True for no GUI).
# simulation_app = SimulationApp({"headless": False})

################################################################
# 2. Import Isaac Sim core APIs after SimulationApp is created
################################################################
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

################################################################
# 3. Create a new World and add objects
################################################################
def main():
    # Create a world instance (the stage is automatically created for you)
    world = World(stage_units_in_meters=1.0)

    # Add a dynamic cuboid
    cube = DynamicCuboid(
        prim_path="/World/Cube",   # Where in the scene hierarchy to place it
        name="my_cube",            # A friendly name
        position=[0, 0, 1],        # Initial position (x, y, z)
        size=[0.5, 0.5, 0.5],      # Dimensions of the cube
        color=[0, 0, 1],           # Optional color (R, G, B)
    )
    world.scene.add(cube)

    # Add a default ground plane (so the cube doesn’t fall infinitely)
    world.scene.add_default_ground_plane()

    # Reset the simulation to place objects at their initial positions
    world.reset()

    # Main simulation loop
    while simulation_app.is_running():
        # Step the physics and rendering
        world.step(render=True)

    # Once the app is closed, cleanup
    simulation_app.close()

if __name__ == "__main__":
    main()
