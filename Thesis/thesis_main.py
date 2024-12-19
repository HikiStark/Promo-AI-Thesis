from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
from lib.setup_import_standart import *
from lib.setup_robot import setup_robot
import lib.setup_task as tasksetup

# from lib.object_detect_lib.cube_detect import detect_edges_pic as detect_box


articulation_controller, my_controller_RMP = setup_robot()
tasksetup.set_the_scene()



# 5-7. Tasks: Detect boxes, estimate grip points, pick and place
reset_needed = False
while simulation_app.is_running():
    world.step(render=True)

    if world.is_stopped() and not reset_needed:
        reset_needed = True

    if world.is_playing():
        if reset_needed:
            world.reset()
            my_controller_RMP.reset()
            reset_needed = False
        observations = world.get_observations()

    #     # Example: Move to a specific target position
        target_position = np.array([0.5, 0.5, 0.3])  # Define specific target position
        target_orientation = np.array(
            [0, 0, 0, 1]
        )  # Define specific target orientation (quaternion)

        actions = my_controller_RMP.forward(
            target_end_effector_position=target_position,
            target_end_effector_orientation=target_orientation,
        )

        # Execute actions
        articulation_controller.apply_action(actions)


# 8. Close the simulation
simulation_app.close()
