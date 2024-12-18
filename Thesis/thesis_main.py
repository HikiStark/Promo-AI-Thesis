from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
from lib.setup_import_standart import *
from lib.setup_robot import setup_robot
import lib.setup_task as tasksetup

# from lib.object_detect_lib.cube_detect import detect_edges_pic as detect_box


tasksetup.set_the_scene()
setup_robot()

# Move the robot to initial home position on top of the table
# table = tasksetup.table
# table_home_pos = table.get_local_pose()[0]
# tasksetup.setup_robot.articulation_controller.apply_action(table_home_pos)


# 5-7. Tasks: Detect boxes, estimate grip points, pick and place
reset_needed = False
while simulation_app.is_running():
    world.step(render=True)

    if world.is_stopped() and not reset_needed:
        reset_needed = True

    # if my_world.is_playing():
    #     if reset_needed:
    #         my_world.reset()
    #         my_controller_RMP.reset()
    #         reset_needed = False
    #     observations = my_world.get_observations()

    #     # 6. Task 2: Detect boxes and estimate grip points

    #     # Example: Move to a specific target position
    #     target_position = np.array([0.5, 0.5, 0.3])  # Define specific target position
    #     target_orientation = np.array(
    #         [0, 0, 0, 1]
    #     )  # Define specific target orientation (quaternion)

    #     actions = my_controller_RMP.forward(
    #         target_end_effector_position=target_position,
    #         target_end_effector_orientation=target_orientation,
    #     )

    #     # Execute actions
    #     articulation_controller.apply_action(actions)


# 8. Close the simulation
simulation_app.close()
