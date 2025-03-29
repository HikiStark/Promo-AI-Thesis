from isaacsim import SimulationApp

# Create and configure the simulation app.
simulation_app = SimulationApp(
    {
        "headless": False,
        # "window_width": 2400,
        # "window_height": 1500,
    }
)
# Import required modules after initializing the simulation app.
import json
from lib.setup_import_standart import *
from lib.setup_robot import setup_robot
import lib.setup_task as tasksetup
from lib.setup_task import initialize_command_receiver, initialize_publisher
from lib.tasks.robot_look_table import robot_look_at_table
from lib.robot_control_lib.robot_controller_function_lib import *


print("-" * 120 + "\n")

articulation_controller, my_controller_RMP, my_controller_PP = setup_robot()
world.reset()
world.step()
scene = tasksetup.set_the_scene(simulation_app)
camera_instance = scene.camera
print("camera_instance:", camera_instance)

# camera_instance.save_camera_frames()
socket = initialize_publisher()
cmd_socket = initialize_command_receiver()

print("\n" + "-" * 120)


def main() -> None:
    # Main simulation loop.
    external_control_active = False  # Flag to track external control
    reset_needed = False

    # Example: define a task
    target_position = np.array([0.5, 0.5, 0.3])
    target_orientation = np.array([0, 0, 0, 1])  # quaternion
    task_state = {"done": False}  # Track if task is complete

    while simulation_app.is_running():
        # Step the simulation with rendering enabled.
        world.step(render=True)

        # # Process external commands if available
        # if cmd_socket.poll(timeout=0):
        #     try:
        #         msg = cmd_socket.recv(zmq.NOBLOCK)
        #         command = json.loads(msg.decode("utf-8"))
        #         print("Received command:", command)
        #         if command.get("command") == "move":
        #             external_control_active = True
        #             target_position = np.array(command["target_position"])
        #             target_orientation = np.array(command["target_orientation"])
        #             # Debug: Print the target values
        #             print("Target Position:", target_position)
        #             print("Target Orientation:", target_orientation)
        #             # Compute the movement action
        #             actions = my_controller_RMP.forward(
        #                 target_end_effector_position=target_position,
        #                 target_end_effector_orientation=target_orientation,
        #             )
        #             # Debug: Print computed actions
        #             print("Computed Actions:", actions)
        #             articulation_controller.apply_action(actions)
        #     except Exception as e:
        #         print("Error processing command:", e)

        # ---------------------------------------------------------------------------------------------------------------------------------
        # Check if simulation is stopped to mark for reset.
        if world.is_stopped() and not reset_needed:
            reset_needed = True

        if world.is_playing():
            camera_instance.publish_camera_frames(socket)
            # If a reset is needed, reset the simulation and the robot controller.
            # if reset_needed:
            #     world.reset()
            #     my_controller_RMP.reset()
            #     reset_needed = False

            # If we still have a task in progress, keep moving the robot
            if not task_state["done"]:
                track_task_progress(articulation_controller, my_controller_RMP, target_position, target_orientation, task_state)

            else:
                # If we’re done, we could do something else or remain idle
                pass

            # # Only call default task if no external command is active.
            # if not external_control_active:
            #     # robot_look_at_table(articulation_controller, my_controller_RMP, my_controller_PP)
            #     print("Executing default task...")
            # else:
            #     # Optionally reset external control after a few steps or based on a condition.
            #     # For instance, reset the flag after applying the command for a few iterations.
            #     external_control_active = False

            # observations = world.get_observations()

    simulation_app.close()


if __name__ == "__main__":
    main()
