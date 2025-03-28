from isaacsim import SimulationApp

# Create and configure the simulation app.
simulation_app = SimulationApp(
    {
        "headless": False,
        "window_width": 2400,
        "window_height": 1500,
    }
)
# Import required modules after initializing the simulation app.
from lib.setup_import_standart import *
from lib.setup_robot import setup_robot
import lib.setup_task as tasksetup
import zmq
from lib.tasks.robot_look_table import robot_look_at_table

# Set up ZMQ publisher
context = zmq.Context()


def initialize_publisher():
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.SNDHWM, 1)  # Limit backlog
    socket.bind("tcp://*:5555")
    return socket


print("-" * 120 + "\n")

articulation_controller, my_controller_RMP, my_controller_PP = setup_robot()
scene = tasksetup.set_the_scene(simulation_app)
camera_instance = scene.camera
print("camera_instance:", camera_instance)

print("\n" + "-" * 120)

# camera_instance.save_camera_frames()
socket = initialize_publisher()


def main() -> None:
    # Main simulation loop.
    reset_needed = False
    while simulation_app.is_running():
        # Step the simulation with rendering enabled.
        world.step(render=True)

        # Check if simulation is stopped to mark for reset.
        if world.is_stopped() and not reset_needed:
            reset_needed = True

        if world.is_playing():
            camera_instance.publish_camera_frames(socket)
            # If a reset is needed, reset the simulation and the robot controller.
            if reset_needed:
                world.reset()
                my_controller_RMP.reset()
                reset_needed = False

            # observations = world.get_observations()  # Retrieve current observations (for potential use).

            # Execute the task where the robot looks at the table.
            robot_look_at_table(articulation_controller, my_controller_RMP, my_controller_PP)

            # Example: Move to a specific target position
            # target_position = np.array([0.5, 0.5, 0.3])  # Define specific target position
            # target_orientation = np.array([0, 0, 0, 1])  # Define specific target orientation (quaternion)
            # actions = my_controller_RMP.forward(target_end_effector_position=target_position, target_end_effector_orientation=target_orientation,)
            # # Execute actions
            # articulation_controller.apply_action(actions)

    # Close the simulation when finished.
    simulation_app.close()


if __name__ == "__main__":
    main()
