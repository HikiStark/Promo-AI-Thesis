from lib.setup_import_standart import *
from lib.setup_import_standart import world
import cv2
# from omni.isaac.core import World
from omni.isaac.sensor import Camera

def listen_to_camera_feed(camera: Camera):
    # while world.simulation_app.is_running():
    for _ in range(1):
        world.step(render=True)

    # for _ in range(100):
    #     frame = camera.get_rgba()  # Returns an RGBA NumPy array
        # Convert from RGBA (or RGB) to BGR for OpenCV (drop alpha channel)
        # bgr_frame = cv2.cvtColor(frame[:, :, :3], cv2.COLOR_RGB2BGR)
        # cv2.imshow("Camera Feed", bgr_frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break