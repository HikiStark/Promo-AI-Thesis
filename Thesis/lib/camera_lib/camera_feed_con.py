# server.py: Run this inside the Isaac Sim environment
import socket
import cv2
import numpy as np
import struct
import time
import omni.syntheticdata
import omni.kit.app

def get_camera_image():
    # Acquire the synthetic data interface
    sd = omni.syntheticdata._syntheticdata
    sdi = sd.acquire_syntheticdata_interface()

    # Optionally, update the simulation a few frames to ensure sensor buffers are populated
    app = omni.kit.app.get_app_interface()
    for _ in range(2):
        app.update(0.0)

    # Get the sensor parameters (assuming you are using the default RGB sensor)
    sensor_type = sd.SensorType.Rgb
    width = sdi.get_sensor_width(sensor_type)
    height = sdi.get_sensor_height(sensor_type)
    row_size = sdi.get_sensor_row_size(sensor_type)

    # Retrieve the image data (as a compressed uint32 array)
    data = sdi.get_sensor_host_uint32_texture_array(sensor_type, width, height, row_size)

    # Convert the data into a NumPy array and reshape it.
    # The exact reshaping depends on your sensor setup (here we assume 3 channels)
    image = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 4))
    
    # If the image has an alpha channel, you might want to drop it:
    image = image[:, :, :3]
    
    # OpenCV uses BGR by default; convert if needed:
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    return image

def main():
    host = '0.0.0.0'  # Listen on all available interfaces
    port = 8000       # Port number to use

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print("Waiting for connection...")

    conn, addr = server_socket.accept()
    print(f"Connected by {addr}")

    try:
        while True:
            # Capture image from the camera sensor
            frame = get_camera_image()

            # Encode the frame as JPEG to compress the data
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                print("Failed to encode image")
                continue
            data = buffer.tobytes()

            # Send the length of the data first (4 bytes, network byte order)
            length = len(data)
            conn.sendall(struct.pack('!I', length))
            # Then send the image data itself
            conn.sendall(data)

            # Wait a bit to mimic a desired frame rate (e.g., ~30 FPS)
            time.sleep(0.033)
    except Exception as e:
        print("Error:", e)
    finally:
        conn.close()
        server_socket.close()

if __name__ == '__main__':
    main()
    
    while True:
        frame = get_camera_image()
        cv2.imshow("Isaac Sim Camera Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
