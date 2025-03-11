import numpy as np
import math

def quaternion_to_euler(q, degrees=False):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw).

    Parameters:
        q (array-like): Quaternion in the form [w, x, y, z].
        degrees (bool): If True, the output angles are in degrees. Otherwise, in radians.

    Returns:
        tuple: (roll, pitch, yaw)
    """
    w, x, y, z = q

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.pi / 2 * np.sign(sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    if degrees:
        roll = np.degrees(roll)
        pitch = np.degrees(pitch)
        yaw = np.degrees(yaw)
    
    return roll, pitch, yaw

if __name__ == '__main__':
    # Example quaternion: replace these values with your desired quaternion
    quat = [0.37118, 0.57839, 0.13886, 0.71303]
    
    # Convert the quaternion to Euler angles (in degrees)
    roll, pitch, yaw = quaternion_to_euler(quat, degrees=True)
    
    print("Input Quaternion [w, x, y, z]:", quat)
    print("Euler Angles (degrees):")
    print("  Roll:  {:.2f}".format(roll))
    print("  Pitch: {:.2f}".format(pitch))
    print("  Yaw:   {:.2f}".format(yaw))



