import numpy as np
import time
import math

def compute_displacement(accel, time_delay=0.1, scale_factor=100):
    """
    Compute the displacement from acceleration in g's to pixels.
    
    Parameters:
    accel_x_g (np.array): Acceleration in the x direction (in g's).
    accel_y_g (np.array): Acceleration in the y direction (in g's).
    time_delay (float): Time delay between samples (in seconds).
    scale_factor (float): Number of pixels per meter for conversion. Default is 100 pixels/meter.
    
    Returns:
    displacement_x_pixels (np.array): Displacement in pixels along x-axis.
    displacement_y_pixels (np.array): Displacement in pixels along y-axis.
    """
    
    # Convert g's to m/s²
    g = 9.81
    accel = accel * g
    
    # Integrating acceleration to get velocity (numerical integration)
    velocity = np.cumsum(accel) * time_delay  # velocity = sum(acceleration * dt)

    # Integrating velocity to get displacement (numerical integration)
    displacement = np.cumsum(velocity) * time_delay  # displacement = sum(velocity * dt)

    # Convert displacement from meters to pixels
    displacement_pixels = displacement * scale_factor

    return displacement_pixels

def getGrav(pitch, roll, yaw)->list:
    return (math.cos(math.radians(pitch)), math.cos(math.radians(roll)), math.cos(math.radians(yaw)))
