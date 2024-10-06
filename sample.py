import numpy as np
from pyfirmata import Arduino, util
import time

# Initialize Arduino connection
board = Arduino('COM5')  # Change this to your Arduino port
it = util.Iterator(board)
it.start()

# Define the servo pins (update these according to your setup)
servo_pin_1 = board.get_pin('d:9:s')  # Servo 1 connected to pin 9
servo_pin_2 = board.get_pin('d:10:s')  # Servo 2 connected to pin 10
servo_pin_3 = board.get_pin('d:11:s')  # Servo 3 connected to pin 11

def move_servo(servo_pin, angle):
    """ Move the servo to the specified angle. """
    servo_pin.write(angle)
    time.sleep(0.015)  # Allow time for the servo to reach the angle

def calculate_inverse_kinematics(midpoint_x_origin, midpoint_y_origin, midpoint_z_origin):
    """
    Calculate the inverse kinematics for a 2DOF robotic arm.

    Parameters:
    midpoint_x_origin (float): The x-coordinate of the target point.
    midpoint_y_origin (float): The y-coordinate of the target point.
    midpoint_z_origin (float): The z-coordinate of the target point.

    Returns:
    tuple: The angles (theta_1, theta_2) for the servos in degrees.
    """
    a_1 = 35  # Length from base to shoulder
    a_2 = 40  # Length from shoulder to end-effector

    # Calculate servo angles
    theta_1 = np.degrees(np.arctan2(midpoint_y_origin, midpoint_x_origin))
    r_1 = np.sqrt(midpoint_x_origin ** 2 + midpoint_y_origin ** 2)
    r_2 = midpoint_z_origin - a_1
    r = np.sqrt(r_1 ** 2 + r_2 ** 2)

    # Calculate theta_2
    cos_theta_2 = (a_1 ** 2 + a_2 ** 2 - r ** 2) / (2 * a_1 * a_2)

    # Ensure cos_theta_2 is within the valid range for arccos
    cos_theta_2 = np.clip(cos_theta_2, -1, 1)

    theta_2 = np.degrees(np.arccos(cos_theta_2))
    phi_2 = np.degrees(np.arctan2(r_2, r_1))
    theta_2 += phi_2  # Adjust theta_2 based on phi_2

    return theta_1, theta_2

def controller(midpoint_x_origin, midpoint_y_origin, midpoint_z_origin):
    """ Control the robotic arm based on the calculated angles. """
    # Calculate angles using inverse kinematics
    theta_1, theta_2 = calculate_inverse_kinematics(midpoint_x_origin, midpoint_y_origin, midpoint_z_origin)

    # Print the calculated angles
    print(f"Theta 1: {theta_1:.2f} degrees")
    print(f"Theta 2: {theta_2:.2f} degrees")

    # Servo movement logic
    initial_angle = 90
    target_angle = int(initial_angle + theta_1)
    step = 2

    # 2nd and 3rd servo initial angles
    initial_angle_1 = 0
    initial_angle_2 = 180
    target_angle_1 = int(initial_angle_1 + theta_2)
    target_angle_2 = int(180 - theta_2)

    # Move servo 1
    if target_angle > initial_angle:
        for angle in range(initial_angle, target_angle, step):
            move_servo(servo_pin_1, angle)
    elif target_angle < initial_angle:
        for angle in range(initial_angle, target_angle, -step):
            move_servo(servo_pin_1, angle)
    else:  # If target angle is equal to initial angle
        move_servo(servo_pin_1, initial_angle)

    # Move servos 2 and 3
    if target_angle_1 > initial_angle_1:
        for angle in range(initial_angle_1, target_angle_1, step):
            move_servo(servo_pin_2, angle)
        for angle in range(initial_angle_2, target_angle_2, -step):
            move_servo(servo_pin_3, angle)
    else:
        move_servo(servo_pin_3, initial_angle_1)
        move_servo(servo_pin_2, initial_angle_2)

# Main loop for user input
while True:
    try:
        # Get user input for coordinates
        midpoint_x_origin = float(input("Enter the x-coordinate: "))
        midpoint_y_origin = float(input("Enter the y-coordinate: "))
        midpoint_z_origin = float(input("Enter the z-coordinate: "))

        # Call the controller function
        controller(midpoint_x_origin, midpoint_y_origin, midpoint_z_origin)

    except ValueError:
        print("Invalid input. Please enter numeric values.")
    except KeyboardInterrupt:
        print("Exiting the program.")
        break
