import os
from ultralytics import YOLO
import cv2
import numpy as np
import pyfirmata

# Connect to Arduino
board = pyfirmata.Arduino('COM6')  # Change port as necessary
servo_pin_1 = 9
servo_pin_2 = 10
servo_pin_3 = 11
servo_pin_4 = 12

it = pyfirmata.util.Iterator(board)
it.start()

board.digital[servo_pin_1].mode = pyfirmata.SERVO
board.digital[servo_pin_2].mode = pyfirmata.SERVO
board.digital[servo_pin_3].mode = pyfirmata.SERVO
board.digital[servo_pin_4].mode = pyfirmata.SERVO

# Function to move servo to a specific angle
def move_servo(pin, angle):
    # Ensure angle is within the valid range
    angle = max(min(angle, 180), 0)  # Clamp angle between 0 and 180 degrees

    # Write angle value to the servo pin
    board.digital[pin].write(angle)

def move_servo_1(pin, angle_2):
    angle_2 = max(min(angle_2, 180), 0)

    board.digital[pin].write(angle_2)

# Known length in pixels and centimeters
known_length_pixels = 475
known_length_cm = 100
# Calculate conversion factor
conversion_factor = known_length_cm / known_length_pixels

# Function to calculate inverse kinematics
def calculate_inverse_kinematics(midpoint_x_origin, midpoint_y_origin, midpoint_z_origin):
    # Length of the links
    a_1 = 20
    a_2 = 20
    a_3 = 10

    # Calculate servo angles
    theta_1 = np.degrees(np.arctan2(midpoint_y_origin, midpoint_x_origin))
    r_1 = np.sqrt(midpoint_x_origin ** 2 + midpoint_y_origin ** 2)
    r_2 = midpoint_z_origin - a_1
    phi_2 = np.arctan2(r_2, r_1)
    numerator = a_3 ** 2 - a_2 ** 2 - r_1 ** 2 - r_2 ** 2
    denominator = -2 * a_2 * np.sqrt(r_1 ** 2 + r_2 ** 2)
    phi_1 = np.arccos(np.clip(numerator / denominator, -1, 1))
    theta_2 = np.degrees(phi_1 - phi_2)
    numerator = r_1 ** 2 + r_2 ** 2 - a_2 ** 2 - a_3 ** 2
    denominator = -2 * a_2 * a_3
    phi_3 = np.arccos(np.clip(numerator / denominator, -1, 1))
    theta_3 = np.degrees(np.pi - phi_3)

    return theta_1, theta_2, theta_3

model_path = os.path.join('.', 'model_- 13 february 2024 21_57.pt')
model = YOLO(model_path)

threshold = 0.5

image_directory = 'image'  # Replace with the path to your image directory

# Iterate through image files in the specified directory
for image_file in os.listdir(image_directory):
    if image_file.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp')):
        image_path = os.path.join(image_directory, image_file)

        # Read the image
        frame = cv2.imread(image_path)

        # Calculate the center of the frame
        frame_height, frame_width, _ = frame.shape
        bottom_center_x = frame_width // 2
        bottom_center_y = frame_height - 1

        # Draw X and Y axes lines
        cv2.line(frame, (0, bottom_center_y), (frame_width, bottom_center_y), (0, 0, 255), 2)  # X-axis (Red)
        cv2.line(frame, (bottom_center_x, 0), (bottom_center_x, frame_height), (0, 255, 0), 2)  # Y-axis (Green)

        # Draw a small circle representing the new origin (midpoint of bottom horizontal line)
        cv2.circle(frame, (bottom_center_x, bottom_center_y), 5, (255, 0, 0), -1)

        # Perform object detection
        results = model(frame)[0]

        # Process detections
        for result in results.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result

            if score > threshold:
                class_name = results.names[int(class_id)].lower()

                # Handle detections based on class and score
                if class_name == "plant":
                    # Handle crop detection (e.g., draw bounding box, display details)
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
                    cv2.putText(frame, "Crop", (int(x1), int(y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3)

                elif class_name == "weed":
                    # Handle weed detection (e.g., highlight, trigger actions)
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    # Calculate the midpoint
                    midpoint_x = (x1 + x2) // 2
                    midpoint_y = (y1 + y2) // 2

                    # Calculate the coordinates of the midpoint with respect to the origin
                    midpoint_y_origin = -(midpoint_x - bottom_center_x) * conversion_factor
                    midpoint_x_origin = (bottom_center_y - midpoint_y) * conversion_factor  # Invert y-coordinate as origin is at bottom
                    midpoint_z_origin = 10 * conversion_factor

                    #inverse kinematics

                    theta_1, theta_2, theta_3 = calculate_inverse_kinematics(midpoint_x_origin, midpoint_y_origin, midpoint_z_origin)

                    #servos tuurning
                    #first servo
                    initial_angle = 90
                    target_angle = int(initial_angle + theta_1)
                    step = 2
                    #2nd servo and 3rd servo
                    initial_angle_1 = 0
                    initial_angle_3 = 180
                    target_angle_1 = int(initial_angle_1 + theta_2)
                    target_angle_3 = int(180 - theta_2)
                    #4th servo turning
                    initial_angle_2 = 45
                    target_angle_2 = int(initial_angle_2 + theta_3)

                    if target_angle > initial_angle:
                        for angle in range(initial_angle, int(target_angle), step):
                            move_servo(servo_pin_1, angle)

                    elif target_angle < initial_angle:
                        for angle in range(initial_angle, int(target_angle), -step):
                            move_servo(servo_pin_1, angle)

                    else:  # If target angle is equal to initial angle
                        move_servo(initial_angle)

                    if target_angle_1 > initial_angle_1:
                        for angle in range(initial_angle_1, int(target_angle_1), step):
                            move_servo(servo_pin_2, angle)
                        for angle in range(initial_angle_3, int(target_angle_3), -step):
                            move_servo(servo_pin_3, angle)
                    else:
                        move_servo(servo_pin_3, initial_angle_3)
                        move_servo(servo_pin_2, initial_angle_2)



                    # Print the midpoint coordinates with respect to the origin
                    print("Midpoint coordinates with respect to origin:", midpoint_x_origin, midpoint_y_origin, midpoint_z_origin)

                    # Draw the rectangle and text
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 4)
                    cv2.putText(frame, "Weed", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 255), 3)

                    # Draw line from the new origin to the midpoint of the bounding box
                    cv2.line(frame, (bottom_center_x, bottom_center_y), (midpoint_x, midpoint_y), (0, 255, 0), 2)

                    break

        # Display the frame with detections and lines
        cv2.imshow("Image Detection", frame)
        cv2.waitKey(0)  # Press any key to proceed to the next image

# Close the OpenCV window
cv2.destroyAllWindows()
board.exit()
