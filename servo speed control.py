import pyfirmata
import time

# Adjust if your Arduino is on a different port
board = pyfirmata.Arduino('COM5')

# Pin where your servo is connected
servo_pin = 9

# Start an iterator for smooth motion
it = pyfirmata.util.Iterator(board)
it.start()

board.digital[servo_pin].mode = pyfirmata.SERVO

target_angle = 120
current_angle = 0
increment = 2  # Adjust for smoother motion

while current_angle < target_angle:
    board.digital[servo_pin].write(current_angle)
    current_angle += increment
    time.sleep(0.02)  # Adjust delay for speed control
