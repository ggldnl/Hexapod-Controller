import time
from interface import Interface
import keyboard
import argparse
import numpy as np

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Servo controller test script.")
parser.add_argument("--angle_step", type=int, default=5, help="Step size for angle adjustment (in degrees).")
parser.add_argument("--angle_min", type=int, default=-90, help="Minimum angle in degrees.")
parser.add_argument("--angle_max", type=int, default=90, help="Maximum angle in degrees.")
args = parser.parse_args()

# Constants
ANGLE_STEP = args.angle_step
ANGLE_MIN = args.angle_min
ANGLE_MAX = args.angle_max
NUM_SERVOS = 18

# Initialize Interface
controller = Interface()
current_servo = 0
current_angle = 0

try:
    print("Connecting to servo controller...")
    controller.open()
    controller.attach_servos()
    print("Use UP/DOWN arrows to adjust angle, ENTER to switch to next servo, ESC to quit.")

    while True:

        angle_radians = np.deg2rad(current_angle)
        print(f"Servo {current_servo}: {current_angle}° ({angle_radians:.2f} rad)\r", end="", flush=True)

        if keyboard.is_pressed("up"):
            current_angle = min(current_angle + ANGLE_STEP, ANGLE_MAX)
            controller.set_angle(current_servo, np.deg2rad(current_angle))
            time.sleep(0.2)

        elif keyboard.is_pressed("down"):
            current_angle = max(current_angle - ANGLE_STEP, ANGLE_MIN)
            controller.set_angle(current_servo, np.deg2rad(current_angle))
            time.sleep(0.2)

        # Move to next servo with ENTER key
        elif keyboard.is_pressed("enter"):
            current_servo += 1
            if current_servo >= NUM_SERVOS:
                break
            current_angle = 0
            print(f"\nSwitching to Servo {current_servo}")
            time.sleep(0.5)

        # Exit the program with ESC key
        elif keyboard.is_pressed("esc"):
            print("\nExiting program...")
            break

finally:
    # Detach servos and close connection
    controller.detach_servos()
    controller.close()
    print("Connection closed.")
