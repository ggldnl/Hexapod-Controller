import curses
import time
from interface import Interface
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


def main(stdscr):

    # Curses setup
    curses.curs_set(0)  # Hide cursor
    stdscr.clear()
    stdscr.nodelay(1)  # Make getch non-blocking
    stdscr.timeout(100)  # Refresh screen every 100 ms

    current_servo = 0
    current_angle = 0

    try:
        stdscr.addstr(0, 0, "Connecting to servo controller...")
        controller.open()
        controller.attach_servos()
        stdscr.addstr(1, 0, "Use UP/DOWN arrows to adjust angle, ENTER to switch to next servo, ESC to quit.")

        while True:
            # Display current servo and angle
            angle_radians = np.deg2rad(current_angle)
            stdscr.addstr(3, 0, f"Servo {current_servo}: {current_angle}° ({angle_radians:.2f} rad)")
            stdscr.refresh()

            # Handle key input
            key = stdscr.getch()
            if key == curses.KEY_UP:
                current_angle = min(current_angle + ANGLE_STEP, ANGLE_MAX)
                controller.set_angle(current_servo, current_angle)
                time.sleep(0.2)

            elif key == curses.KEY_DOWN:
                current_angle = max(current_angle - ANGLE_STEP, ANGLE_MIN)
                controller.set_angle(current_servo, current_angle)
                time.sleep(0.2)

            elif key == 10:  # Enter key
                current_servo += 1
                if current_servo >= NUM_SERVOS:
                    break
                current_angle = 0
                stdscr.addstr(4, 0, f"Switching to Servo {current_servo}")
                time.sleep(0.5)

            elif key == 27:  # Escape key
                stdscr.addstr(5, 0, "Exiting program...")
                break

    finally:
        # Detach servos and close connection
        controller.detach_servos()
        controller.close()
        stdscr.addstr(6, 0, "Connection closed.")
        stdscr.refresh()
        time.sleep(1)


# Run the curses application
curses.wrapper(main)
