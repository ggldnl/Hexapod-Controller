from ..interface import Interface

import argparse
import time
import csv
from sshkeyboard import listen_keyboard

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Servo controller test script.")
parser.add_argument("--small_pulse_step", type=int, default=5, help="Small step size for pulse adjustment.")
parser.add_argument("--large_pulse_step", type=int, default=100, help="Large step size for pulse adjustment.")
parser.add_argument("--min_pulse", type=int, default=500, help="Minimum pulse width (in microseconds).")
parser.add_argument("--max_pulse", type=int, default=2500, help="Maximum pulse width (in microseconds).")
parser.add_argument("--num_servos", type=int, default=18, help="Number of servos to calibrate.")
parser.add_argument("--output_path", type=str, default="./servo_pulses.csv", help="Path to save the CSV file.")
args = parser.parse_args()

# Constants
SMALL_PULSE_STEP = args.small_pulse_step
LARGE_PULSE_STEP = args.large_pulse_step
PULSE_MIN = args.min_pulse
PULSE_MAX = args.max_pulse
NUM_SERVOS = args.num_servos
OUTPUT_PATH = args.output_path

# Initialize Interface
controller = Interface()

# Global variables
current_servo = 0
current_pulse = (PULSE_MIN + PULSE_MAX) / 2  # Start around 0 deg

min_pulse = None
max_pulse = None

# Data for CSV
servo_pulses = []

# We have to raise a KeyboardInterrupt only once
done = False

# Function to handle key presses
def on_key_press(key):
    global current_servo, current_pulse, min_pulse, max_pulse, done

    if key == 'down':
        # Increment the pulse, but not beyond the maximum
        current_pulse = min(current_pulse + SMALL_PULSE_STEP, PULSE_MAX)
        controller.set_pulse(current_servo, current_pulse)
        print_status()
        time.sleep(0.2)

    elif key == 'up':
        # Decrement the pulse, but not below the minimum
        current_pulse = max(current_pulse - SMALL_PULSE_STEP, PULSE_MIN)
        controller.set_pulse(current_servo, current_pulse)
        print_status()
        time.sleep(0.2)

    elif key == 'right':
        # Decrease the pulse by a very large amount
        current_pulse = max(current_pulse - LARGE_PULSE_STEP, PULSE_MIN)
        controller.set_pulse(current_servo, current_pulse)
        print_status()
        time.sleep(0.2)

    elif key == 'left':
        # Increase the pulse by a very large amount
        current_pulse = min(current_pulse + LARGE_PULSE_STEP, PULSE_MAX)
        controller.set_pulse(current_servo, current_pulse)
        print_status()
        time.sleep(0.2)

    elif key == 'enter':
        # Register the current pulse and move to the next step or servo
        if min_pulse is None:
            min_pulse = current_pulse
        elif max_pulse is None:
            max_pulse = current_pulse
            mid_pulse = (min_pulse + max_pulse) // 2
            servo_pulses.append([min_pulse, mid_pulse, max_pulse])
            print(f"Servo {current_servo}: min pulse = {min_pulse}   mid pulse = {mid_pulse}   max pulse = {max_pulse}")
            # Reset for next servo
            current_servo += 1
            if current_servo >= NUM_SERVOS:
                print("All servos calibrated. Saving results and exiting program...")
                save_to_csv()
                if not done:
                    done = True
                    raise KeyboardInterrupt
            min_pulse = None
            max_pulse = None
            current_pulse = (PULSE_MIN + PULSE_MAX) / 2  # Reset pulse to midpoint
            controller.set_pulse(current_servo, current_pulse)
        time.sleep(0.5)

    elif key == 'esc':
        # Exit the program
        print("Exiting program...")
        save_to_csv()
        raise KeyboardInterrupt

# Function to save pulses to CSV
def save_to_csv():
    with open(OUTPUT_PATH, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Min Pulse", "Mid Pulse", "Max Pulse"])
        writer.writerows(servo_pulses)
    print(f"Pulse data saved to {OUTPUT_PATH}")

# Function to print status on the same line
def print_status():
    extra_spaces = ' ' * 10  # Needed to clear the line
    if min_pulse is None:
        print(f"Servo {current_servo}: min pulse = {current_pulse:5.2f}{extra_spaces}\r", end="")
    elif max_pulse is None:
        print(f"Servo {current_servo}: min pulse = {min_pulse:5.2f}\tmax pulse = {current_pulse:5.2f}{extra_spaces}\r", end="")
    else:
        mid_pulse = (min_pulse + max_pulse) // 2
        print(f"Servo {current_servo}: min pulse = {min_pulse:5.2f}\tmax pulse = {max_pulse:5.2f}\tmid pulse = {mid_pulse:5.2f}{extra_spaces}\r", end="")

# Main function
def main():

    print("Connecting to servo controller...")
    controller.open()
    controller.attach_servos()

    # Set the first servo to approximately the center position
    controller.set_pulse(current_servo, current_pulse)

    print("Use UP/DOWN arrows to adjust pulse width, LEFT/RIGHT for large adjustments, ENTER to record pulse, ESC to quit.")

    try:
        listen_keyboard(on_key_press, delay_second_char=0.05)
    except KeyboardInterrupt:
        print("\nDetaching servos and closing connection...")
        controller.detach_servos()
        controller.close()
        print("Connection closed.")

if __name__ == "__main__":
    main()
