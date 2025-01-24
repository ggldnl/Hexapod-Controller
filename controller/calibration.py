from interface import Interface
import argparse
import time
from sshkeyboard import listen_keyboard

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Servo controller test script.")
parser.add_argument("--pulse_step", type=int, default=10, help="Step size for pulse adjustment.")
parser.add_argument("--pulse_min", type=int, default=500, help="Minimum pulse width (in microseconds).")
parser.add_argument("--pulse_max", type=int, default=2500, help="Maximum pulse width (in microseconds).")
args = parser.parse_args()

# Constants
PULSE_STEP = args.pulse_step
PULSE_MIN = args.pulse_min
PULSE_MAX = args.pulse_max
NUM_SERVOS = 18

# Initialize Interface
controller = Interface()

# Global variables
current_servo = 0
current_pulse = (PULSE_MAX + PULSE_MIN) // 2  # Start at the midpoint of the pulse range

# Function to handle key presses
def on_key_press(key):
    global current_servo, current_pulse

    if key == 'up':
        # Increment the pulse, but not beyond the maximum
        current_pulse = min(current_pulse + PULSE_STEP, PULSE_MAX)
        controller.set_pulse(current_servo, current_pulse)
        print(f"Servo {current_servo}: {current_pulse} µs")
        time.sleep(0.2)

    elif key == 'down':
        # Decrement the pulse, but not below the minimum
        current_pulse = max(current_pulse - PULSE_STEP, PULSE_MIN)
        controller.set_pulse(current_servo, current_pulse)
        print(f"Servo {current_servo}: {current_pulse} µs")
        time.sleep(0.2)

    elif key == 'enter':
        # Move to the next servo
        current_servo += 1
        if current_servo >= NUM_SERVOS:
            print("All servos tested. Exiting program...")
            raise KeyboardInterrupt
        current_pulse = (PULSE_MAX + PULSE_MIN) // 2  # Reset pulse to midpoint
        print(f"Switching to Servo {current_servo}")
        time.sleep(0.5)

    elif key == 'esc':
        # Exit the program
        print("Exiting program...")
        raise KeyboardInterrupt

def main():
    print("Connecting to servo controller...")
    controller.open()
    controller.attach_servos()
    print("Use UP/DOWN arrows to adjust pulse width, ENTER to switch to next servo, ESC to quit.")

    try:
        listen_keyboard(on_key_press, delay_second_char=0.05)
    except KeyboardInterrupt:
        print("\nDetaching servos and closing connection...")
        controller.detach_servos()
        controller.close()
        print("Connection closed.")

if __name__ == "__main__":
    main()
