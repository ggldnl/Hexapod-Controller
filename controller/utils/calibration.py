"""
Calibration utility to find the actual min and max pulses of each servo.
"""

from sshkeyboard import listen_keyboard
import argparse
import time
import csv

from controller.hardware.kernel import Kernel


class ServoCalibrator:
    """Manages servo calibration process with keyboard controls."""

    # TODO give this class a cleanup once you test it on the Raspberry

    def __init__(self, kernel, small_step=5, large_step=100, min_pulse=500,
                 max_pulse=2500, num_servos=18, output_path="./servo_pulses.csv"):
        """Initialize calibrator with kernel and configuration parameters."""
        self.kernel = kernel
        self.small_step = small_step
        self.large_step = large_step
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.num_servos = num_servos
        self.output_path = output_path

        self.current_servo = 0
        self.current_pulse = (min_pulse + max_pulse) / 2
        self.servo_min_pulse = None
        self.servo_max_pulse = None
        self.servo_pulses = []
        self.done = False

    def print_status(self):
        """Display current calibration status on same line."""

        if self.servo_min_pulse is None:
            status = f"Servo {self.current_servo:3d}: min pulse = {self.current_pulse:7.2f}"
        elif self.servo_max_pulse is None:
            status = (f"Servo {self.current_servo:3d}: min pulse = {self.servo_min_pulse:7.2f}  "
                      f"max pulse = {self.current_pulse:7.2f}")
        else:
            mid_pulse = (self.servo_min_pulse + self.servo_max_pulse) / 2
            status = (f"Servo {self.current_servo:3d}: min pulse = {self.servo_min_pulse:7.2f}  "
                      f"max pulse = {self.servo_max_pulse:7.2f}  mid pulse = {mid_pulse:7.2f}")

        print(f"{status}\r", end="", flush=True)

    def adjust_pulse(self, step):
        """Adjust current pulse by given step, respecting boundaries."""
        self.current_pulse = max(
            self.min_pulse,
            min(self.current_pulse + step, self.max_pulse)
        )
        self.kernel.set_servo_pulse_width(self.current_servo, self.current_pulse)
        self.print_status()
        time.sleep(0.05)

    def register_pulse(self):
        """Register current pulse as min or max, advance to next servo when complete."""
        if self.servo_min_pulse is None:
            self.servo_min_pulse = self.current_pulse
            self.print_status()
        elif self.servo_max_pulse is None:
            self.servo_max_pulse = self.current_pulse
            mid_pulse = (self.servo_min_pulse + self.servo_max_pulse) / 2
            self.servo_pulses.append([self.servo_min_pulse, mid_pulse, self.servo_max_pulse])
            self.print_status()
            print()

            self.advance_to_next_servo()

    def advance_to_next_servo(self):
        """Move to next servo or finish calibration if all complete."""
        self.current_servo += 1

        if self.current_servo >= self.num_servos:
            print("\nAll servos calibrated!")
            print(f"Saving results to {self.output_path}...")
            self.save_to_csv()
            if not self.done:
                self.done = True
                raise KeyboardInterrupt

        self.servo_min_pulse = None
        self.servo_max_pulse = None
        self.current_pulse = (self.min_pulse + self.max_pulse) / 2
        self.kernel.set_servo_pulse_width(self.current_servo, self.current_pulse)
        self.print_status()

    def save_to_csv(self):
        """Write calibration data to CSV file."""
        with open(self.output_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Min Pulse", "Mid Pulse", "Max Pulse"])
            writer.writerows(self.servo_pulses)
        print(f"Pulse data saved to {self.output_path}")

    def on_key_press(self, key):
        """Handle keyboard input for calibration control."""
        if key == 'up' or key == 'w':
            self.adjust_pulse(self.small_step)
        elif key == 'down' or key == 's':
            self.adjust_pulse(-self.small_step)
        elif key == 'left' or key == 'a':
            self.adjust_pulse(-self.large_step)
        elif key == 'right' or key == 'd':
            self.adjust_pulse(self.large_step)
        elif key == 'enter':
            self.register_pulse()

    def run(self):
        """Start calibration process."""
        print("Connecting to Kernel...")
        self.kernel.attach_servos()
        self.kernel.set_servo_pulse_width(self.current_servo, self.current_pulse)

        print("\nCalibration Controls:")
        print("  [↑/↓]:    Small pulse adjustments (±{})".format(self.small_step))
        print("  [←/→]:    Large pulse adjustments (±{})".format(self.large_step))
        print("  [Enter]:  Record current pulse")
        print("  [Esc]:    Save and quit\n")

        self.print_status()

        try:
            listen_keyboard(self.on_key_press, delay_second_char=0.05)
        except KeyboardInterrupt:
            print("\n\nExiting and saving progress...")
            if self.servo_pulses:
                self.save_to_csv()
            self.kernel.detach_servos()
            self.kernel.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Servo calibration: find actual min and max pulses for each servo."
    )
    parser.add_argument('--small_pulse_step', '-s', type=int, default=5,
                        help="Small step size for pulse adjustment.")
    parser.add_argument('--large_pulse_step', '-l', type=int, default=100,
                        help="Large step size for pulse adjustment.")
    parser.add_argument('--min_pulse', '-u', type=int, default=500,
                        help="Minimum pulse width (in microseconds).")
    parser.add_argument('--max_pulse', '-U', type=int, default=2500,
                        help="Maximum pulse width (in microseconds).")
    parser.add_argument('--num_servos', '-n', type=int, default=18,
                        help="Number of servos to calibrate.")
    parser.add_argument('--output_path', '-o', type=str, default="./servo_pulses.csv",
                        help="Path to save the CSV file.")
    parser.add_argument('--port', '-p', type=str, default='/dev/ttyAMA0',
                        help="Serial port for Servo2040 communication.")
    parser.add_argument('--baud', '-b', type=int, default=115200,
                        help="Baud rate for serial communication.")
    args = parser.parse_args()

    kernel = Kernel(port=args.port, baud=args.baud)
    calibrator = ServoCalibrator(
        kernel,
        small_step=args.small_pulse_step,
        large_step=args.large_pulse_step,
        min_pulse=args.min_pulse,
        max_pulse=args.max_pulse,
        num_servos=args.num_servos,
        output_path=args.output_path
    )
    calibrator.run()