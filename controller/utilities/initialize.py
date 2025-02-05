from path_setup import *
from controller.interface import Interface

import argparse
import csv


def read_mid_pulses(csv_file_path, has_header):
    """Read mid-pulse values from the CSV file."""
    mid_pulses = []
    with open(csv_file_path, mode="r") as file:
        reader = csv.reader(file)
        if has_header:
            next(reader)  # Skip header row
        for row in reader:
            mid_pulses.append(float(row[1]))  # Mid pulse is the second column
    return mid_pulses


def main():

    parser = argparse.ArgumentParser(description="Set servos to their mid-pulse values.")
    parser.add_argument("--path", type=str, default="./servo_pulses.csv", help="Path to the CSV file.")
    parser.add_argument("--header", type=bool, default=True, help="Specify if the CSV file has a header.")
    args = parser.parse_args()

    print("Connecting to servo controller...")
    controller = Interface()
    controller.open()
    controller.attach_servos()

    try:
        # Read mid-pulse values from the CSV file
        mid_pulses = read_mid_pulses(args.path, args.header)
        print(f"Mid pulses loaded: {mid_pulses}")

        # Set each servo to its mid-pulse value
        for servo_id, mid_pulse in enumerate(mid_pulses):
            print(f"Setting servo {servo_id} to mid pulse {mid_pulse}")
            controller.set_pulse(servo_id, mid_pulse)

        print("All servos set to their mid-pulse values.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Detaching servos and closing connection...")
        controller.detach_servos()
        controller.close()
        print("Connection closed.")

if __name__ == '__main__':
    main()
