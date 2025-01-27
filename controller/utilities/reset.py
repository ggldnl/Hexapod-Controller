from controller.interface import Interface

import argparse
import time

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Reset one or more servos to a specific angle.")
parser.add_argument("-m", "--mode", type=str, default="angle", help="Modality (angle/pulse).")
parser.add_argument("-v", "--value", type=int, default=0, help="Angle (in degrees) if in angle mode, pulse (in microseconds) otherwise.")
parser.add_argument("-l", "--leg", type=int, default=-1, help="Leg index. If -1, reset all servos.")
parser.add_argument("-j", "--joint", type=int, default=-1, help="Joint index. If -1, reset all servos in the leg.")
args = parser.parse_args()

# Initialize Interface
controller = Interface()

try:

    print("Connecting to servo controller...")
    controller.open()
    time.sleep(0.5)
    controller.attach_servos()

    if args.mode == "angle":
        if args.leg == -1:
            # Reset all servos
            controller.set_angles(list(range(18)), [args.value] * 18)
            print(f"Reset all servos to {args.value} degrees.")
        elif args.joint == -1:
            # Reset all joints in the specified leg
            for joint in range(3):  # Assuming 3 joints per leg
                controller.set_angle(args.leg * 3 + joint, args.value)
                print(f"Reset Leg {args.leg}, Joint {joint} to {args.value} degrees.")
        else:
            # Reset specific joint in the specified leg
            controller.set_angle(args.leg * 3 + args.joint, args.value)
            print(f"Reset Leg {args.leg}, Joint {args.joint} to {args.value} degrees.")
    elif args.mode == "pulse":
        if args.leg == -1:
            # Reset all servos
            controller.set_pulses(list(range(18)), [args.value] * 18)
            print(f"Reset all servos to {args.value} degrees.")
        elif args.joint == -1:
            # Reset all joints in the specified leg
            for joint in range(3):  # Assuming 3 joints per leg
                controller.set_pulses(args.leg * 3 + joint, args.value)
                print(f"Reset Leg {args.leg}, Joint {joint} to PWM {args.value}.")
        else:
            # Reset specific joint in the specified leg
            controller.set_pulse(args.leg * 3 + args.joint, args.value)
            print(f"Reset Leg {args.leg}, Joint {args.joint} to PWM {args.value}.")
    else:
        raise ValueError(f"Unsupported modality: {args.mode}. Only 'angle' and 'pulse' are supported.")

    time.sleep(0.5)

finally:
    # Detach servos and close connection
    controller.detach_servos()
    controller.close()
    print("Connection closed.")
