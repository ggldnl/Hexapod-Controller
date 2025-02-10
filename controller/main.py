import numpy as np
import argparse
import time
import json

from controller import Controller
from interface import Interface
from hexapod import Hexapod


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Hexapod controller.")
    parser.add_argument("-c", "--config", type=str, default='config/config.json', help="Path to the robot's configuration file")
    parser.add_argument('-n', '--name', type=str, default='hexapod', help="Name of the robot in the config")
    parser.add_argument('-d', '--dt', type=float, default=0.02, help="Time delta for update (default=0.02=50Hz)")
    parser.add_argument('-p', '--port', type=str, default='/dev/ttyAMA0', help="Serial device.")

    args = parser.parse_args()

    # --------------------------------- Interface -------------------------------- #

    interface = Interface(args.port)
    interface.open()
    time.sleep(1)
    interface.attach_servos()
    print(f'Connected.')

    # -------------------------------- Controller -------------------------------- #

    # Read the JSON
    with open(args.config) as f:
        config = json.load(f)

    # Create the Hexapod object
    hexapod = Hexapod(config[args.name])
    controller = Controller(hexapod)

    # ------------------------------- Control loop ------------------------------- #

    try:

        t = 0.0
        # dt = 1. / 240.
        dt = args.dt

        # Stand and wait a little
        controller.stand(5)

        # Look around
        controller.set_body_pose(2, body_orientation=np.array([0, np.deg2rad(10), np.deg2rad(10)]))
        controller.set_body_pose(2, body_orientation=np.array([0, np.deg2rad(-10), np.deg2rad(-10)]))
        controller.set_body_pose(2, body_orientation=np.array([0, 0, 0]))

        # Move a leg along its relative axis
        leg_index = 0
        offset = 50
        x, y, z = 80, 0, 0  # Add an offset to the x to fall in the reachable workspace

        # Move by a certain amount in the leg's x-axis
        controller.set_legs_positions(2, np.array([[x, y, z]]), indices=[leg_index], leg_frame=True)
        controller.set_legs_positions(2, np.array([[x + offset, y, z]]), indices=[leg_index], leg_frame=True)
        controller.set_legs_positions(2, np.array([[x, y, z]]), indices=[leg_index], leg_frame=True)

        # Move by a certain amount in the leg's y-axis
        controller.set_legs_positions(2, np.array([[x, y, z]]), indices=[leg_index], leg_frame=True)
        controller.set_legs_positions(2, np.array([[x, y + offset, z]]), indices=[leg_index], leg_frame=True)
        controller.set_legs_positions(2, np.array([[x, y, z]]), indices=[leg_index], leg_frame=True)

        # Move by a certain amount in the leg's z-axis by following a straight line
        delta=10
        x, y = 150, 0
        for _ in range(3):
            for i in range(-50, 50, delta):
                controller.set_legs_positions(0.1, np.array([[x, y, i]]), indices=[leg_index], leg_frame=True)
            for i in range(50, -50, -delta):
                controller.set_legs_positions(0.1, np.array([[x, y, i]]), indices=[leg_index], leg_frame=True)

        # Move the same leg on points expressed in the origin frame, drawing a square
        x, y, z = 150, 150, 100
        offset = 50

        controller.set_legs_positions(2, np.array([[x, y, z]]), indices=[leg_index])
        controller.set_legs_positions(2, np.array([[x + offset, y, z]]), indices=[leg_index])
        controller.set_legs_positions(2, np.array([[x + offset, y + offset, z]]), indices=[leg_index])
        controller.set_legs_positions(2, np.array([[x, y + offset, z]]), indices=[leg_index])
        controller.set_legs_positions(2, np.array([[x, y, z]]), indices=[leg_index])  # Go back to the starting point

        # Sit and terminate the test script
        controller.sit(5)

        while True:

            # Get the joint angles as pulse widths
            joint_pulses = controller.step(dt)

            # Reshape the joint angles into a single 1D array
            joint_pulses = joint_pulses.reshape(-1).tolist()

            # Set them
            interface.set_pulses([i for i in range(18)], joint_pulses)

            # print("\r>> V:{}\tI:{}".format(round(interface.get_voltage(), 2), round(interface.get_current(), 2)), end='')

            time.sleep(dt)
            t += dt

    except KeyboardInterrupt:

        print('Stopped.')

    finally:

        print('Disconnected.')
        interface.close()
