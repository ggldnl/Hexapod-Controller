import numpy as np
import argparse
import time
import json

from controller import Controller
from interface import Interface
from hexapod import Hexapod


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Code to be run on the Hexapod")
    parser.add_argument("-g", "--gait", type=str, default='tri', help="Gait (tri/wave/ripple)")
    parser.add_argument("-c", "--config", type=str, default='controller/config.json', help="Path to the robot's configuration file")
    parser.add_argument('-n', '--name', type=str, default='hexapod', help="Name of the robot in the config")
    parser.add_argument('-d', '--dt', type=float, default=0.02, help="Time delta for update (default=0.02=50Hz)")

    args = parser.parse_args()

    # --------------------------------- Interface -------------------------------- #

    interface = Interface()
    interface.open()
    time.sleep(1)
    interface.attach_servos()

    # -------------------------------- Controller -------------------------------- #

    # Read the JSON
    with open(args.config) as f:
        config = json.load(f)

    # Create a Hexapod object
    hexapod = Hexapod(config[args.name])
    controller = Controller(hexapod)

    # ------------------------------- Control loop ------------------------------- #

    try:

        interface.open()

        t = 0.0
        # dt = 1. / 240.
        dt = args.dt

        print(f'Standing...')
        controller.wait(2)
        controller.stand(2)
        controller.set_body_pose(
            [0, 0, 0],
            [np.deg2rad(10), np.deg2rad(10), np.deg2rad(10)],
            2
        )
        controller.set_body_pose(
            [0, 0, 0],
            [np.deg2rad(-10), np.deg2rad(-10), np.deg2rad(-10)],
            2
        )
        controller.set_body_pose(
            [0, 0, 0],
            [np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)],
            1
        )

        while True:

            # Get the joint angles
            joint_angles = controller.step(dt)

            # Reshape the joint angles into a single 1D array
            joint_angles = joint_angles.reshape(-1).tolist()

            # Set them
            interface.set_angles([i for i in range(18)], joint_angles)

            time.sleep(dt)
            t += dt

    finally:

        print('Disconnected.')

        interface.detach_servos()
        time.sleep(1)

        interface.close()
