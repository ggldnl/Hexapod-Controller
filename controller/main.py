import numpy as np
import argparse
import time
import json

from controller import Controller
from interface import Interface
from hexapod import Robot


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Hexapod controller.")
    parser.add_argument("-c", "--config", type=str, default='config/config.json', help="Path to the robot's configuration file")
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

    # Create a Robot object
    hexapod = Robot(config[args.name])
    controller = Controller(hexapod)

    # ------------------------------- Control loop ------------------------------- #

    try:

        interface.open()
        print(f'Connected.')

        t = 0.0
        # dt = 1. / 240.
        dt = args.dt

        # Stand and wait
        controller.stand(2, y_offset=160)
        controller.wait(1)

        # Reach a configuration
        controller.reach(
            1,
            body_orientation=np.array([0, 0, np.deg2rad(10)])
        )
        controller.reach(
            1,
            body_orientation=np.array([0, 0, -np.deg2rad(10)])
        )
        print(f'Action sequence computed.')

        while True:

            # Get the joint angles as pulse widths
            joint_pulses = controller.step(dt)

            # Reshape the joint angles into a single 1D array
            joint_pulses = joint_pulses.reshape(-1).tolist()

            # Set them
            # TODO fix this
            # interface.set_pulses([i for i in range(18)], joint_pulses)
            interface.set_angles([i for i in range(18)], joint_pulses)

            time.sleep(dt)
            t += dt

    except KeyboardInterrupt:

        print('Stopped.')

    finally:

        print('Disconnected.')
        interface.close()
