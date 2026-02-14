"""
Control the real robot while streaming its state to a websocket.
"""

import yaml
import time
import argparse
from datetime import datetime

from controller.hardware.kernel import Kernel
from controller.model.interface import Interface
from controller.model.controller import HexapodController


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Hexapod control')
    parser.add_argument('--gait', '-g', type=str, default='ripple',
                        choices=['tripod', 'wave', 'ripple'],
                        help='Initial gait pattern. Default is ripple.')
    parser.add_argument('--port', '-p', type=str, default='/dev/ttyAMA0',
                        help='Serial port for Servo2040 communication. Default is /dev/ttyAMA0.')
    parser.add_argument('--baud', '-b', type=int, default='115200',
                        help='Baud rate for serial communication. Default is 115200.')

    args = parser.parse_args()

    # Load configuration
    config_path = 'controller/config/config.yml'
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # Create kernel
    kernel = Kernel(port=args.port, baud=args.baud)     # Hardware communication
    interface = Interface(kernel, config)               # Interface for servo mapping, limits, ...

    # Create controller
    log_file = f'logs/{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.log'
    controller = HexapodController(interface, config)
    controller.set_gait(args.gait)

    hz = config['control']['update_rate']
    dt = 1.0 / hz
    t = 0.0

    while True:

        # TODO read from joystick/keyboard
        linear_velocity = (0, 0, 0)
        angular_velocity = 0
        body_position = (0, 0, 0)
        body_orientation = (0, 0, 0)

        controller.set_linear_velocity(*linear_velocity)
        controller.set_angular_velocity(angular_velocity)
        controller.set_body_position(*body_position)
        controller.set_body_orientation(*body_orientation)

        # Controller computes and sets joint angles
        outcome = controller.update(dt)

        # TODO stream state through websocket
        # state = controller.get_state()

        time.sleep(dt)
        t += dt
