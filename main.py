"""
Simple Gait Viewer

Quick visualization of a single gait pattern.
"""

import yaml
import time
import argparse

from controller.hardware.kernel import Kernel
from controller.model.interface import Interface
from controller.model.controller import HexapodController


if __name__ == '__main__':

    # TODO work in progress

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
    controller = HexapodController(interface, config)
    controller.set_gait(args.gait)

    hz = config['control']['update_rate']
    dt = 1.0 / hz
    t = 0.0

    # Examples
    t0 = 5
    t1 = 10
    t2 = 15
    t3 = 20

    while True:

        # Controller computes and sets joint angles
        outcome = controller.update(dt)

        # Example: at t0, gait linear velocity changes
        if t0 and t >= t0:
            t0 = None  # invalidate so that we won't change it again
            controller.set_linear_velocity(args.vx * 10, args.vy, args.vz)

        # Example: at t1, body position changes
        if t1 and t >= t1:
            t1 = None
            controller.set_body_position(0, 0, 30)

        # Example: at t2, body orientation changes but the gait generator still moves the robot forward in a straight line
        if t2 and t >= t2:
            t2 = None
            controller.set_body_orientation(0, 0, 30)

        # Example: at t3, gait orientation changes and the robot curves
        if t3 and t >= t3:
            t3 = None
            controller.set_angular_velocity(0)

        time.sleep(dt)
        t += dt
