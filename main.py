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
    parser.add_argument('--vx', '-x', type=float, default=50.0,
                        help='Forward velocity (mm/s)')
    parser.add_argument('--vy', '-y', type=float, default=0.0,
                        help='Strafe velocity (mm/s)')
    parser.add_argument('--vz', '-z', type=float, default=0.0,
                        help='Upward velocity (mm/s)')
    parser.add_argument('--controller-rate', '-c', type=float, default=20,
                        help="Controller update rate in Hz. Default is 20 Hz")
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

    try:

        controller_dt = 1. / args.controller_rate
        controller_time_accumulator = 0.0
        t = 0.0

        t0 = 10.0    # start moving

        last_frame = time.perf_counter()
        while True:

            now = time.perf_counter()
            dt = now - last_frame
            last_frame = now
            
            controller_time_accumulator += dt

            if t0 and t >= t0:
                t0 = None  # invalidate so that we won't change it again
                controller.set_linear_velocity(args.vx, args.vy, args.vz)

            # Only update controller when enough time has passed
            if controller_time_accumulator >= controller_dt:
                outcome = controller.update(controller_dt)
                controller_time_accumulator -= controller_dt  # Keep remainder for accuracy

                # TODO stream state through websocket
                # state = controller.get_state()

            t += controller_dt

            # Sleep only what's left of the frame budget
            elapsed = time.perf_counter() - now
            remaining = controller_dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:

        controller.set_linear_velocity(0, 0, 0)
        controller.set_angular_velocity(0)