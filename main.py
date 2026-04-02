"""
Execute a predefined command sequence on the real robot.
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
                        help='Forward velocity (mm/s).')
    parser.add_argument('--vy', '-y', type=float, default=0.0,
                        help='Strafe velocity (mm/s).')
    parser.add_argument('--vw', '-w', type=float, default=0.0,
                        help='Yaw velocity (deg/s).')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Whether or not to use the logger (increased overhead).')
    parser.add_argument('--log-file', '-l', default=None,
                        help='Log file. The robot must be verbose to actually use the file.')
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
    port = config['serial'].get('port', args.port)
    baud = config['serial'].get('baud', args.baud)
    kernel = Kernel(port=port, baud=baud)               # Hardware communication
    interface = Interface(kernel, config)               # Interface for servo mapping, limits, ...

    # Create controller
    controller = HexapodController(interface, config, verbose=args.verbose, logfile=args.log_file)
    controller.set_gait(args.gait)

    try:

        controller_rate = config['rate'].get('controller_update_rate', 20)
        controller_dt = 1. / controller_rate
        controller_time_accumulator = 0.0
        t = 0.0

        t0 = 5.0    # look in one direction
        t1 = 6.0    # look another direction
        t2 = 7.0    # reset body orientation
        t3 = 8.0    # set linear velocity
        t4 = 12.0   # set angular velocity
        t5 = 18.0   # stop
        t6 = 22.0   # shutdown
        t7 = 25.0   # exit loop

        last_frame = time.perf_counter()
        while True:

            now = time.perf_counter()
            actual_dt = now - last_frame
            last_frame = now
            t += actual_dt

            # Step the controller
            outcome = controller.update(actual_dt)

            # Example: at t0, look in one direction (change body orientation)
            if t0 and t >= t0:
                t0 = None  # invalidate so that we won't change it again
                controller.set_body_orientation(5, -5, 5)

            # Example: at t1, look to another direction (change again body orientation)
            if t1 and t >= t1:
                t1 = None
                controller.set_body_orientation(-5, -5, -5)

            # Example: at t2, reset body orientation
            if t2 and t >= t2:
                t2 = None
                controller.set_body_orientation(0, 0, 0)

            # Example: at t3, start walking
            if t3 and t >= t3:
                t3 = None
                controller.set_linear_velocity(args.vx, args.vy, 0)

            # Example: at t4, change angular velocity
            if t4 and t >= t4:
                t4 = None
                controller.set_angular_velocity(args.vw)

            # Example: at t5, stop walking
            if t5 and t >= t5:
                t5 = None
                controller.set_linear_velocity(0, 0, 0)
                controller.set_angular_velocity(0)

            if t6 and t >= t6:
                t6 = None
                controller.shutdown()

            if t7 and t >= t7:
                t7 = None
                break

            # Warn if over budget
            elapsed = time.perf_counter() - now
            if elapsed > controller_dt:
                print(f"Frame over budget: {elapsed * 1000:.1f}ms > {controller_dt * 1000:.1f}ms")

            remaining = controller_dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:

        controller.emergency_stop()
