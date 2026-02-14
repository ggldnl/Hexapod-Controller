"""
Main controller that manages gait execution and body pose adjustments.
"""

import numpy as np
import logging

from .kinematics import HexapodKinematics
from .gaits import GaitGenerator


class HexapodController:
    """
    Main hexapod controller.
    """

    def __init__(self, interface, config: dict, logfile=None):

        self.interface = interface
        self.config = config

        # Initialize subsystems
        self.kinematics = HexapodKinematics(config)
        self.gait = GaitGenerator(config)

        # Gait velocities
        self.linear_velocity = np.zeros(3)  # [vx, vy, vz] mm/s in body frame
        self.angular_velocity = 0           # wz deg/s in body frame

        # Body pose (applied on top of gait)
        self.body_position = np.zeros(3)  # additional [x, y, z] offset
        self.body_orientation = np.zeros(3)  # [roll, pitch, yaw] in degrees

        # Target body pose
        self.target_body_position = np.zeros(3)
        self.target_body_orientation = np.zeros(3)

        self.body_linear_velocity = config['safety'].get('body_lin_vel_max', 50.0)  # mm/s
        self.body_angular_velocity = config['safety'].get('body_ang_vel_max', 30.0) # deg/s

        # Logging
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')

        if logfile:
            file_handler = logging.FileHandler(logfile)
            file_handler.setFormatter(formatter)
            file_handler.setLevel(logging.INFO)
            self.logger.addHandler(file_handler)

            console_handler = logging.StreamHandler()
            console_handler.setFormatter(formatter)
            console_handler.setLevel(logging.DEBUG)
            self.logger.addHandler(console_handler)

        # Finally, enable the robot
        self.enabled = True
        self.enable()  # also enables the interface

    def enable(self):
        """Enable interface."""
        self.logger.info("Enabling interface.")
        self.interface.enable()
        self.enabled = True

    def disable(self):
        """Disable interface."""
        self.logger.info("Disabling interface.")
        self.interface.disable()
        self.enabled = False

    def update(self, dt: float):
        """
        Update controller state and compute new joint angles.
        This method should be called in the main control loop.

        Args:
            dt: Time step in seconds
        """

        if not self.enabled:
            return False

        # Safety check
        is_safe = self.interface.check()
        if not is_safe:
            self.logger.warning(f"Emergency stop: " +
                                f"voltage={self.interface.get_voltage()}, current={self.interface.get_current()}")
            self.emergency_stop()
            return False

        # Interpolate body pose
        self._interpolate_body_pose(dt)

        # Update gait to get base foot positions
        # The gait generator returns foot positions in the body frame
        # linear velocity: mm/s
        # angular velocity: deg/s
        foot_positions = self.gait.update(dt, self.linear_velocity, self.angular_velocity)
        self.logger.debug(f"""Target foot positions: {','.join([
            f'({foot_positions[leg][0]:4.2f}, {foot_positions[leg][1]:4.2f}, {foot_positions[leg][2]:4.2f})'
                for leg in ['front_right', 'middle_right', 'rear_right', 'rear_left', 'middle_left', 'front_left']
        ])}""")

        # Compute inverse kinematics for all legs
        joint_values = self.kinematics.inverse(
            foot_positions,
            body_position=self.body_position,
            body_orientation=self.body_orientation
        )
        joint_values = {k: tuple(np.degrees(v)) if v else None for k, v in joint_values.items()}
        self.logger.debug(f"""Target joint values  : {','.join([
            f'({joint_values[leg][0]:4.2f}, {joint_values[leg][1]:4.2f}, {joint_values[leg][2]:4.2f})'
                if joint_values[leg] else 'None' for leg in ['front_right', 'middle_right', 'rear_right', 'rear_left', 'middle_left', 'front_left']
        ])}""")

        # Some of the points the planner spit out are unreachable: halt
        if any([elem is None for elem in joint_values.values()]):
            self.logger.warning(f"Legs {[1 if elem is None else 0 for elem in joint_values.values()]} could not reach target.")
            return False

        outcome = self.interface.set_all_legs(joint_values)
        return outcome

    def _interpolate_body_pose(self, dt: float):
        """
        Smoothly interpolate current body pose toward target body pose.

        This prevents abrupt jumps when user changes body height, orientation, etc.
        Uses exponential smoothing with configurable speed limits.

        Args:
            dt: Time step in seconds
        """
        # Interpolate position
        position_error = self.target_body_position - self.body_position
        position_error_magnitude = np.linalg.norm(position_error)

        if position_error_magnitude > 0.001:  # Avoid division by zero
            # Limit maximum change per timestep
            max_position_change = self.body_linear_velocity * dt

            if position_error_magnitude > max_position_change:
                # Move at constant speed toward target
                position_step = (position_error / position_error_magnitude) * max_position_change
            else:
                # Close enough - just reach the target
                position_step = position_error

            self.body_position += position_step

        # Interpolate orientation
        orientation_error = self.target_body_orientation - self.body_orientation

        # Handle angle wrapping for yaw (optional, depends on your needs)
        # For small angles this isn't critical, but for safety:
        orientation_error[2] = np.arctan2(np.sin(orientation_error[2]), np.cos(orientation_error[2]))

        orientation_error_magnitude = np.linalg.norm(orientation_error)

        if orientation_error_magnitude > 0.001:
            # Limit maximum change per timestep
            max_orientation_change = np.radians(self.body_angular_velocity) * dt

            if orientation_error_magnitude > max_orientation_change:
                # Move at constant speed toward target
                orientation_step = (orientation_error / orientation_error_magnitude) * max_orientation_change
            else:
                # Close enough - just reach the target
                orientation_step = orientation_error

            self.body_orientation += orientation_step

    # Gait control

    def set_linear_velocity(self, vx: float, vy: float, vz: float = 0.0):
        """
        Set robot linear velocity.

        Args:
            vx: forward velocity (mm/s, positive = forward)
            vy: sideways velocity (mm/s, positive = right)
            vz: upwards velocity (mm/s, positive = up) - usually 0
        """
        max_linear = self.config['safety']['lin_vel_max']
        self.linear_velocity[0] = np.clip(vx, -max_linear, max_linear)
        self.linear_velocity[1] = np.clip(vy, -max_linear, max_linear)
        self.linear_velocity[2] = np.clip(vz, -max_linear, max_linear)
        self.logger.info(f"Setting gait linear velocity to: "
                         f"{self.linear_velocity[0]}, {self.linear_velocity[1]}, {self.linear_velocity[2]} mm/s")

    def set_angular_velocity(self, wz: float):
        """
        Set robot angular velocity.

        Args:
            wz: yaw rate (deg/s, positive = counter-clockwise)
        """
        max_angular = self.config['safety']['ang_vel_max']
        self.angular_velocity = np.clip(wz, -max_angular, max_angular)
        self.logger.info(f"Setting gait angular velocity to: {self.angular_velocity} deg/s")

    def set_gait(self, gait_name: str):
        """Change gait pattern"""
        self.gait.set_gait(gait_name)

    def set_gait_parameter(self, param_name: str, value: float):
        """Adjust gait parameter in real-time"""
        self.gait.set_gait_parameter(param_name, value)

    # Body pose adjustments

    def stand(self):
        """
        Initialize robot to standing position.
        Resets all body pose adjustments and stops motion.
        """

        # Stop motion
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = 0
        self.body_position = np.zeros(3)
        self.body_orientation = np.zeros(3)

        # Set body height to 2/3 of max height
        min_z, max_z = self.config['safety']['z_range']
        self.body_position[2] = (max_z + min_z) / 3 * 2
        self.logger.info(f"Standing to h={self.body_position[2]} mm")

    def set_body_height(self, height: float):
        """
        Set body height (general action).
        Applied during gait without interruption.
        """
        min_z, max_z = self.config['safety']['z_range']
        height = np.clip(height, min_z, max_z)
        self.target_body_position[2] = height
        self.logger.info(f"Setting body height to h={self.target_body_position[2]} mm")

    def set_body_position(self, dx: float, dy: float, dz: float):
        """
        Set body position offset (general action).
        Small adjustments to body position during gait.

        Args:
            dx: Forward/backward shift (mm, positive = forward)
            dy: Left/right shift (mm, positive = right)
            dz: Up/down shift (mm, positive = up)
        """

        # Limit position offset to prevent unreachable configurations
        x_range = self.config['safety'].get('x_range', (-50, 50))
        y_range = self.config['safety'].get('y_range', (-50, 50))
        z_range = self.config['safety'].get('z_range', (-50, 50))

        clipped_dx = np.clip(dx, x_range[0], x_range[1])
        clipped_dy = np.clip(dy, y_range[0], y_range[1])
        clipped_dz = np.clip(dz, z_range[0], z_range[1])

        self.target_body_position[0] = clipped_dx
        self.target_body_position[1] = clipped_dy
        self.target_body_position[2] = clipped_dz

        self.logger.info(f"Setting body position to: {clipped_dx}, {clipped_dy}, {clipped_dz} mm")

    def set_body_orientation(self, roll: float, pitch: float, yaw: float):
        """
        Set body orientation (general action).
        Small rotations applied during gait.

        Args:
            roll: Body roll angle (degrees, positive = right side down)
            pitch: Body pitch angle (degrees, positive = nose down)
            yaw: Body yaw angle (degrees, positive = counter-clockwise)
        """

        # Limit orientation to prevent unreachable configurations
        roll_range = self.config['safety'].get('roll_range', (-10, 10))
        pitch_range = self.config['safety'].get('pitch_range', (-10, 10))
        yaw_range = self.config['safety'].get('yaw_range', (-10, 10))

        clipped_roll = np.clip(roll, roll_range[0], roll_range[1])
        clipped_pitch = np.clip(pitch, pitch_range[0], pitch_range[1])
        clipped_yaw = np.clip(yaw, yaw_range[0], yaw_range[1])

        self.target_body_orientation[0] = np.radians(clipped_roll)
        self.target_body_orientation[1] = np.radians(clipped_pitch)
        self.target_body_orientation[2] = np.radians(clipped_yaw)

        self.logger.info(f"Setting body orientation to: {clipped_roll}, {clipped_pitch}, {clipped_yaw} deg")

    # Convenience methods

    def emergency_stop(self):
        """Emergency stop - immediately halt all motion"""

        # Stop gait sequence
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = 0

        # Stop body pose interpolation
        self.body_linear_velocity = np.zeros(3)
        self.body_angular_velocity = np.zeros(3)

        # Disable hardware interface
        self.disable()

    def get_status(self) -> dict:
        """Get current robot status"""
        return {
            'enabled': self.enabled,
            'body_position': self.body_position.tolist(),
            'body_orientation': self.body_orientation.tolist(),
            'linear_velocity': self.linear_velocity.tolist(),
            'angular_velocity': self.angular_velocity,
            'gait': {
                'name': self.gait.current_gait,
                'phase': self.gait.phase,
                'stance_legs': self.gait.get_stance_legs()
            },
            'battery_voltage': self.interface.get_voltage(),
            'current_draw': self.interface.get_current()
        }

    def is_body_pose_settled(self, position_threshold: float = 1.0,
                             orientation_threshold: float = 1.0) -> bool:
        """
        Check if body pose has reached target (finished interpolating).

        Args:
            position_threshold: Position error threshold (mm)
            orientation_threshold: Orientation error threshold (degrees)
        """
        position_error = np.linalg.norm(self.target_body_position - self.body_position)
        orientation_error = np.linalg.norm(self.target_body_orientation - self.body_orientation)
        orientation_error_deg = np.degrees(orientation_error)

        return (position_error < position_threshold and
                orientation_error_deg < orientation_threshold)