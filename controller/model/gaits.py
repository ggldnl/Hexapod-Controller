"""
Generates foot trajectories for different gaits:
- Tripod gait (fastest, stable)
- Wave gait (slow, maximum stability)
- Ripple gait (medium speed and stability)
"""

import numpy as np
from typing import Dict, List


class GaitGenerator:
    """Generate leg trajectories for various gaits"""

    # Gait phase groups - which legs move together
    GAIT_GROUPS = {
        'tripod': [
            ['front_right', 'middle_left', 'rear_right'],
            ['front_left', 'middle_right', 'rear_left']
        ],
        'wave': [
            ['front_right'],
            ['middle_right'],
            ['rear_right'],
            ['rear_left'],
            ['middle_left'],
            ['front_left']
        ],
        'ripple': [
            ['front_right', 'rear_left'],
            ['middle_right', 'front_left'],
            ['rear_right', 'middle_left']
        ]
    }

    def __init__(self, config: dict):
        """
        Args:
            config: Configuration dictionary
        """
        self.config = config

        self.gait_params = self.config['gaits']
        self.leg_names = list(self.config['kinematics']['legs'].keys() - {'coxa', 'femur', 'tibia'})

        # Current gait state
        self.current_gait = 'tripod'
        self.phase = 0.0  # Current phase in gait cycle [0, 1]

        # Compute neutral stance positions from leg geometry
        self.neutral_stance_positions = self._compute_neutral_stance()

    def _compute_neutral_stance(self) -> Dict[str, np.ndarray]:
        """
        Compute neutral stance positions for all legs based on leg mounting points.

        The neutral stance is where feet touch the ground when the robot is standing
        in its default pose. These are computed from:
        1. Leg mounting positions (from config)
        2. Desired stance width/spread
        3. Ground contact assumption (z = 0 in world frame, or relative to body)

        Returns:
            Dictionary mapping leg names to neutral foot positions [x, y, z] in body frame
        """
        legs_config = self.config['kinematics']['legs']
        stance_positions = {}

        # Get stance parameters
        stance_radius = self.config['safety'].get('stance_radius', 150.0)  # mm from body center

        for leg_name in self.leg_names:
            leg_config = legs_config[leg_name]
            mount_position = np.array(leg_config['position'])

            # Compute neutral stance position
            # The foot should be at some distance from the body center,
            # roughly aligned with the leg mounting angle
            mount_angle = np.arctan2(mount_position[1], mount_position[0])

            # Place foot at stance_radius from body center, at ground level
            x = stance_radius * np.cos(mount_angle)
            y = stance_radius * np.sin(mount_angle)
            z = 0.0  # Ground level in body frame (body frame z=0 is at the ground)

            stance_positions[leg_name] = np.array([x, y, z])

        return stance_positions

    def set_gait(self, gait_name: str):
        """Set the active gait pattern"""
        if gait_name not in self.gait_params:
            raise ValueError(f"Unknown gait: {gait_name}")
        self.current_gait = gait_name

    def get_leg_phase(self, leg_name: str, global_phase: float) -> float:
        """
        Get the phase for a specific leg in the gait cycle

        Args:
            leg_name: Name of the leg
            global_phase: Global gait phase [0, 1]

        Returns:
            Leg phase [0, 1] where 0-duty_factor is stance, duty_factor-1 is swing
        """
        groups = self.GAIT_GROUPS[self.current_gait]

        # Find which group this leg belongs to
        leg_group_idx = None
        for idx, group in enumerate(groups):
            if leg_name in group:
                leg_group_idx = idx
                break

        if leg_group_idx is None:
            raise ValueError(f"Leg {leg_name} not found in gait groups")

        # Calculate phase offset for this group
        num_groups = len(groups)
        phase_offset = leg_group_idx / num_groups

        # Calculate leg phase with offset
        leg_phase = (global_phase + phase_offset) % 1.0

        return leg_phase

    def is_leg_in_stance(self, leg_name: str, global_phase: float) -> bool:
        """
        Check if leg is in stance phase (on ground)

        Args:
            leg_name: Name of the leg
            global_phase: Global gait phase [0, 1]

        Returns:
            True if leg is in stance phase
        """
        leg_phase = self.get_leg_phase(leg_name, global_phase)
        duty_factor = self.gait_params[self.current_gait]['duty_factor']

        return leg_phase < duty_factor

    def calculate_foot_position(self, leg_name: str, global_phase: float,
                                stride_vector: np.ndarray) -> np.ndarray:
        """
        Calculate target foot position for a leg at given phase

        Args:
            leg_name: Name of the leg
            global_phase: Global gait phase [0, 1]
            stride_vector: Movement vector for this step [dx, dy, 0]

        Returns:
            Target foot position [x, y, z] in body frame
        """
        leg_phase = self.get_leg_phase(leg_name, global_phase)
        duty_factor = self.gait_params[self.current_gait]['duty_factor']
        step_height = self.gait_params[self.current_gait]['step_height']

        # Get neutral stance position for this leg
        stance_position = self.neutral_stance_positions[leg_name].copy()

        if leg_phase < duty_factor:
            # STANCE PHASE - leg is on ground, moving backward relative to body
            # Normalized stance phase [0, 1]
            stance_norm = leg_phase / duty_factor

            # Move from front of stride to back
            # At phase 0: foot is at front of stride (+0.5 * stride_vector)
            # At phase duty_factor: foot is at back of stride (-0.5 * stride_vector)
            stride_progress = 0.5 - stance_norm

            position = stance_position + stride_progress * stride_vector
            position[2] = 0.0  # Keep foot on ground (z=0 in body frame)

        else:
            # SWING PHASE - leg is in air, moving forward relative to body
            # Normalized swing phase [0, 1]
            swing_norm = (leg_phase - duty_factor) / (1.0 - duty_factor)

            # Move from back of stride to front
            stride_progress = -0.5 + swing_norm

            position = stance_position + stride_progress * stride_vector

            # Add vertical component (parabolic trajectory)
            # Maximum height at mid-swing
            height_progress = 4 * swing_norm * (1 - swing_norm)  # Parabola peaking at 0.5
            position[2] = step_height * height_progress

        return position

    def update(self, dt: float, velocity: np.ndarray, angular_velocity: float) -> Dict[str, np.ndarray]:
        """
        Update gait and calculate target positions for all legs

        Args:
            dt: Time step (seconds)
            velocity: Linear velocity [vx, vy, vz] in body frame (mm/s)
            angular_velocity: Angular velocity around z-axis (degrees/s)

        Returns:
            Dictionary mapping leg names to target positions [x, y, z] in body frame
            where z=0 represents ground level
        """
        cycle_time = self.config['control']['cycle_time']

        # Update global phase
        self.phase += dt / cycle_time
        self.phase %= 1.0

        # Calculate stride vector based on velocity and cycle time
        # Only use x and y components for horizontal movement
        stride_length = np.linalg.norm(velocity[:2]) * cycle_time

        if stride_length > 0.001:  # Avoid division by zero
            stride_direction = velocity[:2] / np.linalg.norm(velocity[:2])
        else:
            stride_direction = np.array([0.0, 0.0])

        # Calculate target positions for all legs
        target_positions = {}

        for leg_name in self.leg_names:
            # Get neutral stance for this leg
            stance_pos = self.neutral_stance_positions[leg_name]

            # Calculate stride vector for this leg
            # Start with linear motion stride
            leg_stride_vector = np.array([
                stride_direction[0] * stride_length,
                stride_direction[1] * stride_length,
                0.0
            ])

            # Add rotational component if turning
            if abs(angular_velocity) > 0.1:
                # Legs farther from center move more
                radius = np.linalg.norm(stance_pos[:2])
                angular_stride = np.radians(angular_velocity) * cycle_time

                # Tangential velocity direction (perpendicular to radius)
                angle = np.arctan2(stance_pos[1], stance_pos[0])
                tangent_direction = np.array([
                    -np.sin(angle),  # Perpendicular to radius
                    np.cos(angle)
                ])

                rotational_stride = tangent_direction * radius * angular_stride
                leg_stride_vector[:2] += rotational_stride

            # Calculate foot position
            target_pos = self.calculate_foot_position(
                leg_name, self.phase, leg_stride_vector
            )

            target_positions[leg_name] = target_pos

        return target_positions

    def get_gait_info(self) -> dict:
        """Get information about current gait"""
        return {
            'name': self.current_gait,
            'phase': self.phase,
            'duty_factor': self.gait_params[self.current_gait]['duty_factor'],
            'step_height': self.gait_params[self.current_gait]['step_height']
        }

    def set_gait_parameter(self, param_name: str, value: float):
        """
        Set a gait parameter in real-time

        Args:
            param_name: Parameter name ('duty_factor', 'step_height', 'overlap')
            value: New parameter value
        """
        if param_name in self.gait_params[self.current_gait]:
            self.gait_params[self.current_gait][param_name] = value
        else:
            raise ValueError(f"Unknown parameter: {param_name}")

    def get_stance_legs(self, global_phase: float = None) -> List[str]:
        """
        Get list of legs currently in stance phase

        Args:
            global_phase: Phase to check (uses current phase if None)

        Returns:
            List of leg names in stance
        """
        if global_phase is None:
            global_phase = self.phase

        stance_legs = []
        for leg_name in self.leg_names:
            if self.is_leg_in_stance(leg_name, global_phase):
                stance_legs.append(leg_name)

        return stance_legs

    def set_stance_radius(self, radius: float):
        """
        Update the stance radius and recalculate neutral positions

        Args:
            radius: Distance from body center to foot contact (mm)
        """
        self.config['control']['stance_radius'] = radius  # update local copy
        self.neutral_stance_positions = self._compute_neutral_stance()