"""
Generates foot trajectories for different gaits:
- Tripod gait (fastest, stable)
- Wave gait (slow, maximum stability)
- Ripple gait (medium speed and stability)
"""

import numpy as np
from enum import Enum


class GaitState(Enum):
    """
    State machine for gait control.

    WALK:       Normal gait — legs alternate between stance (foot on ground, moving
                backward relative to body) and swing (foot in air, moving forward).

    STOPPING:   Normal stance/swing continues until a swing -> stance crossing
                is detected, then transitions to SETTLING.

    SETTLING:   One hold/swing cycle. On the next swing -> stance crossing all legs
                have landed on neutral stance and the state transitions to STOP.

    STOP:       Robot is stationary. All legs held at neutral stance until walk()
                is called.
    """
    WALK = 0
    STOPPING = 1
    SETTLING = 2
    STOP = 3


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
        self.leg_names = [k for k, v in config['kinematics']['legs'].items() if isinstance(v, dict)]

        # Current gait state
        self.current_gait = 'tripod'
        self.phase = 0.0  # current phase in gait cycle [0, 1]

        # Compute neutral stance positions from leg geometry
        self.neutral_stance_positions = self._compute_neutral_stance()

        self.state = GaitState.WALK

        self._settling_start_positions: dict = {}
        self._settling_entry_phase = None

    # Properties

    @property
    def overlap(self):
        return self.gait_params[self.current_gait].get('overlap', 0.0)

    @property
    def duty_factor(self):
        return self.gait_params[self.current_gait].get('duty_factor', 0.5)

    @property
    def step_height(self):
        return self.gait_params[self.current_gait].get('step_height', 30.0)

    @property
    def cycle_time(self):
        return self.config['control'].get('cycle_time', 1.0)

    @property
    def stance_radius(self):
        return self.config['safety'].get('stance_radius', 150.0)  # mm from body center

    # State control

    def stop(self):
        """
        Signal that the robot should come to a halt.

        The current gait cycle completes normally, then on the next full cycle
        each leg group holds its lift-off point and swings to neutral stance.
        """
        if self.state == GaitState.WALK:
            self.state = GaitState.STOPPING

    def walk(self):
        """
        Resume normal walking.
        """
        if self.state == GaitState.STOP:
            self.state = GaitState.WALK

    # Phase

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
        # phase_offset = leg_group_idx / num_groups
        phase_offset = leg_group_idx * (1.0 - self.overlap) / num_groups

        # Calculate leg phase with offset
        leg_phase = (global_phase + phase_offset) % 1.0

        return leg_phase

    def get_leg_phase_at_zero(self, leg_name: str) -> float:
        """
        Return the global phase value that places leg_name at leg_phase=0.

        Args:
            leg_name: Name of the leg
        """

        groups = self.GAIT_GROUPS[self.current_gait]
        leg_group_idx = next(idx for idx, group in enumerate(groups) if leg_name in group)
        phase_offset = leg_group_idx / len(groups)
        return (0.0 - phase_offset) % 1.0

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
        duty_factor = self.duty_factor

        return leg_phase < duty_factor

    def is_leg_in_swing(self, leg_name: str, global_phase: float) -> bool:
        """
        Check if leg is in swing phase (in air)

        Args:
            leg_name: Name of the leg
            global_phase: Global gait phase [0, 1]

        Returns:
            True if leg is in swing phase
        """
        return not self.is_leg_in_stance(leg_name, global_phase)

    def any_leg_swing_to_stance(self, prev_phase: float, curr_phase: float) -> bool:
        """True if any leg transitions from swing to stance this tick."""
        return any(
            self.is_leg_in_swing(leg, prev_phase) and self.is_leg_in_stance(leg, curr_phase)
            for leg in self.leg_names
        )

    def has_cycle_elapsed(self, leg_name: str = None, phase: float = None):
        """True if the current cycle is very close to ending."""

        if not leg_name:
            leg_name = self.leg_names[0]

        if not phase:
            phase = self.phase

        leg_phase = self.get_leg_phase(leg_name, phase)
        return (1.0 - leg_phase) <= 1e-3

    # Foot positioning

    def _compute_neutral_stance(self) -> dict:
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
        for leg_name in self.leg_names:
            leg_config = legs_config[leg_name]
            mount_position = np.array(leg_config['position'])

            # Compute neutral stance position
            # The foot should be at some distance from the body center,
            # roughly aligned with the leg mounting angle
            mount_yaw = np.radians(leg_config['orientation'][2])

            # Place foot at stance_radius from body center, at ground level
            x = mount_position[0] + self.stance_radius * np.cos(mount_yaw)
            y = mount_position[1] + self.stance_radius * np.sin(mount_yaw)
            z = 0.0

            stance_positions[leg_name] = np.array([x, y, z])

        return stance_positions

    def get_touchdown_position(self, leg_name: str, stride_vector: np.ndarray) -> np.ndarray:
        """
        Return the foot position at touchdown — where the leg lands at the start
        of stance (leg_phase = 0, front of stride).
        """
        global_phase = self.get_leg_phase_at_zero(leg_name)
        return self.compute_foot_position(leg_name, global_phase, stride_vector)

    def get_liftoff_position(self, leg_name: str, stride_vector: np.ndarray) -> np.ndarray:
        """
        Return the foot position at liftoff — where the leg leaves the ground at
        the end of stance (leg_phase = duty_factor, back of stride).
        """
        global_phase = (self.get_leg_phase_at_zero(leg_name) + self.duty_factor) % 1.0
        return self.compute_foot_position(leg_name, global_phase, stride_vector)

    def compute_foot_position(self, leg_name: str, global_phase: float,
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
        duty_factor = self.duty_factor
        step_height = self.step_height

        # Get neutral stance position for this leg
        stance_position = self.neutral_stance_positions[leg_name]

        if leg_phase < duty_factor:
            # Stance phase - leg is on ground, moving backward relative to body
            # Normalized stance phase [0, 1]
            stance_norm = leg_phase / duty_factor

            # Move from front of stride to back
            # At phase 0: foot is at front of stride (+0.5 * stride_vector)
            # At phase duty_factor: foot is at back of stride (-0.5 * stride_vector)
            stride_progress = 0.5 - stance_norm

            position = stance_position + stride_progress * stride_vector
            position[2] = 0.0  # keep foot on ground (z=0 in body frame)

        else:
            # Swing phase - leg is in air, moving forward relative to body
            # Normalized swing phase [0, 1]
            swing_norm = (leg_phase - duty_factor) / (1.0 - duty_factor)

            # Move from back of stride to front
            stride_progress = -0.5 + swing_norm

            position = stance_position + stride_progress * stride_vector

            # Add vertical component (parabolic trajectory)
            # Maximum height at mid-swing
            height_progress = 4 * swing_norm * (1 - swing_norm)  # parabola peaking at 0.5
            position[2] = step_height * height_progress

        return position

    def compute_settling_foot_position(self, leg_name: str) -> np.ndarray:
        """
        Settling trajectory: each leg group swings to neutral in sequence.

        The settling cycle [0, 1] is divided into equal slots, one per group.
        - Before the group's slot:  hold the liftoff (start) position
        - During the group's slot:  swing from start to neutral with a parabolic arc
        - After the group's slot:   hold neutral
        """
        groups = self.GAIT_GROUPS[self.current_gait]
        num_groups = len(groups)

        leg_group_idx = next(
            idx for idx, group in enumerate(groups) if leg_name in group
        )

        slot_start = leg_group_idx / num_groups
        slot_end = (leg_group_idx + 1) / num_groups

        settling_phase = self._settling_entry_phase
        neutral = self.neutral_stance_positions[leg_name]
        start = self._settling_start_positions[leg_name]

        if settling_phase < slot_start:
            return start

        elif settling_phase < slot_end:
            swing_norm = (settling_phase - slot_start) / (slot_end - slot_start)
            position = start + swing_norm * (neutral - start)
            position[2] = self.step_height * 4 * swing_norm * (1 - swing_norm)
            return position

        else:
            return neutral

    # Update

    def update(self, dt: float, velocity: np.ndarray, angular_velocity: float) -> dict:
        """
        Update gait phase and return target foot positions for all legs.

        Args:
            dt: Time step (seconds)
            velocity: Linear velocity [vx, vy, vz] in body frame (mm/s)
            angular_velocity: Angular velocity around z-axis (degrees/s)

        Returns:
            Dictionary mapping leg names to target positions [x, y, z] in body frame
        """

        # Phase advance
        prev_phase = self.phase
        self.phase = (self.phase + dt / self.cycle_time) % 1.0

        # Calculate stride vector based on velocity and cycle time
        velocity_norm = np.linalg.norm(velocity[:2])
        stride_length = velocity_norm * self.cycle_time
        stride_direction = velocity[:2] / velocity_norm if stride_length > 1e-3 else np.array([0.0, 0.0])

        # Per-leg stride vector with linear and rotational components
        stride_vectors = {}
        for leg_name in self.leg_names:
            stance_pos = self.neutral_stance_positions[leg_name]
            leg_stride_vector = np.array([
                stride_direction[0] * stride_length,
                stride_direction[1] * stride_length,
                0.0
            ])

            if abs(angular_velocity) > 0.1:
                radius = np.linalg.norm(stance_pos[:2])
                angular_stride = np.radians(angular_velocity) * self.cycle_time
                angle = np.arctan2(stance_pos[1], stance_pos[0])
                tangent_direction = np.array([-np.sin(angle), np.cos(angle)])
                leg_stride_vector[:2] += tangent_direction * radius * angular_stride

            stride_vectors[leg_name] = leg_stride_vector

        # State transitions
        curr_state = self.state

        # phase_changed = (prev_phase < self.duty_factor <= self.phase) or (prev_phase > self.phase)
        if curr_state is GaitState.STOPPING:

            phase_wrapped = prev_phase > self.phase
            if phase_wrapped:
                self._settling_entry_phase = 0
                for leg_name in self.leg_names:
                    self._settling_start_positions[leg_name] = self.compute_foot_position(
                        leg_name, self.phase, stride_vectors[leg_name]
                    )
                self.state = GaitState.SETTLING

        elif curr_state is GaitState.SETTLING:

            increment = dt / self.cycle_time
            prev_settling_phase = self._settling_entry_phase
            self._settling_entry_phase = (self._settling_entry_phase + increment) % 1.0
            settling_phase_wrapped = (prev_settling_phase + increment) >= 1.0
            if settling_phase_wrapped:
                self.state = GaitState.STOP

        # No transitions needed for WALK and STOP states

        # State machine
        target_positions = {}
        for leg_name in self.leg_names:

            """
            if leg_name != 'middle_right':
                target_positions[leg_name] = self.neutral_stance_positions[leg_name]
                continue
            """

            # WALK and STOPPING: normal stance/swing trajectory
            if curr_state is GaitState.WALK or curr_state is GaitState.STOPPING:
                target_positions[leg_name] = self.compute_foot_position(
                    leg_name, self.phase, stride_vectors[leg_name]
                )

            # Hold-swing trajectory
            elif curr_state is GaitState.SETTLING:
                target_positions[leg_name] = self.compute_settling_foot_position(leg_name)

            # Foots are at neutral stance position
            elif curr_state is GaitState.STOP:
                target_positions[leg_name] = self.neutral_stance_positions[leg_name]

        return target_positions

    def get_gait_info(self) -> dict:
        """Get information about current gait"""
        return {
            'name': self.current_gait,
            'phase': self.phase,
            'duty_factor': self.duty_factor,
            'step_height': self.step_height,
            'state': self.state.name
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

    def get_stance_legs(self, global_phase: float = None) -> list:
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

    def get_swing_legs(self, global_phase: float = None) -> list:
        """
        Get list of legs currently in swing phase

        Args:
            global_phase: Phase to check (uses current phase if None)

        Returns:
            List of leg names in swing
        """
        if global_phase is None:
            global_phase = self.phase

        swing_legs = []
        for leg_name in self.leg_names:
            if self.is_leg_in_swing(leg_name, global_phase):
                swing_legs.append(leg_name)

        return swing_legs

    def set_stance_radius(self, radius: float):
        """
        Update the stance radius and recalculate neutral positions

        Args:
            radius: Distance from body center to foot contact (mm)
        """
        self.config['safety']['stance_radius'] = radius  # update local copy
        self.neutral_stance_positions = self._compute_neutral_stance()

    def reset(self):
        """Resets current phase to 0."""
        self.phase = 0.0

        self.state = GaitState.WALK

        self._settling_entry_phase = None
        self._settling_start_positions = {}
