"""
Generates foot trajectories for different gaits:
- Tripod gait (fastest, stable)
- Wave gait (slow, maximum stability)
- Ripple gait (medium speed and stability)

Phase advances proportionally to effective speed (velocity-scaled, fixed stride)..
"""

import numpy as np


class GaitGenerator:
    """Generate leg trajectories for various gaits."""

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
        self.gait_params = config['gaits']
        self.leg_names = [k for k, v in config['kinematics']['legs'].items() if isinstance(v, dict)]

        self.current_gait = 'tripod'
        self.phase = 0.0  # current gait phase [0, 1]

        self.neutral_stance_positions = self._compute_neutral_stance()

        # Phase rate saved from the last non-zero velocity tick.
        # Used to keep phase advancing while a swing is still completing.
        self._last_phase_rate = 1.0 / self.cycle_time
        self._finishing_swing = False  # true while completing a residual swing after v=0

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
    def stride_length(self):
        """Fixed stride length (mm). Phase rate scales so this distance is always covered per step."""
        return self.gait_params[self.current_gait].get('stride_length', 60.0)

    @property
    def cycle_time(self):
        return self.config['gaits'].get('cycle_time', 1.0)

    @property
    def stance_radius(self):
        return self.config['gaits'].get('stance_radius', 150.0)

    # Gait selection

    def set_gait(self, gait_name: str):
        """Set the active gait pattern."""
        if gait_name not in self.gait_params:
            raise ValueError(f"Unknown gait: {gait_name}")
        self.current_gait = gait_name

    def set_gait_parameter(self, param_name: str, value: float):
        """
        Set a gait parameter in real-time.

        Args:
            param_name: Parameter name ('duty_factor', 'step_height', 'stride_length', 'overlap')
            value: New parameter value
        """
        if param_name in self.gait_params[self.current_gait]:
            self.gait_params[self.current_gait][param_name] = value
        else:
            raise ValueError(f"Unknown parameter: {param_name}")

    # Phase helpers

    def get_leg_phase(self, leg_name: str, global_phase: float) -> float:
        """
        Return the phase for a specific leg in the gait cycle.

        Args:
            leg_name: Name of the leg
            global_phase: Global gait phase [0, 1]

        Returns:
            Leg phase [0, 1] where [0, duty_factor) is stance and [duty_factor, 1) is swing
        """
        groups = self.GAIT_GROUPS[self.current_gait]
        num_groups = len(groups)

        leg_group_idx = next(
            (idx for idx, group in enumerate(groups) if leg_name in group), None
        )
        if leg_group_idx is None:
            raise ValueError(f"Leg {leg_name} not found in gait groups")

        phase_offset = leg_group_idx * (1.0 - self.overlap) / num_groups
        return (global_phase + phase_offset) % 1.0

    def is_leg_in_stance(self, leg_name: str, global_phase: float) -> bool:
        """Return True if the leg is in stance (on ground) at the given global phase."""
        return self.get_leg_phase(leg_name, global_phase) < self.duty_factor

    def is_leg_in_swing(self, leg_name: str, global_phase: float) -> bool:
        """Return True if the leg is in swing (in air) at the given global phase."""
        return not self.is_leg_in_stance(leg_name, global_phase)

    # Neutral stance

    def _compute_neutral_stance(self) -> dict:
        """
        Compute neutral foot positions from leg mounting points and stance_radius.

        Returns:
            Dict mapping leg names to [x, y, z] positions in body frame (z = 0)
        """
        legs_config = self.config['kinematics']['legs']
        stance_positions = {}

        for leg_name in self.leg_names:
            leg_config = legs_config[leg_name]
            mount_position = np.array(leg_config['position'])
            mount_yaw = np.radians(leg_config['orientation'][2])

            x = mount_position[0] + self.stance_radius * np.cos(mount_yaw)
            y = mount_position[1] + self.stance_radius * np.sin(mount_yaw)

            stance_positions[leg_name] = np.array([x, y, 0.0])

        return stance_positions

    # Foot trajectory

    def compute_foot_position(self, leg_name: str, global_phase: float,
                              stride_vector: np.ndarray) -> np.ndarray:
        """
        Compute target foot position for a leg at the given phase.

        The stride_vector is the total displacement the foot travels backward relative
        to the body during one full stance phase.

        Args:
            leg_name: Name of the leg
            global_phase: Global gait phase [0, 1]
            stride_vector: Foot displacement vector for one stance phase [dx, dy, 0] (mm)

        Returns:
            Target foot position [x, y, z] in body frame
        """
        leg_phase = self.get_leg_phase(leg_name, global_phase)
        stance_position = self.neutral_stance_positions[leg_name]

        if leg_phase < self.duty_factor:
            # Stance: foot on ground, moving from front to back of stride
            stance_norm = leg_phase / self.duty_factor  # [0, 1] across stance
            stride_progress = 0.5 - stance_norm         # +0.5 at touchdown, -0.5 at liftoff

            position = stance_position + stride_progress * stride_vector
            position[2] = 0.0  # foot stays on ground

        else:
            # Swing: foot in air, arcing from back of stride to front
            swing_norm = (leg_phase - self.duty_factor) / (1.0 - self.duty_factor)  # [0, 1]
            stride_progress = -0.5 + swing_norm  # -0.5 at liftoff, +0.5 at touchdown

            position = stance_position + stride_progress * stride_vector
            position[2] = self.step_height * 4.0 * swing_norm * (1.0 - swing_norm)  # parabolic arc

        return position

    # Update

    def update(self, dt: float, velocity: np.ndarray, angular_velocity: float) -> dict:
        """
        Update gait phase and return target foot positions for all legs.

        Phase rate is proportional to effective speed so that stride_length is always
        covered per step regardless of how fast the robot is commanded to move.
        When velocity is zero and no leg is in swing, the gait freezes. Externally,
        a controller should then hop the legs in place, using the same leg groups.

        Args:
            dt: Time step (seconds)
            velocity: Linear velocity [vx, vy, vz] in body frame (mm/s)
            angular_velocity: Yaw rate (degrees/s)

        Returns:
            Dict mapping leg names to target foot positions [x, y, z] in body frame
        """
        omega_rad = np.radians(angular_velocity)

        # Effective speed: linear speed plus tangential equivalent of yaw rate at stance radius.
        # This governs how fast the phase advances so both contribute equally to stride pacing.
        linear_speed = np.linalg.norm(velocity[:2])
        angular_speed_equiv = abs(omega_rad) * self.stance_radius
        effective_speed = linear_speed + angular_speed_equiv

        any_in_swing = any(self.is_leg_in_swing(leg, self.phase) for leg in self.leg_names)

        if effective_speed > 1e-3:
            # Normal walking: phase advances proportional to speed.
            # phase_rate = effective_speed * duty_factor / stride_length
            # At the reference speed (stride_length / (duty_factor * cycle_time)) this
            # equals 1/cycle_time, which is the natural unscaled rate.
            self._finishing_swing = False
            phase_rate = effective_speed * self.duty_factor / self.stride_length
            self._last_phase_rate = phase_rate
            self.phase = (self.phase + phase_rate * dt) % 1.0

            # Stride vector: total foot displacement during one stance phase.
            # Each leg gets a linear component plus a tangential component from yaw.
            T_stance = self.stride_length / effective_speed
            stride_vectors = self._compute_stride_vectors(velocity, omega_rad, T_stance)

        elif any_in_swing:
            # Velocity dropped to zero but at least one leg is still airborne.
            # Keep advancing phase at the last known rate so the swing completes
            # naturally. A zero stride_vector makes compute_foot_position arc the
            # swing leg back to neutral XY with the standard parabolic height.
            self._finishing_swing = True
            self.phase = (self.phase + self._last_phase_rate * dt) % 1.0
            stride_vectors = {leg: np.zeros(3) for leg in self.leg_names}

        else:
            # Fully stopped and all legs on the ground: freeze phase, return neutral.
            self._finishing_swing = False
            return {leg: self.neutral_stance_positions[leg] for leg in self.leg_names}

        return {
            leg: self.compute_foot_position(leg, self.phase, stride_vectors[leg])
            for leg in self.leg_names
        }

    def _compute_stride_vectors(self, velocity: np.ndarray,
                                omega_rad: float, T_stance: float) -> dict:
        """
        Compute per-leg stride vectors.

        The stride vector is the total distance the foot travels backward relative
        to the body during stance. It combines the robot's linear velocity with the
        tangential velocity each foot experiences due to yaw.

        Args:
            velocity: Linear velocity [vx, vy, vz] in body frame (mm/s)
            omega_rad: Yaw rate in radians/s
            T_stance: Duration of one stance phase (seconds)

        Returns:
            Dict mapping leg names to stride vectors [dx, dy, 0] (mm)
        """
        stride_vectors = {}

        for leg_name in self.leg_names:
            stance_pos = self.neutral_stance_positions[leg_name]

            # Tangential velocity at this foot's position due to body yaw
            r = np.linalg.norm(stance_pos[:2])
            angle = np.arctan2(stance_pos[1], stance_pos[0])
            tangent = np.array([-np.sin(angle), np.cos(angle), 0.0])

            linear_vel_3d = np.array([velocity[0], velocity[1], 0.0])
            foot_velocity = linear_vel_3d + tangent * r * omega_rad

            stride_vectors[leg_name] = foot_velocity * T_stance

        return stride_vectors

    # Queries

    def get_gait_info(self) -> dict:
        """Return a snapshot of current gait state for telemetry."""
        return {
            'name': self.current_gait,
            'phase': self.phase,
            'duty_factor': self.duty_factor,
            'step_height': self.step_height,
            'stride_length': self.stride_length,
            'finishing_swing': self._finishing_swing
        }

    def get_stance_legs(self, global_phase: float = None) -> list:
        """Return list of leg names currently in stance phase."""
        if global_phase is None:
            global_phase = self.phase
        return [leg for leg in self.leg_names if self.is_leg_in_stance(leg, global_phase)]

    def get_swing_legs(self, global_phase: float = None) -> list:
        """Return list of leg names currently in swing phase."""
        if global_phase is None:
            global_phase = self.phase
        return [leg for leg in self.leg_names if self.is_leg_in_swing(leg, global_phase)]

    def is_finishing_swing(self) -> bool:
        """True while completing a residual swing after velocity dropped to zero."""
        return self._finishing_swing

    def set_stance_radius(self, radius: float):
        """
        Update stance radius and recompute neutral positions.

        Args:
            radius: Distance from body center to foot contact (mm)
        """
        self.config['gaits']['stance_radius'] = radius
        self.neutral_stance_positions = self._compute_neutral_stance()

    def reset(self):
        """Reset phase and finishing state. Call before starting a fresh walk sequence."""
        self.phase = 0.0
        self._finishing_swing = False
        self._last_phase_rate = 1.0 / self.cycle_time