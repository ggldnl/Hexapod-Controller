"""
Generates foot trajectories for different gaits:
- Tripod gait (fastest, stable)
- Wave gait (slow, maximum stability)
- Ripple gait (medium speed and stability)

Speed model
-----------
The gait runs at a *constant cadence* (one cycle every ``cycle_time`` seconds).
Speed is produced by scaling the *step size* with the commanded velocity, not by
speeding the cadence up:

    step size (per leg) = foot_velocity * stance_duration

where ``foot_velocity`` is that leg's required ground speed (linear velocity plus
the tangential contribution of yaw) and ``stance_duration = duty_factor * cycle_time``.
Bigger command -> bigger steps at the same rhythm. When the largest required step
would exceed the reachable ``max_stride``, the whole commanded twist (linear and
angular together) is scaled down so the radius is preserved — this caps top speed
cleanly instead of distorting the gait.

This is the standard "fixed frequency, velocity-scaled stride" approach. It keeps
turning smooth (cadence never spikes, so no fast tiny steps when turning) and makes
realized velocity match the command up to a well-defined limit of
``max_stride / stance_duration``.
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

        # Average distance from body centre to each foot contact point.
        # Used as the representative moment arm when turning.
        self._body_radius = float(np.mean([
            np.linalg.norm(pos[:2])
            for pos in self.neutral_stance_positions.values()
        ]))

        self._finishing_swing = False  # true while completing a residual swing after v=0

        # Achieved (post-clamp) twist from the last update, for odometry.
        self._achieved_velocity = np.zeros(3)        # [vx, vy, vz] mm/s
        self._achieved_angular_velocity = 0.0        # deg/s

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
    def max_stride(self):
        """Maximum step size (mm). Steps grow with speed up to this reachable limit."""
        return self.gait_params[self.current_gait].get('max_stride', 100.0)

    @property
    def cycle_time(self):
        return self.config['gaits'].get('cycle_time', 1.0)

    @property
    def cadence(self) -> float:
        """Gait cadence (Hz) — cycles per second. Constant by design."""
        return 1.0 / self.cycle_time

    @property
    def stance_radius(self):
        return self.config['gaits'].get('stance_radius', 150.0)

    @property
    def body_radius(self) -> float:
        """Average distance from body centre to foot contact points (mm)."""
        return self._body_radius

    @property
    def max_speed(self) -> float:
        """Top linear speed (mm/s) at constant cadence: max_stride / stance_duration."""
        return self.max_stride / (self.duty_factor * self.cycle_time)

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
            param_name: Parameter name ('duty_factor', 'step_height', 'max_stride', 'overlap')
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

        Cadence is constant (1 / cycle_time). Each leg's stride scales with its own
        required ground velocity, so faster commands produce bigger steps at the same
        rhythm. If the largest required stride exceeds ``max_stride`` the commanded
        twist is scaled down (linear and angular together, preserving the turn radius)
        so the step stays reachable — this is the only speed cap.

        When velocity is zero and no leg is in swing, the gait freezes.

        Args:
            dt: Time step (seconds)
            velocity: Linear velocity [vx, vy, vz] in body frame (mm/s)
            angular_velocity: Yaw rate (degrees/s)

        Returns:
            Dict mapping leg names to target foot positions [x, y, z] in body frame
        """
        omega_rad = np.radians(angular_velocity)

        # Per-leg ground velocity the foot must track during stance (mm/s).
        foot_velocities = self._compute_foot_velocities(velocity, omega_rad)
        max_foot_speed = max(np.linalg.norm(fv[:2]) for fv in foot_velocities.values())

        # Constant cadence.
        phase_rate = self.cadence
        stance_duration = self.duty_factor * self.cycle_time

        if max_foot_speed > 1e-3:
            # Normal walking. Stride per leg = foot velocity * stance duration.
            self._finishing_swing = False
            stride_vectors = {leg: foot_velocities[leg] * stance_duration for leg in self.leg_names}

            # Reachability clamp: scale the whole twist down if the largest stride
            # would overreach. Scaling both linear and angular keeps the radius.
            max_stride_needed = max(np.linalg.norm(s) for s in stride_vectors.values())
            twist_scale = 1.0
            if max_stride_needed > self.max_stride:
                twist_scale = self.max_stride / max_stride_needed
                stride_vectors = {leg: s * twist_scale for leg, s in stride_vectors.items()}

            self._achieved_velocity = np.asarray(velocity, dtype=float) * twist_scale
            self._achieved_angular_velocity = angular_velocity * twist_scale

            self.phase = (self.phase + phase_rate * dt) % 1.0

        elif any(self.is_leg_in_swing(leg, self.phase) for leg in self.leg_names):
            # Velocity is zero but a leg is still airborne: keep advancing at the
            # constant cadence with zero stride so the swing arcs back to neutral.
            self._finishing_swing = True
            self._achieved_velocity = np.zeros(3)
            self._achieved_angular_velocity = 0.0
            self.phase = (self.phase + phase_rate * dt) % 1.0
            stride_vectors = {leg: np.zeros(3) for leg in self.leg_names}

        else:
            # Fully stopped and all legs grounded: freeze, return neutral.
            self._finishing_swing = False
            self._achieved_velocity = np.zeros(3)
            self._achieved_angular_velocity = 0.0
            return {leg: self.neutral_stance_positions[leg] for leg in self.leg_names}

        return {
            leg: self.compute_foot_position(leg, self.phase, stride_vectors[leg])
            for leg in self.leg_names
        }

    def _compute_foot_velocities(self, velocity: np.ndarray, omega_rad: float) -> dict:
        """
        Compute the per-leg ground velocity each foot must track during stance.

        Combines the robot's linear velocity with the tangential velocity each foot
        experiences due to body yaw.

        Args:
            velocity: Linear velocity [vx, vy, vz] in body frame (mm/s)
            omega_rad: Yaw rate in radians/s

        Returns:
            Dict mapping leg names to velocity vectors [vx, vy, 0] (mm/s)
        """
        linear_vel_3d = np.array([velocity[0], velocity[1], 0.0])
        foot_velocities = {}

        for leg_name in self.leg_names:
            stance_pos = self.neutral_stance_positions[leg_name]

            # Tangential velocity at this foot's position due to body yaw
            r = np.linalg.norm(stance_pos[:2])
            angle = np.arctan2(stance_pos[1], stance_pos[0])
            tangent = np.array([-np.sin(angle), np.cos(angle), 0.0])

            foot_velocities[leg_name] = linear_vel_3d + tangent * r * omega_rad

        return foot_velocities

    # Queries

    def get_achieved_twist(self) -> tuple:
        """
        Return the twist the gait actually executed last update (post-clamp).

        Equal to the commanded twist unless the stride saturated, in which case both
        components were scaled down by the same factor. Useful for honest odometry.

        Returns:
            (velocity [vx, vy, vz] mm/s, angular_velocity deg/s)
        """
        return self._achieved_velocity.copy(), self._achieved_angular_velocity

    def get_gait_info(self) -> dict:
        """Return a snapshot of current gait state for telemetry."""
        return {
            'name': self.current_gait,
            'phase': self.phase,
            'cadence': self.cadence,
            'duty_factor': self.duty_factor,
            'step_height': self.step_height,
            'max_stride': self.max_stride,
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
        self._achieved_velocity = np.zeros(3)
        self._achieved_angular_velocity = 0.0
