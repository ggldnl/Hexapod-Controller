"""
Main controller that manages gait execution and body pose adjustments.

States
------
SETUP       Uninterruptible startup sequence: curl joints -> extend to neutral
            stance -> raise body. Ends in IDLE.

IDLE        Stable state. Accepts body pose, leg, and joint adjustment commands.
            Transitions to WALK on any non-zero velocity command (leg
            are reset to neutral stance first; body pose changes are
            preserved). Transitions to SHUTDOWN on shutdown().

WALK        Gait-driven. Accepts velocity and body pose change commands.
            If a non-zero velocity command was just received from IDLE, a brief
            uninterruptible leg-reset phase runs first (legs interpolate to
            neutral stance), then the gait generator takes over.
            Automatically returns to IDLE when velocity is zero and the gait
            generator has fully settled (all legs grounded).
            Transitions to SHUTDOWN on shutdown() (gait is allowed to finish
            its current swing before the shutdown sequence begins).

SHUTDOWN    Uninterruptible shutdown sequence: finish any residual gait swing
            (only when entered from WALK) -> lower body -> curl joints.
            After completing, the state stays at SHUTDOWN. Call setup() to
            restart from scratch.

All transitions except IDLE->WALK and WALK->IDLE are triggered by explicit
method calls (setup(), shutdown()). IDLE->WALK is triggered by any non-zero
velocity command. WALK->IDLE is automatic once velocity is zero and the gait
has settled.

Velocity commands follow the ROS cmd_vel convention: the controller holds the
last commanded value until overwritten. Zero-velocity watchdog timeouts are
the responsibility of the publishing node.
"""

from enum import Enum
import numpy as np
import logging

from .kinematics import HexapodKinematics
from .gaits import GaitGenerator


class State(Enum):
    """Hexapod controller states."""
    SETUP = 0
    IDLE = 1
    WALK = 2
    SHUTDOWN = 3


class HexapodController:
    """Main hexapod controller."""

    def __init__(self, interface, config: dict, verbose: bool = False, logfile: str = None):

        self.interface = interface
        self.config = config

        # Initialize subsystems
        self.kinematics = HexapodKinematics(config)
        self.gait = GaitGenerator(config)

        # Fix leg names order
        self.leg_names = [k for k, v in config['kinematics']['legs'].items() if isinstance(v, dict)]

        # Gait velocities (held until overwritten — no internal decay)
        self.linear_velocity = np.zeros(3)  # [vx, vy, vz] mm/s in body frame
        self.angular_velocity = 0.0         # wz deg/s in body frame

        # Body pose: persistent offset from the standing reference frame.
        # set_body_position / set_body_orientation update the targets;
        # the controller interpolates toward them every tick.
        # Body pose is preserved across IDLE ↔ WALK transitions.
        self.body_position = np.zeros(3)      # current [x, y, z] offset (mm)
        self.body_orientation = np.zeros(3)   # current [roll, pitch, yaw] (rad)
        self.target_body_position = np.zeros(3)
        self.target_body_orientation = np.zeros(3)

        # Body interpolation speeds
        self.body_linear_velocity = config['safety'].get('body_lin_vel_max', 50.0)   # mm/s
        self.body_angular_velocity = config['safety'].get('body_ang_vel_max', 30.0)  # deg/s

        # Leg positions
        self.leg_positions = {}
        self.target_leg_positions = {}

        # Joint angles (degrees) — used only during joint-space sequencer steps
        self.current_joints = {}
        self.joint_interpolation_speed = config['safety'].get('joint_vel_max', 60.0)  # deg/s

        # Dead-reckoning odometry in world frame
        self.odom_x = 0.0    # mm
        self.odom_y = 0.0    # mm
        self.odom_yaw = 0.0  # rad

        # Step sequencer (used by SETUP and SHUTDOWN)
        self._sequence = []
        self._sequence_index = 0
        self._post_sequence_state = State.IDLE

        # WALK sub-state: True once legs have reached neutral stance and the
        # gait can begin. Set to False whenever WALK is entered from IDLE.
        self._walk_ready = False

        # Status LED — managed inline
        self._led_accumulator = 0.0
        self._led_state = False
        self._led_interval = config.get('led_interval', 0.5)  # seconds per toggle

        # Logging
        self.logger = None
        if verbose:
            self.logger = logging.getLogger(self.__class__.__name__)
            self.logger.setLevel(logging.DEBUG)
            formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')

            console_handler = logging.StreamHandler()
            console_handler.setFormatter(formatter)
            console_handler.setLevel(logging.DEBUG)
            self.logger.addHandler(console_handler)

            if logfile:
                file_handler = logging.FileHandler(logfile)
                file_handler.setFormatter(formatter)
                file_handler.setLevel(logging.INFO)
                self.logger.addHandler(file_handler)

        # Enable hardware, seed joint/leg state, then start setup sequence
        self.enabled = True
        self.state = State.SETUP
        self.enable()
        self._build_setup_sequence()

    # Hardware enable / disable

    def enable(self):
        """Enable interface and read current robot state."""
        if self.logger:
            self.logger.info("Enabling interface.")
        self.interface.enable()

        joint_values = self.interface.get_all_legs()
        self.current_joints = joint_values

        joint_values_rad = {
            k: np.radians(v) if v is not None else None
            for k, v in joint_values.items()
        }
        current_leg_positions = self.kinematics.forward(joint_values_rad)
        self.leg_positions = current_leg_positions
        self.target_leg_positions = {k: v.copy() for k, v in current_leg_positions.items()}

        self.enabled = True

    def disable(self):
        """Disable interface."""
        if self.logger:
            self.logger.info("Disabling interface.")
        self.interface.disable()
        self.enabled = False

    # LED

    def _update_led(self, dt: float):
        """Toggle the status LED at a fixed interval."""
        self._led_accumulator += dt
        if self._led_accumulator >= self._led_interval:
            self._led_accumulator -= self._led_interval
            self._led_state = not self._led_state

        if self._led_state:
            self.interface.set_led(0, 208, 107, 51)
        else:
            self.interface.set_led(0, 0, 0, 0)

    def _led_off(self):
        self._led_state = False
        self.interface.set_led(0, 0, 0, 0)

    # IK and hardware output

    def _send_joint_angles(self) -> bool:
        """
        Run inverse kinematics on current leg positions + body pose and send
        the result to the interface.
        """
        joint_values = self.kinematics.inverse(
            self.leg_positions,
            body_position=self.body_position,
            body_orientation=self.body_orientation,
        )
        joint_values = {
            k: np.degrees(v) if v is not None else None
            for k, v in joint_values.items()
        }

        if self.logger:
            self.logger.debug(
                "Joint targets: " + ', '.join([
                    (
                        f'({joint_values[leg][0]:4.2f}, '
                        f'{joint_values[leg][1]:4.2f}, '
                        f'{joint_values[leg][2]:4.2f})'
                        if joint_values[leg] is not None else 'None'
                    ) for leg in self.leg_names
                ])
            )

        if any(v is None for v in joint_values.values()):
            if self.logger:
                self.logger.warning(
                    f"Legs {[1 if joint_values[l] is None else 0 for l in self.leg_names]} "
                    "could not reach target."
                )
            return False

        result = self.interface.set_all_legs(joint_values)
        if result:
            self.current_joints = joint_values  # sync only when hardware accepted
        return result

    # Main update

    def update(self, dt: float) -> bool:
        """
        Advance controller state and send new joint angles.
        Call this in the main control loop.

        Args:
            dt: Time step in seconds
        """
        self._update_led(dt)

        if not self.enabled:
            return False

        if not self.interface.check():
            if self.logger:
                self.logger.warning(
                    f"Emergency stop: voltage={self.interface.get_voltage()}, "
                    f"current={self.interface.get_current()}"
                )
            self.emergency_stop()
            return False

        if self.state is State.SETUP:
            return self._update_sequencer(dt)

        if self.state is State.IDLE:
            return self._update_idle(dt)

        if self.state is State.WALK:
            return self._update_walk(dt)

        if self.state is State.SHUTDOWN:
            return self._update_sequencer(dt)

        return False

    # Per-state updates

    def _update_idle(self, dt: float) -> bool:
        """Interpolate body pose and individually commanded leg positions."""
        self._interpolate_body_pose(dt)
        self._interpolate_leg_positions(dt)
        return self._send_joint_angles()

    def _update_walk(self, dt: float) -> bool:
        """
        Walk update.

        Phase 1 — _walk_ready is False: interpolate legs to neutral stance
                  before starting the gait (uninterruptible). Body pose
                  interpolation still runs so any new target takes effect.

        Phase 2 — _walk_ready is True: gait generator drives leg positions.
                  Automatically transitions to IDLE when velocity is zero and
                  the gait has fully settled.
        """
        self._interpolate_body_pose(dt)
        self._update_odometry(dt)

        if not self._walk_ready:
            # Leg-reset phase: bring all legs to neutral before gait starts
            self._interpolate_leg_positions(dt)
            if self.are_leg_positions_settled():
                self._walk_ready = True
                self.gait.reset()
            return self._send_joint_angles()

        if self._gait_is_active():
            positions = self.gait.update(dt, self.linear_velocity, self.angular_velocity)
            self.leg_positions = positions
            self.target_leg_positions = {k: v.copy() for k, v in positions.items()}
        else:
            # Velocity is zero and gait has fully settled — return to IDLE
            if self.logger:
                self.logger.info("Gait settled, transitioning to IDLE.")
            self.state = State.IDLE

        return self._send_joint_angles()

    # Step sequencer

    def _update_sequencer(self, dt: float) -> bool:
        """Run the current step; advance when it returns True."""
        if self._sequence_index >= len(self._sequence):
            self.state = self._post_sequence_state
            return True

        done = self._sequence[self._sequence_index](dt)
        if done:
            self._sequence_index += 1
            if self._sequence_index >= len(self._sequence):
                self.state = self._post_sequence_state

        return True

    # Sequencer steps

    def _step_finish_gait(self, dt: float) -> bool:
        """
        Keep running the gait at zero velocity until all legs have landed.
        Used as the first shutdown step when shutdown() is called from WALK.
        """
        self._interpolate_body_pose(dt)
        if self._gait_is_active():
            positions = self.gait.update(dt, np.zeros(3), 0.0)
            self.leg_positions = positions
            self.target_leg_positions = {k: v.copy() for k, v in positions.items()}
            self._send_joint_angles()
            return False
        return True

    def _step_curl_joints(self, dt: float) -> bool:
        """
        Interpolate joints to the curled (safe power-off) position.
        When done, seeds leg_positions from FK so the next IK-based step
        starts from where the robot actually is.
        """
        curled_joints = {
            leg_name: np.array([
                0,
                self.interface.max_femur_kinematic_space,
                self.interface.max_tibia_kinematic_space
            ]) for leg_name in self.leg_names
        }

        previous_joints = {k: v.copy() for k, v in self.current_joints.items()}
        self._interpolate_joints(dt, curled_joints)

        result = self.interface.set_all_legs(self.current_joints)
        if not result:
            self.current_joints = previous_joints  # restore if hardware rejected

        if self.logger:
            self.logger.debug(
                "Curl joints: " + ', '.join([
                    (
                        f'({self.current_joints[leg][0]:4.2f}, '
                        f'{self.current_joints[leg][1]:4.2f}, '
                        f'{self.current_joints[leg][2]:4.2f})'
                        if self.current_joints[leg] is not None else 'None'
                    ) for leg in self.leg_names
                ])
            )

        settled = all(
            np.linalg.norm(curled_joints[n] - self.current_joints[n]) < 1e-3
            for n in self.leg_names
        )
        if settled:
            joints_rad = {leg: np.radians(self.current_joints[leg]) for leg in self.leg_names}
            self.leg_positions = self.kinematics.forward(joints_rad)
            self.target_leg_positions = {
                k: v.copy() for k, v in self.gait.neutral_stance_positions.items()
            }

        return settled

    def _step_extend_to_neutral(self, dt: float) -> bool:
        """
        Interpolate leg positions from curled to neutral stance.
        When done, arms the body raise by setting the standing-height target.
        """
        self._interpolate_leg_positions(dt)
        self._send_joint_angles()

        if self.are_leg_positions_settled():

            # The next step (_step_interpolate_body) will interpolate to the target
            # body position (z=standing_height). This means at the end of the
            # interpolation body_position will be (0, 0, standing_height), while
            # it should represent an offset relative to the standing reference frame.
            # We will use this machinery but reset self.body_position afterward
            standing_height = self.config['gaits'].get('standing_height', 80.0)
            self.target_body_position = np.array([0.0, 0.0, standing_height])
            return True

        return False

    def _step_interpolate_body(self, dt: float) -> bool:
        """
        Interpolate body pose toward the current target.
        Used both for raising (setup) and lowering (shutdown).
        """
        self._interpolate_body_pose(dt)
        self._send_joint_angles()

        if self.is_body_pose_settled():

            # Reset body height to 0: body_position represents an offset relative to
            # the standing reference frame
            self.body_position = np.array([0.0, 0.0, 0.0])
            self.target_body_position = np.array([0.0, 0.0, 0.0])
            return True

        return False

    # Sequence builders

    def _build_setup_sequence(self):
        """Populate the sequencer for the startup path."""
        self._sequence = [
            self._step_curl_joints,
            self._step_extend_to_neutral,
            self._step_interpolate_body,
        ]
        self._sequence_index = 0
        self._post_sequence_state = State.IDLE

        if self.logger:
            self.logger.info("Starting setup sequence.")

    def _build_shutdown_sequence(self, from_walk: bool = False):
        """
        Populate the sequencer for the shutdown path.

        Args:
            from_walk: If True, prepend a gait-finishing step so any airborne
                       legs land before the curl begins.
        """
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = 0.0
        self.target_body_position = np.zeros(3)
        self.target_body_orientation = np.zeros(3)

        steps = []
        if from_walk:
            steps.append(self._step_finish_gait)
        steps += [
            self._step_interpolate_body,  # lower body back to reference
            self._step_curl_joints,       # curl legs for safe power-off
        ]

        self._sequence = steps
        self._sequence_index = 0
        self._post_sequence_state = State.SHUTDOWN  # stays in SHUTDOWN when done

        if self.logger:
            self.logger.info("Starting shutdown sequence.")

    # Interpolation helpers

    def _interpolate_body_pose(self, dt: float):
        """Smoothly move body_position / body_orientation toward their targets."""

        # Position
        pos_err = self.target_body_position - self.body_position
        pos_mag = np.linalg.norm(pos_err)
        if pos_mag > 1e-3:
            max_step = self.body_linear_velocity * dt
            self.body_position += pos_err / pos_mag * min(pos_mag, max_step)

        # Orientation
        ori_err = self.target_body_orientation - self.body_orientation
        ori_err[2] = np.arctan2(np.sin(ori_err[2]), np.cos(ori_err[2]))  # wrap yaw
        ori_mag = np.linalg.norm(ori_err)
        if ori_mag > 1e-3:
            max_step = np.radians(self.body_angular_velocity) * dt
            self.body_orientation += ori_err / ori_mag * min(ori_mag, max_step)

    def _interpolate_leg_positions(self, dt: float):
        """Move each leg toward its target at body_linear_velocity (mm/s)."""
        max_step = self.body_linear_velocity * dt
        for leg_name in self.leg_names:
            err = self.target_leg_positions[leg_name] - self.leg_positions[leg_name]
            dist = np.linalg.norm(err)
            if dist > 1e-3:
                self.leg_positions[leg_name] += err / dist * min(dist, max_step)
            else:
                self.leg_positions[leg_name] = self.target_leg_positions[leg_name].copy()

    def _interpolate_joints(self, dt: float, target_joints: dict):
        """
        Move current_joints toward target_joints at joint_interpolation_speed (deg/s).
        Used only during joint-space sequencer steps.

        Args:
            dt:            Time step in seconds
            target_joints: {leg_name: np.array([coxa, femur, tibia])} in degrees
        """
        max_step = self.joint_interpolation_speed * dt
        for leg_name in self.leg_names:
            err = target_joints[leg_name] - self.current_joints[leg_name]
            dist = np.linalg.norm(err)
            if dist > 1e-3:
                self.current_joints[leg_name] += err / dist * min(dist, max_step)
            else:
                self.current_joints[leg_name] = target_joints[leg_name].copy()

    def _update_odometry(self, dt: float):
        """Integrate body-frame velocity into world-frame dead-reckoning odometry."""
        vx = self.linear_velocity[0]
        vy = self.linear_velocity[1]
        wz = np.radians(self.angular_velocity)

        self.odom_x += (vx * np.cos(self.odom_yaw) + vy * np.sin(self.odom_yaw)) * dt
        self.odom_y += (vx * np.sin(self.odom_yaw) - vy * np.cos(self.odom_yaw)) * dt
        self.odom_yaw += wz * dt

    # Internal helpers

    def _effective_speed(self) -> float:
        """
        Scalar speed used to decide whether the gait is active.
        Combines linear speed with the tangential equivalent of yaw rate
        at the nominal stance radius, matching the formula in GaitGenerator.
        """
        linear_speed = np.linalg.norm(self.linear_velocity[:2])
        angular_speed_equiv = abs(np.radians(self.angular_velocity)) * self.gait.stance_radius
        return linear_speed + angular_speed_equiv

    def _gait_is_active(self) -> bool:
        """True when the gait generator is driving leg positions."""
        return self._effective_speed() > 1e-3 or self.gait.is_finishing_swing()

    def _maybe_enter_walk(self):
        """
        If a non-zero velocity was just set and the robot is in IDLE, begin
        the IDLE->WALK transition: reset legs to neutral stance, then start gait.
        Body pose is preserved.
        """
        if self.state is not State.IDLE:
            return
        if self._effective_speed() < 1e-3:
            return

        if self.logger:
            self.logger.info("Transitioning IDLE -> WALK (leg reset phase).")

        self.target_leg_positions = {
            k: v.copy() for k, v in self.gait.neutral_stance_positions.items()
        }
        self._walk_ready = False
        self.state = State.WALK

    # State queries

    def is_setup_complete(self) -> bool:
        """Return True once the startup sequence has finished."""
        return self.state in {State.IDLE, State.WALK}

    def is_walking(self) -> bool:
        """Return True when the gait is actively driving leg positions."""
        return self.state is State.WALK and self._walk_ready and self._gait_is_active()

    def is_busy(self) -> bool:
        """
        Return True while an uninterruptible sequence is running (SETUP or
        SHUTDOWN) or while the leg-reset phase at the start of WALK is active.
        """
        if self.state in {State.SETUP, State.SHUTDOWN}:
            return True
        if self.state is State.WALK and not self._walk_ready:
            return True
        return False

    def is_body_pose_settled(self, position_threshold: float = 1.0,
                              orientation_threshold: float = 1.0) -> bool:
        """
        Return True when body pose has reached its target.

        Args:
            position_threshold:    Position error threshold (mm)
            orientation_threshold: Orientation error threshold (degrees)
        """
        position_error = np.linalg.norm(self.target_body_position - self.body_position)
        orientation_error = np.degrees(
            np.linalg.norm(self.target_body_orientation - self.body_orientation)
        )
        return position_error < position_threshold and orientation_error < orientation_threshold

    def are_leg_positions_settled(self, position_threshold: float = 1.0) -> bool:
        """
        Return True when all legs have reached their targets.

        Args:
            position_threshold: Position error threshold (mm)
        """
        return all(
            np.linalg.norm(self.target_leg_positions[n] - self.leg_positions[n])
            < position_threshold
            for n in self.leg_names
        )

    # Velocity control (IDLE and WALK)

    def set_linear_velocity(self, vx: float, vy: float, vz: float = 0.0):
        """
        Set robot linear velocity. Held until overwritten.
        Calling with a non-zero value from IDLE triggers a transition to WALK.

        Args:
            vx: Forward velocity (mm/s, positive = forward)
            vy: Sideways velocity (mm/s, positive = right)
            vz: Vertical velocity (mm/s, usually 0)
        """
        if self.state not in {State.IDLE, State.WALK}:
            if self.logger:
                self.logger.warning(
                    f"set_linear_velocity() ignored — state is {self.state.name}"
                )
            return

        max_linear = self.config['safety']['lin_vel_max']
        self.linear_velocity = np.array([
            np.clip(vx, -max_linear, max_linear),
            np.clip(vy, -max_linear, max_linear),
            np.clip(vz, -max_linear, max_linear),
        ])

        self._maybe_enter_walk()

        if self.logger:
            self.logger.info(
                f"Linear velocity: {self.linear_velocity[0]:.1f}, "
                f"{self.linear_velocity[1]:.1f}, {self.linear_velocity[2]:.1f} mm/s"
            )

    def set_angular_velocity(self, wz: float):
        """
        Set robot angular velocity. Held until overwritten.
        Calling with a non-zero value from IDLE triggers a transition to WALK.

        Args:
            wz: Yaw rate (deg/s, positive = counter-clockwise)
        """
        if self.state not in {State.IDLE, State.WALK}:
            if self.logger:
                self.logger.warning(
                    f"set_angular_velocity() ignored — state is {self.state.name}"
                )
            return

        max_angular = self.config['safety']['ang_vel_max']
        self.angular_velocity = float(np.clip(wz, -max_angular, max_angular))

        self._maybe_enter_walk()

        if self.logger:
            self.logger.info(f"Angular velocity: {self.angular_velocity:.1f} deg/s")

    # Body pose control (IDLE and WALK)

    def set_body_position(self, dx: float, dy: float, dz: float):
        """
        Set body position offset relative to the standing reference frame.
        Persistent: stays until explicitly changed. Accepted in IDLE and WALK.

        Args:
            dx: Forward/backward shift (mm, positive = forward)
            dy: Left/right shift (mm, positive = right)
            dz: Up/down shift (mm, positive = up)
        """
        if self.state not in {State.IDLE, State.WALK}:
            if self.logger:
                self.logger.warning(
                    f"set_body_position() ignored — state is {self.state.name}"
                )
            return

        x_range = self.config['safety'].get('x_range', (-50, 50))
        y_range = self.config['safety'].get('y_range', (-50, 50))
        z_range = self.config['safety'].get('z_range', (-50, 50))

        self.target_body_position[:] = [
            np.clip(dx, *x_range),
            np.clip(dy, *y_range),
            np.clip(dz, *z_range),
        ]

        if self.logger:
            self.logger.info(
                f"Body position target: {self.target_body_position[0]:.1f}, "
                f"{self.target_body_position[1]:.1f}, {self.target_body_position[2]:.1f} mm"
            )

    def set_body_orientation(self, roll: float, pitch: float, yaw: float):
        """
        Set body orientation offset relative to the standing reference frame.
        Persistent: stays until explicitly changed. Accepted in IDLE and WALK.

        Args:
            roll:  Body roll (degrees, positive = right side down)
            pitch: Body pitch (degrees, positive = nose down)
            yaw:   Body yaw (degrees, positive = counter-clockwise)
        """
        if self.state not in {State.IDLE, State.WALK}:
            if self.logger:
                self.logger.warning(
                    f"set_body_orientation() ignored — state is {self.state.name}"
                )
            return

        roll_range = self.config['safety'].get('roll_range', (-10, 10))
        pitch_range = self.config['safety'].get('pitch_range', (-10, 10))
        yaw_range = self.config['safety'].get('yaw_range', (-10, 10))

        self.target_body_orientation[:] = np.radians([
            np.clip(roll, *roll_range),
            np.clip(pitch, *pitch_range),
            np.clip(yaw, *yaw_range),
        ])

        if self.logger:
            self.logger.info(
                f"Body orientation target: "
                f"{np.degrees(self.target_body_orientation[0]):.1f}, "
                f"{np.degrees(self.target_body_orientation[1]):.1f}, "
                f"{np.degrees(self.target_body_orientation[2]):.1f} deg"
            )

    # Individual leg control (IDLE only)

    def set_leg_position(self, leg_name: str, x: float, y: float, z: float):
        """
        Set target foot position for a single leg (mm, body frame).
        Only accepted in IDLE.
        """
        if self.state is not State.IDLE:
            if self.logger:
                self.logger.warning(
                    f"set_leg_position() ignored — state is {self.state.name}"
                )
            return

        if leg_name not in self.leg_names:
            if self.logger:
                self.logger.warning(
                    f"set_leg_position() ignored — {leg_name} does not exist"
                )
            return

        self.target_leg_positions[leg_name] = np.array([x, y, z])

        if self.logger:
            self.logger.info(f"Leg {leg_name} target: {x:.1f}, {y:.1f}, {z:.1f} mm")

    def set_joint_angles(self, leg_name: str, coxa: float, femur: float, tibia: float):
        """
        Command joint angles (degrees) for a single leg via forward kinematics.
        Only accepted in IDLE.
        """
        if self.state is not State.IDLE:
            if self.logger:
                self.logger.warning(
                    f"set_joint_angles() ignored — state is {self.state.name}"
                )
            return

        if leg_name not in self.leg_names:
            if self.logger:
                self.logger.warning(
                    f"set_joint_angles() ignored — {leg_name} does not exist"
                )
            return

        angles_rad = np.radians([coxa, femur, tibia])
        foot_pos = self.kinematics.forward({leg_name: angles_rad})[leg_name]
        self.target_leg_positions[leg_name] = foot_pos

        if self.logger:
            self.logger.info(
                f"Leg {leg_name} joint target: {coxa:.1f}, {femur:.1f}, {tibia:.1f} deg"
            )

    # Gait parameters

    def set_gait(self, gait_name: str):
        """Change gait pattern."""
        self.gait.set_gait(gait_name)

    def set_gait_parameter(self, param_name: str, value: float):
        """Adjust a gait parameter in real-time."""
        self.gait.set_gait_parameter(param_name, value)

    # Lifecycle

    def shutdown(self):
        """
        Begin the orderly shutdown sequence.
        Accepted from IDLE or WALK. Ignored during SETUP or while already
        in SHUTDOWN.

        When called from WALK, velocities are zeroed and a gait-finishing step
        is prepended so any airborne legs land before curling begins.
        """
        if self.state not in {State.IDLE, State.WALK}:
            if self.logger:
                self.logger.warning(
                    f"shutdown() ignored — state is {self.state.name}"
                )
            return

        from_walk = (self.state is State.WALK)
        self._build_shutdown_sequence(from_walk=from_walk)
        self.state = State.SHUTDOWN

    def setup(self):
        """
        Re-run the startup sequence. Only accepted from SHUTDOWN.
        Resets odometry, re-enables hardware, and starts the setup sequence.
        """
        if self.state is not State.SHUTDOWN:
            if self.logger:
                self.logger.warning(
                    f"setup() ignored — state is {self.state.name}"
                )
            return

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.enable()
        self._build_setup_sequence()
        self.state = State.SETUP

    # Emergency stop

    def emergency_stop(self):
        """Immediately halt all motion and disable hardware."""
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = 0.0

        # Freeze body pose where it is
        self.target_body_position = self.body_position.copy()
        self.target_body_orientation = self.body_orientation.copy()

        self._led_off()
        self.disable()

    # Telemetry

    def get_status(self) -> dict:
        """Return a snapshot of current robot state."""
        return {
            'enabled': self.enabled,
            'state': self.state.name,
            'body_position': self.body_position.tolist(),
            'body_orientation': np.degrees(self.body_orientation).tolist(),
            'linear_velocity': self.linear_velocity.tolist(),
            'angular_velocity': self.angular_velocity,
            'leg_positions': {k: v.tolist() for k, v in self.leg_positions.items()},
            'joint_values': {k: v.tolist() for k, v in self.current_joints.items()},
            'odometry': {
                'x': self.odom_x,
                'y': self.odom_y,
                'yaw': self.odom_yaw,
            },
            'gait': self.gait.get_gait_info(),
            'battery_voltage': self.interface.get_voltage(),
            'current_draw': self.interface.get_current(),
        }