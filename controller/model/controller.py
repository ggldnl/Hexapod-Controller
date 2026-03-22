"""
Main controller that manages gait execution and body pose adjustments.

SETUP               Startup sequence: curl legs (joint-space), then rise to standing
                    height (IK). Ends in IDLE.
IDLE                Free control: body pose, individual legs/joints.
WALKING             Gait-driven: only velocity / gait commands accepted.
IDLE_TO_WALKING     Transition: legs interpolate to neutral stance, then gait starts.
WALKING_TO_IDLE     Transition: gait runs until all legs are in stance, then legs
                    interpolate to neutral stance, then idle state begins.
SHUTDOWN            Shutdown sequence: lower the body, curl legs, done.

All transitions are fire-and-forget: call transition_to_walking() /
transition_to_idle() and the state advances on each update() call.
Poll self.state or is_transition_complete() to know when it is done.
"""

from enum import Enum
import numpy as np
import logging

from .kinematics import HexapodKinematics
from .gaits import GaitGenerator


class State(Enum):
    """Hexapod states."""

    """
    SETUP is split into three states:
    1. We curl the legs to a position where standing is easy - CURL
    2. We reach the neutral stance position - EXTEND
    3. We raise the body - RAISE
    """
    SETUP_CURL = 0
    SETUP_EXTEND = 1
    SETUP_RAISE = 2

    """
    Main states
    """
    IDLE = 3
    IDLE_TO_WALKING = 4
    WALKING = 5
    WALKING_TO_IDLE = 6

    """
    SHUTDOWN is split into three states:
    1. We lower the body - LOWER
    2. We interpolate the joints to curled position - CURL
    3. Shutdown completed - DONE
    """
    SHUTDOWN_LOWER = 7  # lower body back to z=0
    SHUTDOWN_CURL = 8  # interpolate joints to curled position
    SHUTDOWN_DONE = 9


class LED:
    """
    Status LED automatically toggles on and off at a given frequency.
    """

    def __init__(self, interface, interval: float = 0.5):
        self.interface = interface
        self.interval = interval
        self.accumulator = 0.0
        self.state = False

    def update(self, dt: float):

        self.accumulator += dt
        if self.accumulator >= self.interval:
            self.accumulator -= self.interval
            self.state = not self.state

        # TODO set led color based on status (status_ok, status_error, ...)
        if self.state:
            self.interface.set_led(0, 208, 107, 51)
        else:
            self.interface.set_led(0, 0, 0, 0)

    def off(self):
        self.interface.set_led(0, 0, 0, 0)


class HexapodController:
    """
    Main hexapod controller.
    """

    def __init__(self, interface, config: dict, verbose:bool = False, logfile:str=None):

        self.interface = interface
        self.config = config

        # Initialize subsystems
        self.kinematics = HexapodKinematics(config)
        self.gait = GaitGenerator(config)

        # Fix leg names order
        self.leg_names = [k for k, v in config['kinematics']['legs'].items() if isinstance(v, dict)]

        # Gait velocities
        self.linear_velocity = np.zeros(3)  # [vx, vy, vz] mm/s in body frame
        self.angular_velocity = 0           # wz deg/s in body frame

        # Current and target body pose
        self.body_position = np.zeros(3)  # additional [x, y, z] offset
        self.body_orientation = np.zeros(3)  # [roll, pitch, yaw] in degrees
        self.target_body_position = np.zeros(3)
        self.target_body_orientation = np.zeros(3)

        # Body interpolation speed
        self.body_linear_velocity = config['safety'].get('body_lin_vel_max', 50.0)  # mm/s
        self.body_angular_velocity = config['safety'].get('body_ang_vel_max', 30.0) # deg/s

        # Current and target leg positions
        self.leg_positions = {}
        self.target_leg_positions = {}

        """
        I don't like both keeping track of leg/body position/pose and joint values
        because they are two faces of the same coin. Keeping track of both means
        each time we update leg positions or body pose, we need to reflect the 
        change on joint values and vice versa. This can be avoided since we have 
        only one state that works directly by interpolating joint angles (SETUP).
        """
        # Current and target joint values
        self.current_joints = {}
        # self.target_joint_values = {}

        self.joint_interpolation_speed = config['safety'].get('joint_vel_max', 60.0) # deg/s

        # Dead-reckoning odometry in world frame
        self.odom_x = 0.0  # mm
        self.odom_y = 0.0  # mm
        self.odom_yaw = 0.0  # radians

        # Current state
        self.state = State.SETUP_CURL

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

        # Status LED
        self.led = LED(interface, interval=0.5)

        # Finally, enable the robot
        self.enabled = True
        self.enable()  # also enables the interface

    def enable(self):
        """Enable interface."""

        if self.logger: self.logger.info("Enabling interface.")
        self.interface.enable()

        # Get current joint angles (degrees)
        # self.joint_values = self.interface.get_all_legs()
        # self.target_joint_values = self.joint_values
        joint_values = self.interface.get_all_legs()
        self.current_joints = joint_values

        # Compute current leg positions
        joint_values_rad = {
            k: np.radians(v) if v is not None else None
            for k, v in joint_values.items()
        }
        current_leg_positions = self.kinematics.forward(joint_values_rad)
        self.leg_positions = current_leg_positions
        self.target_leg_positions = current_leg_positions

        self.enabled = True

    def disable(self):
        """Disable interface."""

        if self.logger: self.logger.info("Disabling interface.")
        self.interface.disable()
        self.enabled = False

    # Send joint values to interface

    def _send_joint_angles(self):
        """
        Performs inverse kinematics on the target foot positions and body pose,
        converts the result to degrees and send them to the interface.
        """

        # Find joint angles corresponding to foot positions
        joint_values = self.kinematics.inverse(
            self.leg_positions,
            body_position=self.body_position,
            body_orientation=self.body_orientation,
        )

        # Convert joint angles: the cpp libraries on the robot assume the angles are expressed in degrees
        joint_values = {
            k: np.degrees(v) if v is not None else None
            for k, v in joint_values.items()
        }
        if self.logger: self.logger.debug(
            "Target joint values  : " + ', '.join([
                (
                    f'({joint_values[leg][0]:4.2f}, '
                    f'{joint_values[leg][1]:4.2f}, '
                    f'{joint_values[leg][2]:4.2f})'
                     if joint_values[leg] is not None else 'None'
                ) for leg in self.leg_names
            ])
        )

        # Apply the angles if possible
        if any(v is None for v in joint_values.values()):
            if self.logger: self.logger.warning(
                f"Legs {[1 if joint_values[l] is None else 0 for l in self.leg_names]} "
                "could not reach target."
            )
            return False

        result = self.interface.set_all_legs(joint_values)
        if result:
            self.current_joints = joint_values  # only sync if hardware accepted the command
        return result

    # Update

    def update(self, dt: float):
        """
        Update controller state and compute new joint angles.
        This method should be called in the main control loop.

        Args:
            dt: Time step in seconds
        """

        # TODO make status led change color based on state
        self.led.update(dt)

        if not self.enabled:
            return False

        # Safety check
        if not self.interface.check():
            if self.logger: self.logger.warning(
                f"Emergency stop: voltage={self.interface.get_voltage()}, "
                f"current={self.interface.get_current()}"
            )
            self.emergency_stop()
            return False

        # Dispatch to per-state update

        # Setup states
        if self.state is State.SETUP_CURL:
            return self._update_setup_curl(dt)

        elif self.state is State.SETUP_EXTEND:
            return self._update_setup_extend(dt)

        elif self.state is State.SETUP_RAISE:
            return self._update_setup_raise(dt)

        # Main states
        elif self.state is State.IDLE:
            return self._update_idle(dt)

        elif self.state is State.WALKING:
            return self._update_walking(dt)

        elif self.state is State.IDLE_TO_WALKING:
            return self._update_idle_to_walking(dt)

        elif self.state is State.WALKING_TO_IDLE:
            return self._update_walking_to_idle(dt)

        # Shutdown states
        elif self.state is State.SHUTDOWN_LOWER:
            return self._update_shutdown_lower(dt)

        elif self.state is State.SHUTDOWN_CURL:
            return self._update_shutdown_curl(dt)

        else:
            return False  # unreachable

    def _update_idle(self, dt: float) -> bool:
        """Update in IDLE state: interpolate body pose, manually control legs/joints."""

        self._interpolate_body_pose(dt)
        self._interpolate_leg_positions(dt)
        return self._send_joint_angles()

    def _update_idle_to_walking(self, dt: float) -> bool:
        """Update in IDLE_TO_WALKING state: interpolate legs to neutral stance position."""

        # self._interpolate_body_pose(dt)
        self._interpolate_leg_positions(dt)
        if self.are_leg_positions_settled(): # and self.is_body_pose_settled():

            # Reset gait generator
            self.gait.reset()

            # Reset hexapod state
            self.state = State.WALKING

        return self._send_joint_angles()

    def _update_walking(self, dt: float) -> bool:
        """Update in WALKING state: gait loop."""

        """
        No need to interpolate leg positions here because in this case leg positions will be
        updated at each update cycle by the gait generator.
        """
        self._interpolate_body_pose(dt)
        self._update_odometry(dt)
        self.target_leg_positions = self.gait.update(dt, self.linear_velocity, self.angular_velocity)
        self.leg_positions = self.target_leg_positions

        return self._send_joint_angles()

    def _update_walking_to_idle(self, dt: float, tolerance: float = 1e-3) -> bool:
        """
        Update in WALKING_TO_IDLE state.

        Phase 1 — swing legs are still airborne: keep running gait at zero velocity
                  so they land naturally, mirroring positions into leg_positions.
        Phase 2 — all legs grounded: interpolate toward neutral stance, then go IDLE.
        """

        self._interpolate_body_pose(dt)
        self._update_odometry(dt)  # keep integrating while legs are settling
        self.target_leg_positions = self.gait.update(dt, self.linear_velocity, self.angular_velocity)
        self.leg_positions = self.target_leg_positions

        # If all legs are at phase end, we can interpolate to neutral stance phase
        if all(
            np.linalg.norm(self.leg_positions[n] - self.gait.neutral_stance_positions[n]) < tolerance
            for n in self.leg_names
        ):

            # Zero velocities
            self.linear_velocity = np.zeros(3)
            self.angular_velocity = 0

            # Switch to IDLE state
            self.state = State.IDLE

        return self._send_joint_angles()

    def _update_setup_curl(self, dt: float):
        """Update in SETUP_CURL state: interpolate joints to curl the legs."""

        curled_joints = {
            leg_name: np.array([
                0,
                self.interface.max_femur_kinematic_space,
                self.interface.max_tibia_kinematic_space
            ]) for leg_name in self.leg_names
        }

        previous_joints = self.current_joints
        self._interpolate_joints(dt, curled_joints)

        result = self.interface.set_all_legs(self.current_joints)
        if not result:
            self.current_joints = previous_joints  # restore if hardware rejected the command

        if self.logger: self.logger.debug(
            "Target joint values  : " + ', '.join([
                (
                    f'({self.current_joints[leg][0]:4.2f}, '
                    f'{self.current_joints[leg][1]:4.2f}, '
                    f'{self.current_joints[leg][2]:4.2f})'
                     if self.current_joints[leg] is not None else 'None'
                ) for leg in self.leg_names
            ])
        )

        are_joints_settled = all(
            np.linalg.norm(curled_joints[n] - self.current_joints[n]) < 1e-3
            for n in self.leg_names
        )
        if are_joints_settled:
            # Seed foot positions from the curled joint state so IK starts
            # from where the robot actually is
            current_joints_rad = {leg_name: np.radians(self.current_joints[leg_name]) for leg_name in self.leg_names}
            self.leg_positions = self.kinematics.forward(current_joints_rad)
            self.target_leg_positions = self.gait.neutral_stance_positions

            # Update state
            self.state = State.SETUP_EXTEND

        return True  # joint angles already sent above

    def _update_setup_extend(self, dt: float):
        """
        Update in SETUP_EXTEND state:
        from curled legs, extend to reach the neutral stance position before lowering them.
        """

        self._interpolate_leg_positions(dt)

        if self.are_leg_positions_settled():
            self.target_body_position = np.array([0.0, 0.0, 80.0])
            self.state = State.SETUP_RAISE

        return self._send_joint_angles()

    def _update_setup_raise(self, dt: float):
        """Update in SETUP_RAISE state: raise the body."""

        self._interpolate_body_pose(dt)
        # self._interpolate_leg_positions(dt)

        if self.is_body_pose_settled():
            self.state = State.IDLE

        return self._send_joint_angles()

    def _update_shutdown_lower(self, dt: float) -> bool:
        """Smoothly bring body_position and body_orientation back to zero."""

        self._interpolate_body_pose(dt)

        if self.is_body_pose_settled():
            self.state = State.SHUTDOWN_CURL

        return self._send_joint_angles()

    def _update_shutdown_curl(self, dt: float) -> bool:
        """Drive joints to the same curled position used in SETUP_CURL."""

        # Begin shutdown_curl: interpolate joints to curled
        curled_joints = {
            leg_name: np.array([
                0,
                self.interface.max_femur_kinematic_space,
                self.interface.max_tibia_kinematic_space
            ]) for leg_name in self.leg_names
        }

        previous_joints = self.current_joints
        self._interpolate_joints(dt, curled_joints)

        result = self.interface.set_all_legs(self.current_joints)
        if not result:
            self.current_joints = previous_joints  # restore if hardware rejects the command

        if self.logger: self.logger.debug(
            "Target joint values  " + ', '.join([
                (
                    f'({self.current_joints[leg][0]:4.2f}, '
                    f'{self.current_joints[leg][1]:4.2f}, '
                    f'{self.current_joints[leg][2]:4.2f})'
                    if self.current_joints[leg] is not None else 'None'
                )
                for leg in self.leg_names
            ])
        )

        are_joints_settled = all(
            np.linalg.norm(curled_joints[n] - self.current_joints[n]) < 1e-3
            for n in self.leg_names
        )
        if are_joints_settled:

            # Update state
            self.state = State.SHUTDOWN_DONE
            self.disable()

        return True

    def _update_odometry(self, dt: float):
        """Integrate body-frame velocity into world-frame odometry."""

        vx = self.linear_velocity[0]
        vy = self.linear_velocity[1]
        wz = np.radians(self.angular_velocity)

        # Rotate body-frame velocity into world frame using current yaw
        self.odom_x += (vx * np.cos(self.odom_yaw) - vy * np.sin(self.odom_yaw)) * dt
        self.odom_y += (vx * np.sin(self.odom_yaw) + vy * np.cos(self.odom_yaw)) * dt
        self.odom_yaw += wz * dt

    # Interpolation

    def _interpolate_body_pose(self, dt: float):
        """Smoothly move body_position / body_orientation toward their targets."""

        # Position
        pos_err = self.target_body_position - self.body_position
        pos_mag = np.linalg.norm(pos_err)
        if pos_mag > 1e-3:
            max_step = self.body_linear_velocity * dt
            step = pos_err / pos_mag * min(pos_mag, max_step)
            self.body_position += step

        # Orientation
        ori_err = self.target_body_orientation - self.body_orientation
        ori_err[2] = np.arctan2(np.sin(ori_err[2]), np.cos(ori_err[2]))  # wrap yaw
        ori_mag = np.linalg.norm(ori_err)
        if ori_mag > 1e-3:
            max_step = np.radians(self.body_angular_velocity) * dt
            step = ori_err / ori_mag * min(ori_mag, max_step)
            self.body_orientation += step

    def _interpolate_leg_positions(self, dt: float):
        """
        Move each leg toward its target at body_linear_velocity (mm/s).
        """
        max_step = self.body_linear_velocity * dt
        for leg_name in self.leg_names:
            err = self.target_leg_positions[leg_name] - self.leg_positions[leg_name]
            dist = np.linalg.norm(err)
            if dist > 1e-3:
                self.leg_positions[leg_name] += err / dist * min(dist, max_step)
            else:
                self.leg_positions[leg_name] = self.target_leg_positions[leg_name]

    def _interpolate_joints(self, dt: float, target_joints: dict):
        """
        Move self._current_joints toward target_joints at body_linear_velocity (deg/s).
        Used only during the SETUP and SHUTDOWN states.

        Args:
            dt: time step in seconds
            target_joints: {leg_name: np.array([coxa, femur, tibia])} in degrees
        """

        max_step = self.joint_interpolation_speed * dt  # deg/step
        for leg_name in self.leg_names:
            err = target_joints[leg_name] - self.current_joints[leg_name]
            dist = np.linalg.norm(err)
            if dist > 1e-3:
                self.current_joints[leg_name] += err / dist * min(dist, max_step)
            else:
                self.current_joints[leg_name] = target_joints[leg_name]

    # External control (IDLE, WALKING, SHUTDOWN)

    def is_transition_complete(self) -> bool:
        """Return True when not in a transitional state."""
        return self.state in {
            State.IDLE,
            State.WALKING,
            State.SHUTDOWN_DONE
        }

    def is_transition_ongoing(self) -> bool:
        """Return True when in a transitional state."""
        return self.state in {
            State.IDLE_TO_WALKING,
            State.WALKING_TO_IDLE,
            State.SETUP_CURL,
            State.SETUP_EXTEND,
            State.SETUP_RAISE,
            State.SHUTDOWN_LOWER,
            State.SHUTDOWN_CURL,
            State.SHUTDOWN_DONE
        }

    def _transition_to_walking(self):
        """
        Transition from IDLE to WALKING.
        Ignored if already WALKING or a transition is in progress.
        """

        if self.state != State.IDLE:
            if self.logger: self.logger.warning(f"transition_to_walking() ignored - current state is {self.state.name}")
            return

        if self.logger: self.logger.info("Transitioning from IDLE to WALKING state")

        # Set legs to neutral stance
        # TODO set the target leg positions to the respective position at phase start
        self.target_leg_positions = self.gait.neutral_stance_positions

        # Enter IDLE_TO_WAKING state
        self.state = State.IDLE_TO_WALKING

    def _transition_to_idle(self):
        """
        Transition from WALKING to IDLE.
        Ignored if already IDLE or a transition is in progress.
        """

        if self.state != State.WALKING:
            if self.logger: self.logger.warning(f"transition_to_idle() ignored - current state is {self.state.name}")
            return

        if self.logger: self.logger.info("Transitioning from WALKING to IDLE state")

        self.gait.stop()

        # Enter WALKING_TO_IDLE state
        self.state = State.WALKING_TO_IDLE

    # Gait control

    def set_linear_velocity(self, vx: float, vy: float, vz: float = 0.0):
        """
        Set robot linear velocity.

        Args:
            vx: forward velocity (mm/s, positive = forward)
            vy: sideways velocity (mm/s, positive = right)
            vz: upwards velocity (mm/s, positive = up) - usually 0
        """

        if self.is_transition_ongoing():
            if self.logger: self.logger.warning(f"set_linear_velocity() ignored - current state is {self.state.name}")
            return

        max_linear = self.config['safety']['lin_vel_max']
        new_linear_velocity = np.array([
            np.clip(vx, -max_linear, max_linear),
            np.clip(vy, -max_linear, max_linear),
            np.clip(vz, -max_linear, max_linear)
        ])
        vel_norm = np.linalg.norm(new_linear_velocity)
        vel_change = False

        # If the robot was IDLE and no velocity is given, stay IDLE
        if self.state is State.IDLE and vel_norm < 1e-3:
            pass

        # If the robot was IDLE and velocity is given, transition to WALKING and then walk with new vel
        elif self.state is State.IDLE and vel_norm > 1e-3:
            self.linear_velocity = new_linear_velocity
            self._transition_to_walking()
            vel_change = True

        # If the robot was WALKING and velocity is given, stay WALKING
        elif self.state is State.WALKING and vel_norm > 1e-3:
            self.linear_velocity = new_linear_velocity
            vel_change = True

        # If the robot was WALKING and zero velocity is given:
        #       if angular velocity is == 0, go IDLE
        #       if angular velocity is != 0, continue WALKING
        elif self.state is State.WALKING and vel_norm < 1e-3:
            if abs(self.angular_velocity) < 1e-3:
                self._transition_to_idle()
            else:
                self.linear_velocity = new_linear_velocity
                vel_change = True

        if vel_change:
            if self.logger: self.logger.info(f"Setting gait linear velocity to: "
                             f"{new_linear_velocity[0]}, {new_linear_velocity[1]}, {new_linear_velocity[2]} mm/s")

    def set_angular_velocity(self, wz: float):
        """
        Set robot angular velocity.

        Args:
            wz: yaw rate (deg/s, positive = counter-clockwise)
        """

        if self.is_transition_ongoing():
            if self.logger: self.logger.warning(f"set_angular_velocity() ignored - current state is {self.state.name}")
            return

        max_angular = self.config['safety']['ang_vel_max']
        new_ang_vel = float(np.clip(wz, -max_angular, max_angular))
        vel_norm = abs(new_ang_vel)
        vel_change = False

        # If the robot was IDLE and no velocity is given, stay IDLE
        if self.state is State.IDLE and vel_norm < 1e-3:  # if IDLE, lin vel was already 0
            pass

        # If the robot was IDLE and velocity is given, transition to WALKING and then walk with new vel
        elif self.state is State.IDLE and vel_norm > 1e-3:
            self.angular_velocity = new_ang_vel
            self._transition_to_walking()
            vel_change = True

        # If the robot was WALKING and velocity is given, stay WALKING
        elif self.state is State.WALKING and vel_norm > 1e-3:
            self.angular_velocity = new_ang_vel
            vel_change = True

        # If the robot was WALKING and zero velocity is given:
        #       if linear velocity is == 0, go IDLE
        #       if linear velocity is != 0, continue WALKING
        elif self.state is State.WALKING and vel_norm < 1e-3:
            if np.linalg.norm(self.linear_velocity) < 1e-3:
                self._transition_to_idle()
            else:
                self.angular_velocity = new_ang_vel
                vel_change = True

        if vel_change:
            if self.logger: self.logger.info(f"Setting gait angular velocity to: {new_ang_vel} deg/s")

    def set_gait(self, gait_name: str):
        """Change gait pattern"""
        self.gait.set_gait(gait_name)

    def set_gait_parameter(self, param_name: str, value: float):
        """Adjust gait parameter in real-time"""
        self.gait.set_gait_parameter(param_name, value)

    # Body pose adjustments (IDLE and WALKING)

    def set_body_position(self, dx: float, dy: float, dz: float):
        """
        Set body position offset (general action).
        Small adjustments to body position during gait.

        Args:
            dx: Forward/backward shift (mm, positive = forward)
            dy: Left/right shift (mm, positive = right)
            dz: Up/down shift (mm, positive = up)
        """

        if self.is_transition_ongoing():
            if self.logger: self.logger.warning(f"set_body_position() ignored - current state is {self.state.name}")
            return

        # Limit position offset to prevent unreachable configurations
        x_range = self.config['safety'].get('x_range', (-50, 50))
        y_range = self.config['safety'].get('y_range', (-50, 50))
        z_range = self.config['safety'].get('z_range', (-50, 50))

        self.target_body_position[:] = [
            np.clip(dx, *x_range),
            np.clip(dy, *y_range),
            np.clip(dz, *z_range),
        ]

        if self.logger: self.logger.info(f"Setting body position to: {self.target_body_position[0]:.1f}, "
                         f"{self.target_body_position[1]:.1f}, {self.target_body_position[2]:.1f} mm")

    def set_body_orientation(self, roll: float, pitch: float, yaw: float):
        """
        Set body orientation (general action).
        Small rotations applied during gait.

        Args:
            roll: Body roll angle (degrees, positive = right side down)
            pitch: Body pitch angle (degrees, positive = nose down)
            yaw: Body yaw angle (degrees, positive = counter-clockwise)
        """

        if self.is_transition_ongoing():
            if self.logger: self.logger.warning(f"set_body_orientation() ignored - current state is {self.state.name}")
            return

        # Limit orientation to prevent unreachable configurations
        roll_range = self.config['safety'].get('roll_range', (-10, 10))
        pitch_range = self.config['safety'].get('pitch_range', (-10, 10))
        yaw_range = self.config['safety'].get('yaw_range', (-10, 10))

        self.target_body_orientation[:] = np.radians([
            np.clip(roll,  *roll_range),
            np.clip(pitch, *pitch_range),
            np.clip(yaw,   *yaw_range),
        ])

        if self.logger: self.logger.info(f"Setting body orientation to: {self.target_body_orientation[0]:.1f}, "
                         f"{self.target_body_orientation[1]:.1f}, {self.target_body_orientation[2]:.1f} deg")

    # Shutdown sequence

    def shutdown(self):
        """
        Begin the orderly shutdown sequence from IDLE.
        Ignored if a transition is already in progress or the robot is
        still in SETUP.  Call transition_to_idle() first if WALKING.
        """
        if self.state != State.IDLE:
            if self.logger: self.logger.warning(f"shutdown() ignored – current state is {self.state.name}")
            return

        if self.logger: self.logger.info("Starting shutdown sequence")

        # Start shutdown_lower: lower the body back to z = 0
        self.target_body_position = np.zeros(3)
        self.target_body_orientation = np.zeros(3)
        self.state = State.SHUTDOWN_LOWER

    def setup(self):
        """
        Begin the orderly setup sequence from IDLE.
        Ignored if a transition is already in progress or the robot is
        still in SHUTDOWN.  Call transition_to_idle() first if WALKING.
        """
        if self.state != State.SHUTDOWN_DONE:
            if self.logger: self.logger.warning(f"setup() ignored - current state is {self.state.name}")
            return

        if self.logger: self.logger.info("Starting setup sequence")

        # Reset odometry from the new power cycle
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.state = State.SETUP_RAISE

    # Leg position adjustments (IDLE)

    def set_leg_position(self, leg_name: str, x: float, y: float, z: float):
        """Set the target foot position for a single leg (mm, body frame)."""

        if self.state != State.IDLE:
            if self.logger: self.logger.warning(f"set_leg_position() ignored - current state is {self.state.name}")
            return

        if leg_name not in self.leg_names:
            if self.logger: self.logger.warning(f"set_leg_position() ignored - {leg_name} does not exist")
            return

        self.target_leg_positions[leg_name] = np.array([x, y, z])
        if self.logger: self.logger.info(f"Setting leg {leg_name} to position: {x:.1f}, {y:.1f}, {z:.1f} mm")

    def set_joint_angles(self, leg_name: str, coxa: float, femur: float, tibia: float):
        """Directly command joint angles (degrees) for a single leg."""

        if self.state != State.IDLE:
            if self.logger: self.logger.warning(f"set_joint_angles() ignored - current state is {self.state.name}")
            return

        if leg_name not in self.leg_names:
            if self.logger: self.logger.warning(f"set_joint_angles() ignored - {leg_name} does not exist")
            return

        angles_rad = np.radians([coxa, femur, tibia])
        foot_pos = self.kinematics.forward(leg_name, angles_rad)
        self.target_leg_positions[leg_name] = foot_pos
        if self.logger: self.logger.info(f"Setting leg {leg_name} to angles: {coxa:.1f}, {femur:.1f}, {tibia:.1f} deg")

    # Other stuff

    def emergency_stop(self):
        """Emergency stop - immediately halt all motion"""

        # Stop gait sequence
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = 0

        # Stop body pose interpolation
        self.body_linear_velocity = np.zeros(3)
        self.body_angular_velocity = np.zeros(3)

        # Set IDLE state
        self.state = State.IDLE

        # Turn off status led
        self.led.off()

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
            'leg_positions': {k: v.tolist() for k, v in self.leg_positions.items()},
            'joint_values': {k: v.tolist() for k, v in self.current_joints.items()},
            'odometry': {
                'x': self.odom_x,
                'y': self.odom_y,
                'yaw': self.odom_yaw
            },
            'state': self.state.name,
            'gait': self.gait.get_gait_info(),
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
        orientation_error = np.degrees(np.linalg.norm(self.target_body_orientation - self.body_orientation))

        return position_error < position_threshold and orientation_error < orientation_threshold

    def are_leg_positions_settled(self, position_threshold: float = 1) -> bool:
        """
        Check whether leg targets have reached the target.

        Args:
            position_threshold: Position error threshold (mm)
        """
        return all(
            np.linalg.norm(self.target_leg_positions[n] - self.leg_positions[n])
            < position_threshold
            for n in self.leg_names
        )