import numpy as np

from action import Action
from state import State

from hexapod import transformation_matrix, rotation_matrix


class Controller:

    def __init__(self, hexapod):

        self.hexapod = hexapod

        self.current_action = None
        self.action_queue = []
        self.elapsed = 0  # Time elapsed in the current interpolation

    # ----------------------------- Utility functions ---------------------------- #

    def step(self, dt):
        """
        Update the joint angles by interpolating between current and target states.

        Parameters:
            dt (float): Time step in seconds.
        Returns:
            np.ndarray: Updated joint angles in servo configuration.
        """

        if not self.current_action:
            # Get the next action if the current one is complete
            if self.action_queue:
                self.current_action = self.action_queue.pop(0)
                self.current_action.started = True
                self.elapsed = 0
            else:
                # No actions to execute
                return self.hexapod.get_joint_values()

        # Get the current target and duration
        target_state, duration = self.current_action.current_target()
        if target_state is None:
            # Action is complete
            self.current_action = None
            return self.hexapod.get_joint_values()

        # Update elapsed time
        self.elapsed += dt
        progress = min(self.elapsed / duration, 1)  # Clamp progress to [0, 1]

        # Interpolate between current and target state
        self.hexapod.state.interpolate(target_state, progress)

        if progress == 1:
            # Target reached, advance to the next step in the action
            # print(f'Action \'{self.current_action.name}\' terminated in time {round(self.elapsed, 2)}s')
            self.current_action.advance()
            self.elapsed = 0

        return self.hexapod.get_joint_values()

    def is_done(self):
        """
        Check if all actions in the queue are complete.
        """
        return not self.current_action and not self.action_queue

    def add_action(self, action):
        """
        Add an action to the queue. An action consists of a state and a duration.
        The robot will try to reach the desired state in the specified amount of time.

        Parameters:
            action (Action): An Action object to add to the queue.
        """
        self.action_queue.append(action)

    def get_last_state_in_queue(self):
        """
        Get the last state in the queue.

        Returns:
            State: The state of the last action to be executed.
        """

        # Get the last state of the last action of the queue
        last_state = self.hexapod.state
        if self.action_queue:
            last_state = self.action_queue[-1].states[-1]
        return last_state.copy()

    def get_points_in_origin_frame(self, points, indices, body_position=None, body_orientation=None):
        """
        Transforms points from leg frames to the origin frame.

        Parameters:
            points (np.ndarray): (N, 3) array of points expressed in leg frames.
            indices (int or list/np.ndarray):
                - If an integer (0 to 5), all points belong to this leg.
                - If a list/array, it specifies the leg index for each point.
            body_position (np.ndarray): [x, y, z] position of the body in the origin frame. Default is [0, 0, 0].
            body_orientation (np.ndarray): [roll, pitch, yaw] orientation of the body in the origin frame. Default is [0, 0, 0].

        Returns:
            np.ndarray: (N, 3) array of points in the origin frame.
        """
        points = np.asarray(points)
        assert ((points.ndim == 2 and points.shape[1] == 3) or
                (points.ndim == 1 and points.shape[0] == 3)), "Points must be an Nx3 array."

        # Handle body position and orientation
        if body_position is None:
            body_position = np.zeros(3)
        if body_orientation is None:
            body_orientation = np.zeros(3)

        # Create body transformation matrix
        body_frame = transformation_matrix(rotation_matrix(body_orientation), body_position)

        # Handle single leg index case
        if isinstance(indices, (int, np.integer)):
            if not 0 <= indices <= 5:
                raise ValueError("Leg index must be between 0 and 5")
            leg_frame = self.hexapod.leg_frames[indices]
            # Transform all points using the same leg frame
            homogeneous_points = np.hstack([points, np.ones((len(points), 1))])
            transformed_points = body_frame @ leg_frame @ homogeneous_points
            return transformed_points[:3]

        # Handle multiple leg indices case
        indices = np.asarray(indices)
        assert indices.shape == (len(points),), "Number of leg indices must match number of points"
        assert all(0 <= idx < 6 for idx in indices), "All leg indices must be between 0 and 5."

        # Transform each point using its corresponding leg frame
        transformed_points = []
        for point, leg_idx in zip(points, indices):
            leg_frame = self.hexapod.leg_frames[leg_idx]
            homogeneous_point = np.array([*point, 1])
            transformed_point = body_frame @ leg_frame @ homogeneous_point
            transformed_points.append(transformed_point[:3])

        return np.array(transformed_points)

    # ---------------------------------- Actions --------------------------------- #

    def stand(self, duration, height=100, y_offset=120):
        """
        Make the robot stand. The height and distances are used to describe points
        in leg frames. These distances are the same for each leg. The robot is
        supposed to be on the ground, thus body orientation and position
        are (0, 0, 0), (0, 0, 0).

        Parameters:
            height (float): Height at which the robot should stand.
            y_offset (float): Radial distance from the robot's body.
            duration (float): Time in seconds to interpolate to the target angles.
        """

        assert duration > 0, f"The duration cannot be < 0."

        body_position_extend_phase = np.zeros(3)
        body_orientation_extend_phase = np.zeros(3)
        legs_positions_extend_phase = self.hexapod.translate_to_origin_frame(
            np.array([[y_offset, 0, 0] for _ in range(6)])
        )
        joint_values_extend_phase = self.hexapod.inverse_kinematics_origin_frame(
            legs_positions_extend_phase,
            body_position_extend_phase,
            body_orientation_extend_phase
        )

        extend_phase = State(
            legs_positions_extend_phase,
            body_position_extend_phase,
            body_orientation_extend_phase,
            joint_values_extend_phase
        )

        body_position_lift_phase = np.array([0, 0, height])
        body_orientation_lift_phase = np.zeros(3)
        legs_positions_lift_phase = self.hexapod.translate_to_origin_frame(
            np.array([[y_offset, 0, 0] for _ in range(6)])
        )
        joint_values_lift_phase = self.hexapod.inverse_kinematics_origin_frame(
            legs_positions_lift_phase,
            body_position_lift_phase,
            body_orientation_lift_phase
        )

        lift_phase = State(
            legs_positions_lift_phase,
            body_position_lift_phase,
            body_orientation_lift_phase,
            joint_values_lift_phase
        )

        stand_action = Action(
            states=[
                extend_phase,
                lift_phase
            ],
            durations=[
                duration / 2,
                duration / 2
            ],
            name='stand'
        )
        self.add_action(stand_action)

    def wait(self, duration):
        """
        Make the robot wait keeping the current configuration for the given amount of time.

        Parameters:
            duration (float): Time in seconds to interpolate to the target angles.
        """

        assert duration > 0, f"The duration cannot be < 0."

        wait_action = Action(
            states=[
                self.get_last_state_in_queue()
            ],
            durations=[duration],
            name='wait'
        )
        self.add_action(wait_action)

    def set_body_pose(self, duration, body_position=None, body_orientation=None):
        """
        Set the body pose.

        Parameters:
            duration (float): Time in seconds to interpolate to the target angles.
            body_position (np.ndarray): [x, y, z] position of the body in the world frame.
                Defaults to the value in the last state, if any.
            body_orientation (np.ndarray): [roll, pitch, yaw] orientation of the body in the world frame.
                Defaults to the value in the last state, if any.
        """

        assert duration > 0, f"The duration cannot be < 0."
        if body_position is not None:
            assert body_position.shape == (3,), f"Invalid body position: {body_position}."
        if body_orientation is not None:
            assert body_orientation.shape == (3,), f"Invalid body orientation: {body_position}."


        # Get the last state of the last action of the queue
        current_state = self.get_last_state_in_queue()

        if body_position is not None:
            current_state.body_position = body_position

        if body_orientation is not None:
            current_state.body_orientation = body_orientation

        current_state.joint_angles = self.hexapod.inverse_kinematics_origin_frame(
            current_state.legs_positions,
            current_state.body_position,
            current_state.body_orientation
        )

        reach_action = Action(
            states=[
                current_state
            ],
            durations=[duration],
            name='set_body_pose'
        )
        self.add_action(reach_action)

    def set_legs_positions(self, duration, legs_positions, indices=None, leg_frame=False):
        """
        Set the legs positions, specified in origin frame. You can either set the
        positions for all the legs, in which case the size of the legs_positions
        array should be (6, 3), or of a subset of them, in which case the array
        should be smaller and the indices array should be provided.

        Parameters:
            duration (float): Time in seconds to interpolate to the target angles.
            legs_positions (np.ndarray): End-effector positions origin frame.
                If no indices are provided, the size of the array should be
                (6, 3). If the indices array is provided the array can be smaller.
            indices (list): Indices of the legs for which we are specifying the
                target position.
            leg_frame (bool): True if the positions are specified in leg frame,
                False otherwise.
        """

        assert duration > 0, f"The duration cannot be < 0."

        # Get the last state of the last action of the queue
        last_state = self.get_last_state_in_queue()
        updated_legs_positions = last_state.legs_positions.copy()

        if legs_positions is not None:
            legs_positions = np.atleast_2d(legs_positions)
            assert legs_positions.shape[1] == 3, "Invalid leg positions specifier."

            if indices is None:
                indices = range(len(legs_positions))  # Auto-set indices if missing
            else:
                assert len(indices) == legs_positions.shape[0], "Indices length must match legs_positions count."
                assert all(0 <= idx < 6 for idx in indices), "Indices must be in range 0-5."

            # Translate points into the respective frames if needed
            if leg_frame:
                legs_positions = self.get_points_in_origin_frame(
                    legs_positions,
                    indices=indices,
                    body_position=last_state.body_position,
                    body_orientation=last_state.body_orientation
                )

            # Update only specified legs
            for i, idx in enumerate(indices):
                    updated_legs_positions[idx] = legs_positions[i]

        last_state.joint_angles = self.hexapod.inverse_kinematics_origin_frame(
            updated_legs_positions,
            last_state.body_position,
            last_state.body_orientation
        )

        reach_action = Action(
            states=[
                last_state
            ],
            durations=[
                duration
            ],
            name='set_legs_positions'
        )
        self.add_action(reach_action)

    def set_body_pose_and_legs_positions(self,
                                         duration,
                                         body_position=None,
                                         body_orientation=None,
                                         legs_positions=None,
                                         indices=None
        ):
        """
        Reach a target configuration (body pose + legs positions). The legs will move directly
        to the target position.

        Parameters:
            duration (float): Time in seconds to interpolate to the target angles.
            body_position (np.ndarray): [x, y, z] position of the body in the world frame.
                Defaults to the value in the last state, if any.
            body_orientation (np.ndarray): [roll, pitch, yaw] orientation of the body in the world frame.
                Defaults to the value in the last state, if any.
            legs_positions (np.ndarray): End-effector positions origin frame.
                If no indices are provided, the size of the array should be
                (6, 3). If the indices array is provided the array can be smaller.
            indices (list): Indices of the legs for which we are specifying the
                target position.
        """

        assert duration > 0, f"The duration cannot be < 0."

        # Get the last state of the last action of the queue
        last_state = self.get_last_state_in_queue()
        updated_legs_positions = last_state.legs_positions.copy()

        if legs_positions is not None:
            legs_positions = np.atleast_2d(legs_positions)
            assert legs_positions.shape[1] == 3, "Invalid leg positions specifier."

            if indices is None:
                indices = range(len(legs_positions))  # Auto-set indices if missing
            else:
                assert len(indices) == legs_positions.shape[0], "Indices length must match legs_positions count."
                assert all(0 <= idx < 6 for idx in indices), "Indices must be in range 0-5."

            # Update only specified legs
            for i, idx in enumerate(indices):
                updated_legs_positions[idx] = legs_positions[i]

        if body_position is not None:
            last_state.body_position = body_position

        if body_orientation is not None:
            last_state.body_orientation = body_orientation

        last_state.joint_angles = self.hexapod.inverse_kinematics_origin_frame(
            last_state.legs_positions,
            last_state.body_position,
            last_state.body_orientation
        )

        reach_action = Action(
            states=[
                last_state
            ],
            durations=[
                duration
            ],
            name='set_body_pose_and_leg_positions'
        )
        self.add_action(reach_action)

    def sit(self, duration, height=30, y_offset=50):
        """
        Make the robot sit, retracting the legs near their maximum. In this case
        we don't know the body position and orientation so we will perform
        the movement in three phases:
        1. a reset phase, where we reset the body orientation to (0, 0, 0)
        2. a lowering phase, where we bring the body to the ground by setting the body position to (0, 0, 0)
        3. a retract phase, where we retract the legs as much as we can

        Parameters:
            duration (float): Time in seconds to interpolate to the target angles.
        """

        assert duration > 0, f"The duration cannot be < 0."

        # Get the last state of the last action of the queue
        current_state = self.get_last_state_in_queue()

        # Reset phase
        reset_joint_values = self.hexapod.inverse_kinematics_origin_frame(
            current_state.legs_positions,
            current_state.body_position,
            np.zeros(3)
        )

        reset_phase = State(
            current_state.legs_positions,
            current_state.body_position,
            np.zeros(3),
            reset_joint_values
        )

        # Lowering phase
        lowering_joint_values = self.hexapod.inverse_kinematics_origin_frame(
            current_state.legs_positions,
            np.zeros(3),
            np.zeros(3)
        )

        lowering_phase = State(
            current_state.legs_positions,
            np.zeros(3),
            np.zeros(3),
            lowering_joint_values
        )

        # Retract phase
        retract_legs_positions = self.hexapod.translate_to_origin_frame(
            np.array([[y_offset, 0, height] for _ in range(6)])
        )
        retract_joint_values = self.hexapod.inverse_kinematics_origin_frame(
            retract_legs_positions,
            np.zeros(3),
            np.zeros(3)
        )

        retract_phase = State(
            retract_legs_positions,
            np.zeros(3),
            np.zeros(3),
            retract_joint_values
        )

        sit_action = Action(
            states=[
                reset_phase,
                lowering_phase,
                retract_phase
            ],
            durations=[
                duration / 3,
                duration / 3,
                duration / 3
            ],
            name='sit'
        )
        self.add_action(sit_action)
