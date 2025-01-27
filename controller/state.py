class State:

    def __init__(self, legs_positions, body_position, body_orientation, joint_angles):
        """
        Represents the state of the robot in a given instant. We will compute the state of the
        robot in some key points and interpolate between them. This will let us interpolate
        in joint space, thus with less computational cost.

        Parameters:
            body_position (np.ndarray): Body position in origin frame.
            body_orientation (np.ndarray): Body orientation in origin frame.
            legs_positions (np.ndarray): Position of the end effectors of each leg in origin frame.
            joint_angles (np.ndarray): Joint angles that realize the state (kinematic model).
        """

        self.body_position = body_position
        self.body_orientation = body_orientation
        self.legs_positions = legs_positions
        self.joint_angles = joint_angles

    def interpolate(self, target_state, progress):
        """
        Interpolate between the current state and another one.

        Parameters:
            target_state (State): The new state.
            progress (float): Interpolation progress (float in range [0, 1]).
        """

        self.joint_angles = (
                (1 - progress) * self.joint_angles + progress * target_state.joint_angles
        )

    def copy(self):
        """
        Creates a copy of the current state.

        Returns:
            State: A new instance of State with identical values.
        """
        return State(
            self.legs_positions.copy(),
            self.body_position.copy(),
            self.body_orientation.copy(),
            self.joint_angles.copy()
        )