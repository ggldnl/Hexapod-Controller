import numpy as np

from action import Action
from state import State


class Controller:

    def __init__(self, hexapod):

        self.hexapod = hexapod

        self.current_action = None
        self.action_queue = []
        self.elapsed = 0  # Time elapsed in the current interpolation
        # self.velocity = np.array([0, 0])  # x, y velocity for gait sequence

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

    # ---------------------------------- Actions --------------------------------- #

    def get_state(self, legs_positions, body_position=None, body_orientation=None, targets_in_leg_frames=True):

        joint_angles = self.hexapod.inverse_kinematics(legs_positions, body_position, body_orientation, targets_in_leg_frames)

        state = State(
            body_position=body_position,
            body_orientation=body_orientation,
            legs_positions=legs_positions,
            joint_angles=joint_angles
        )

        if all(item is None for leg_angles in joint_angles for item in leg_angles):
            state = self.hexapod.state

        return state

    def stand(self, duration, height=100, y_offset=120):
        """
        Make the robot stand. The height and distances are used to describe points
        in leg frames. These distances are the same for each leg.

        Parameters:
            height (float): Height at which the robot should stand.
            y_offset (float): Radial distance from the robot's body.
            duration (float): Time in seconds to interpolate to the target angles.
        """

        stand_action = Action(
            states=[

                # Extend the leg
                self.get_state([[y_offset, 0, 0] for _ in range(6)]),

                # Full lift
                self.get_state([[y_offset, 0, -height] for _ in range(6)])
            ],
            durations=[duration, duration]
        )
        self.add_action(stand_action)

    def wait(self, duration):
        """
        Make the robot wait keeping the current configuration for the given amount of time.

        Parameters:
            duration (float): Time in seconds to interpolate to the target angles.
        """

        stand_action = Action(
            states=[
                # self.get_state([leg.joint_angles for leg in self.hexapod])
                self.hexapod.state
            ],
            durations=[duration]
        )
        self.add_action(stand_action)
