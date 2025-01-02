from abc import abstractmethod, ABC
import numpy as np

from kinematics import HexapodModel
from kinematics import transformation_matrix, rotation_matrix
from kinematics import LegModel
from state import State


class RobotInterface(HexapodModel, ABC):
    """
    Wrapper to the kinematic model that also Keeps an internal state
    (body position, body orientation and legs positions) and handles
    exceptions.
    """

    def __init__(self, config, fail_silently=True):

        # Misc
        self.config = config
        self.fail_silently = fail_silently

        body_config = self.config["body"]

        legs = []
        current_angles = []
        for i, leg in enumerate(self.config["legs"]):

            current_coxa_angle = self.config["legs"][leg]["coxa"]["cur_angle"]
            current_femur_angle = self.config["legs"][leg]["femur"]["cur_angle"]
            current_tibia_angle = self.config["legs"][leg]["tibia"]["cur_angle"]
            current_angles.append([current_coxa_angle, current_femur_angle, current_tibia_angle])

            coxa_length = self.config["legs"][leg]["coxa"]["length"]
            femur_length = self.config["legs"][leg]["femur"]["length"]
            tibia_length = self.config["legs"][leg]["tibia"]["length"]

            position = self.config["legs"][leg]["frame"]["position"]
            orientation = self.config["legs"][leg]["frame"]["orientation"]
            leg_frame = transformation_matrix(rotation_matrix(orientation), position)

            legs.append(LegModel(coxa_length, femur_length, tibia_length, leg_frame))

        super().__init__(legs=legs)

        # Robot state
        body_position = body_config["frame"]["position"]
        body_orientation = body_config["frame"]["orientation"]
        legs_positions = self.forward_kinematics(current_angles, body_position, body_orientation)

        # Robot state
        self.state = State(
            body_position,
            body_orientation,
            legs_positions,
            current_angles
        )

    def inverse_kinematics(self, legs_positions, body_position=None, body_orientation=None, targets_in_body_frame=True):
        try:
            return self.inverse_kinematics(legs_positions, body_position, body_orientation, targets_in_body_frame)
        except ValueError as e:
            if self.fail_silently:
                print('Failed to reach state: {e}')
                return [[None] * 3] * 6
            else:
                raise e

    @abstractmethod
    def get_joint_values(self):
        pass


class VirtualRobot(RobotInterface):
    """
    Represents the robot in a simulation environment. The robot is described in its parts with a URDF file
    but this has different joint ranges with respect both to the kinematic model and the actual robot.
    This class applies a transformation to the computed joint values to mimic the real movement.
    """

    def __init__(self, config, fail_silently=True):
        super().__init__(config, fail_silently=fail_silently)

    def get_joint_values(self):
        translated_angles = self.translate(self.state.joint_angles)
        return translated_angles

    @staticmethod
    def map_range(value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def translate(self, angles):
        """
        Transform the angles in way that the robot moves consistently in a simulated environment.

        Parameters:
            angles (list[list[float]]): Joint angles.
        """
        translated_angles = angles.copy()

        # Translate angles for each leg
        for i, leg_angles in enumerate(translated_angles):
            leg_angles = [
                leg_angles[0],  # Coxa angle is the same
                self.map_range(leg_angles[1], np.pi / 2, -np.pi / 2, 0, np.pi),
                self.map_range(leg_angles[2], -np.pi / 2, np.pi / 2, 0, -np.pi)
            ]
            translated_angles[i] = leg_angles

        # Apply offsets to the angles
        offset = np.deg2rad(25)
        for i, leg_angles in enumerate(translated_angles):
            leg_angles[1] -= offset
            leg_angles[2] += offset

        # Mirror left legs
        for i, leg_angles in enumerate(translated_angles):
            if i > 2:
                leg_angles[1] *= -1

        return translated_angles


class Robot(RobotInterface):
    """
    Represents the real robot. This means we have to translate the joint angles
    computed by the kinematic model into the servo space, check if the conversion
    is valid (the angles do not violate physical constraints) and translate the
    angles as pulse widths.
    """

    def __init__(self, config, fail_silently=True):
        super().__init__(config, fail_silently=fail_silently)

        # Servo ranges
        self.min_angles = []
        self.max_angles = []
        self.min_pulses = []
        self.max_pulses = []

        for i, leg in enumerate(self.config["legs"]):
            coxa_min_angle = self.config["legs"][leg]["coxa"]["min_angle"]
            femur_min_angle = self.config["legs"][leg]["femur"]["min_angle"]
            tibia_min_angle = self.config["legs"][leg]["tibia"]["min_angle"]
            self.min_angles.append([coxa_min_angle, femur_min_angle, tibia_min_angle])

            coxa_max_angle = self.config["legs"][leg]["coxa"]["max_angle"]
            femur_max_angle = self.config["legs"][leg]["femur"]["max_angle"]
            tibia_max_angle = self.config["legs"][leg]["tibia"]["max_angle"]
            self.max_angles.append([coxa_max_angle, femur_max_angle, tibia_max_angle])

            coxa_min_pulse = self.config["legs"][leg]["coxa"]["min_pulse"]
            femur_min_pulse = self.config["legs"][leg]["femur"]["min_pulse"]
            tibia_min_pulse = self.config["legs"][leg]["tibia"]["min_pulse"]
            self.min_pulses.append([coxa_min_pulse, femur_min_pulse, tibia_min_pulse])

            coxa_max_pulse = self.config["legs"][leg]["coxa"]["max_pulse"]
            femur_max_pulse = self.config["legs"][leg]["femur"]["max_pulse"]
            tibia_max_pulse = self.config["legs"][leg]["tibia"]["max_pulse"]
            self.max_pulses.append([coxa_max_pulse, femur_max_pulse, tibia_max_pulse])

    def get_joint_values(self):
        translated_angles = self.translate(self.state.joint_angles)
        translated_angles = self.check(translated_angles)
        pulses = self.to_pulse(translated_angles)
        return pulses

    @staticmethod
    def map_range(value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    @classmethod
    def angle_to_pulse(cls, angle, min_pulse, max_pulse):
        # return int(round(cls.map_range(angle, -np.pi/2, np.pi/2, min_pulse, max_pulse)))
        # TODO fix this
        return angle

    def translate(self, angles):
        """
        Translate the angles into the servo space.

        Parameters:
            angles (list[list[float]]): Joint angles.
        """
        translated_angles = angles.copy()

        # Translate angles for each leg
        for i, leg_angles in enumerate(translated_angles):
            leg_angles = [
                leg_angles[0],  # Coxa angle is the same
                self.map_range(leg_angles[1], np.pi / 2, -np.pi / 2, 0, np.pi),
                self.map_range(leg_angles[2], -np.pi / 2, np.pi / 2, 0, -np.pi)
            ]
            translated_angles[i] = leg_angles

        # Apply offsets to the angles
        offset = np.deg2rad(25)
        for i, leg_angles in enumerate(translated_angles):
            leg_angles[1] -= offset
            leg_angles[2] += offset

        return translated_angles

    def check(self, angles):
        """
        Check if the angles are all in the valid range.

        Parameters:
            angles (list[list[float]]): Joint angles.
        """

        for leg_angles, min_leg_angles, max_leg_angles in zip(angles, self.min_angles, self.max_angles):
            for angle, min_angle, max_angle in zip(leg_angles, min_leg_angles, max_leg_angles):
                if not min_angle <= angle <= max_angle:
                    if not self.fail_silently:
                        raise ValueError(f'Angle {angle} out of range ({min_angle}, {max_angle})')
                    else:
                        return [[None] * 3] * 6

        return angles

    def to_pulse(self, angles):
        """
        Converts the angles to pulse widths.

        Parameters:
            angles (list[list[float]]): Joint angles.
        """
        joint_pulses = []
        for leg_angles, min_leg_pulses, max_leg_pulses in zip(angles, self.min_pulses, self.max_pulses):
            leg_pulses = []
            for angle, min_pulse, max_pulse in zip(leg_angles, min_leg_pulses, max_leg_pulses):
                leg_pulses.append(self.angle_to_pulse(angle, min_pulse, max_pulse))
            joint_pulses.append(leg_pulses)
        return joint_pulses


if __name__ == '__main__':

    import argparse
    import json

    parser = argparse.ArgumentParser(description="Code to be run on the Hexapod")
    parser.add_argument("-c", "--config", type=str, default='config/config.json',
                        help="Path to the robot's configuration file")
    parser.add_argument('-n', '--name', type=str, default='hexapod', help="Name of the robot in the config")

    args = parser.parse_args()

    # Read the JSON
    with open(args.config) as f:
        config = json.load(f)

    # Create a Hexapod object
    hexapod = RobotInterface(config[args.name], fail_silently=False)

    """
    angles = [45, 0, -45, -135, 180, 135]
    angles = [np.deg2rad(a) for a in angles]
    radius = 150
    points = np.array([
        [round(radius * np.cos(angle), 2), round(radius * np.sin(angle), 2), 0]  # Convert polar to Cartesian
        for angle in angles
    ])

    hexapod.set_state(
        np.array(points),
        [0, 0, 60],
        [0, 0, np.deg2rad(10)],
        targets_in_leg_frames=False
    )
    """
