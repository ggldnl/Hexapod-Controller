from interpolator import Interpolator
import math


class Leg:
    """
    The Leg class represents a single leg of the hexapod robot, consisting of three links: coxa, femur, and tibia.
    Each link is driven by a servo. The servos are controlled by an Interpolator that produces smooth motion.
    """

    def __init__(self, config, h=60):

        # Store configuration details for each link
        self.servo_config = config

        # Extract the total length of the leg
        femur_link_len = self.servo_config['femur']['length']
        tibia_link_len = self.servo_config['tibia']['length']
        self.total_len = femur_link_len + tibia_link_len

        self.coxa = Interpolator()
        self.femur = Interpolator()
        self.tibia = Interpolator()

        self.h = h


    @staticmethod
    def _distance(position):
        """
        Euclidean distance from the leg's origin to the provided point.
        """
        return math.sqrt(position[0] ** 2 + position[1] ** 2 + position[2] ** 2)

    def _inverse_kinematics(self, position):
        """
        Computes the inverse kinematics solution to reach the point.
        """

        x, y, z = position

        f = self.servo_config['femur']['length']
        t = self.servo_config['tibia']['length']
        h = self.h

        alpha = math.atan2(y, x)

        d = math.sqrt(y ** 2 + (h - z) ** 2)
        b1 = math.asin(y / d)

        gamma = math.acos((t ** 2 + f ** 2 - d ** 2) / (2 * f * t))

        b2 = math.acos((f ** 2 + d ** 2 - t ** 2) / (2 * f * d))
        beta = b1 + b2

        return alpha, beta, gamma

    @staticmethod
    def _map(value, in_min, in_max, out_min, out_max):
        """
        Map the value from the input range to the output range.
        """
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def set_target(self, target_position):
        """
        Set the target angle to each servo interpolator to reach the required position.
        The method first checks if the point falls in the reachable workspace of the leg's
        kinematic skeleton; if so, computes the inverse kinematic solution and then checks
        that no angle violates the joint limits. It can happen that we have a solution with
        but joint limits on the actual structure prevent us to reach it. The method returns
        False if we cannot reach the point for whatever reason (point out of the workspace
        or violated joint limits) and True otherwise.
        """

        if self._distance(target_position) > self.total_len:
            return False

        # Dummy inverse kinematics calculation (replace with actual IK calculation)
        coxa_angle, femur_angle, tibia_angle = self._inverse_kinematics(target_position)
        coxa_angle_deg = math.degrees(coxa_angle)
        femur_angle_deg = math.degrees(femur_angle)
        tibia_angle_deg = math.degrees(tibia_angle)

        print(f'Angles: {coxa_angle_deg}, {femur_angle_deg}, {tibia_angle_deg} -> ', end='')

        # Translate in servo space:
        coxa_angle_deg = self._map(coxa_angle_deg, 0, 180, -90, 90)
        femur_angle_deg = self._map(femur_angle_deg, 0, 180, -90, 90)
        tibia_angle_deg = self._map(tibia_angle_deg, 0, 180, 90, -90)

        print(f'{coxa_angle_deg}, {femur_angle_deg}, {tibia_angle_deg} -> ', end='')

        # We have a sequence of angles that will result in the tip of the leg on the target point.
        # Check that no angle violates the limits:

        coxa_min_range = self.servo_config['coxa']['min_range']
        coxa_max_range = self.servo_config['coxa']['max_range']
        if not coxa_min_range <= coxa_angle_deg <= coxa_max_range:
            return False

        femur_min_range = self.servo_config['femur']['min_range']
        femur_max_range = self.servo_config['femur']['max_range']
        if not femur_min_range <= femur_angle_deg <= femur_max_range:
            return False

        tibia_min_range = self.servo_config['tibia']['min_range']
        tibia_max_range = self.servo_config['tibia']['max_range']
        if not tibia_min_range <= tibia_angle_deg <= tibia_max_range:
            return False

        print(f'Limits enforced.')

        # Set the target angles for each interpolator
        self.coxa.set_target(coxa_angle_deg)
        self.femur.set_target(femur_angle_deg)
        self.tibia.set_target(tibia_angle_deg)

        return True

    def is_at_target(self):
        return self.coxa.is_at_target() and self.femur.is_at_target() and self.tibia.is_at_target()

    def set_speed(self, speed):
        self.coxa.set_speed(speed)
        self.femur.set_speed(speed)
        self.tibia.set_speed(speed)

    def update(self):
        """
        Update the angles for the coxa, femur, and tibia servos using their respective interpolators.
        Returns the updated angles for each servo.
        """
        coxa_angle = self.coxa.update()
        femur_angle = self.femur.update()
        tibia_angle = self.tibia.update()

        return coxa_angle, femur_angle, tibia_angle


# Example usage
if __name__ == '__main__':

    from controller import Controller
    import json
    import time

    # Load config.json from file
    with open('config/config.json', 'r') as f:
        config = json.load(f)

        # Take the config dict
        leg_config = config["leg_6"]

    # Create a leg instance
    leg = Leg(leg_config)

    # Create the controller
    controller = Controller()

    # Take the pins to send the command
    pins = [leg_config[link]['pin'] for link in ['coxa', 'femur', 'tibia']]

    # Open the serial communication
    with controller:

        controller.attach_servos()
        time.sleep(1)

        leg.set_target((50, 50, 20))  # Set target angles for the leg's servos
        leg.set_speed(0.6)

        while not leg.is_at_target():
            angles = leg.update()
            print(f'Angles: {angles[0]}, {angles[1]}, {angles[2]}')

            for pin, angle in zip(pins, angles):
                controller.set_angle(pin, angle)

            time.sleep(0.05)

        # Release the servos
        time.sleep(2)
        controller.detach_servos()

        print("Done.")
