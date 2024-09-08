import math
import time


class Leg:
    def __init__(self, controller, config):
        self.controller = controller
        # Configuration for each joint's servo (hip, knee, ankle)
        self.servos = {
            'hip': config['hip'],
            'knee': config['knee'],
            'ankle': config['ankle']
        }

        # Current servo positions
        self.current_positions = {'hip': 0.0, 'knee': 0.0, 'ankle': 0.0}

        # Target positions (desired angles)
        self.target_positions = {'hip': 0.0, 'knee': 0.0, 'ankle': 0.0}

        # Servo speed factor
        self.speed = 1.0  # A factor that controls speed of the interpolation

    def compute_inverse_kinematics(self, x, y, z):
        """
        Compute the necessary angles for the hip, knee, and ankle servos
        based on the leg's desired foot position (x, y, z).
        """
        # Placeholder example for inverse kinematics
        hip_angle = math.atan2(y, x)  # Example for computing hip angle
        knee_angle = math.atan2(z, math.sqrt(x ** 2 + y ** 2))  # Example for knee
        ankle_angle = -knee_angle / 2  # Example ankle computation

        # Update target positions
        self.target_positions['hip'] = self.clamp_angle(hip_angle, 'hip')
        self.target_positions['knee'] = self.clamp_angle(knee_angle, 'knee')
        self.target_positions['ankle'] = self.clamp_angle(ankle_angle, 'ankle')

    def clamp_angle(self, angle, joint):
        """
        Clamp the calculated angle between the servo's minimum and maximum limits.
        """
        min_angle = self.servos[joint]['min_angle']
        max_angle = self.servos[joint]['max_angle']
        return max(min(angle, max_angle), min_angle)

    def interpolate_movement(self):
        """
        Smoothly move from the current servo position to the target position.
        Interpolates each servo movement at the specified speed.
        """
        for joint in self.servos:
            current = self.current_positions[joint]
            target = self.target_positions[joint]

            if current != target:
                step = (target - current) * self.speed  # Simple interpolation
                new_position = current + step
                self.current_positions[joint] = new_position
                self.set_servo_position(joint, new_position)

    def set_servo_position(self, joint, angle):
        """
        Set the pulse width for the servo corresponding to the specified angle.
        """
        servo = self.servos[joint]
        pulse_width = self.angle_to_pulse(servo, angle)
        self.controller.set_pulse(servo['pin'], pulse_width)

    def angle_to_pulse(self, servo, angle):
        """
        Convert a given angle to the corresponding pulse width based on the servo's range.
        """
        min_pulse = servo['min_pulse']
        max_pulse = servo['max_pulse']
        min_angle = servo['min_angle']
        max_angle = servo['max_angle']

        # Linear mapping of angle to pulse width
        pulse_width = (angle - min_angle) / (max_angle - min_angle) * (max_pulse - min_pulse) + min_pulse
        return pulse_width

    def update(self):
        """
        Call this function regularly in a loop to continuously interpolate and move the servos.
        """
        self.interpolate_movement()


# Example usage

if __name__ == '__main__':
    config = {
        'hip': {'pin': 0, 'min_pulse': 500, 'max_pulse': 2500, 'min_angle': -90, 'max_angle': 90},
        'knee': {'pin': 1, 'min_pulse': 500, 'max_pulse': 2500, 'min_angle': 0, 'max_angle': 135},
        'ankle': {'pin': 2, 'min_pulse': 500, 'max_pulse': 2500, 'min_angle': -45, 'max_angle': 45}
    }

    controller = Controller()
    with controller:
        leg = Leg(controller, config)
        leg.compute_inverse_kinematics(100, 50, -100)  # Example target position
        while True:
            leg.update()
            time.sleep(0.02)  # Call update at regular intervals
