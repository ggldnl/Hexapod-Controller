"""
Interfaces with kernel to control servos and read sensors.
Handles servo mapping, calibration, and safety limits.
"""

import numpy as np


class Interface:
    """Hardware abstraction layer for robot control."""
    
    def __init__(self, kernel, config: dict):
        self.kernel = kernel
        self.config = config

        # Fix leg names order
        self.leg_names = [k for k, v in config['kinematics']['legs'].items() if isinstance(v, dict)]

        # Hardware info
        self.pin = {leg_name: config['hardware']['pins'][leg_name] for leg_name in self.leg_names}
        self.trim = {leg_name: config['hardware']['trim'][leg_name] for leg_name in self.leg_names}
        self.direction = {leg_name: config['hardware']['direction'][leg_name] for leg_name in self.leg_names}

        # Limits
        coxa_min, coxa_max = config['safety']['coxa_range']
        femur_min, femur_max = config['safety']['femur_range']
        tibia_min, tibia_max = config['safety']['tibia_range']
        self.servo_min = {leg_name: [coxa_min, femur_min, tibia_min] for leg_name in self.leg_names}
        self.servo_max = {leg_name: [coxa_max, femur_max, tibia_max] for leg_name in self.leg_names}

        # Safety limits
        self.voltage_min = self.config['safety']['voltage_min']
        self.current_max = self.config['safety']['current_max']

        self.enabled = True

    def enable(self):
        """Enable all servos."""
        self.kernel.attach_servos()  # Enables the servos
        self.kernel.connect_power()  # Physically connects the power trace to the servos
        self.enabled = True
    
    def disable(self):
        """Disable all servos."""
        self.kernel.detach_servos()
        self.kernel.disconnect_power()
        self.enabled = False
    
    def kinematic_space_to_servo_space(self, leg: str, joint: int, angle: float) -> float:
        """Convert angle from kinematic space to servo space using config specification."""

        angle *= self.direction[leg][joint]
        angle += self.trim[leg][joint]
        servo_min = self.servo_min[leg][joint]
        servo_max = self.servo_max[leg][joint]
        return np.clip(angle, servo_min, servo_max)

    def servo_space_to_kinematic_space(self, leg: str, joint: int, angle: float) -> float:
        """Convert angle from servo space back to kinematic space."""

        angle -= self.trim[leg][joint]
        angle /= self.direction[leg][joint]
        return angle

    def set_joint(self, leg: str, joint: int, value: float) -> bool:
        """Set single joint angle (degrees)."""

        # Map kinematic angle to servo angle
        new_angle = self.kinematic_space_to_servo_space(leg, joint, value)
        pin = self.pin[leg][joint]
        result = self.kernel.set_servo_angle(pin, new_angle)

        return result

    def set_leg(self, leg: str, angles: list) -> bool:
        """Set leg angles (degrees)."""

        # Map angles in kinematic space to servo space
        new_angles = [self.kinematic_space_to_servo_space(leg, i, angles[i]) for i in range(len(angles))]
        pins = [self.pin[leg][i] for i in range(len(angles))]
        result = self.kernel.set_servo_angles((pin, angle) for pin, angle in zip(pins, new_angles))

        return result

    def set_all_legs(self, joint_values: dict) -> bool:
        """Set angles (degrees) for all legs simultaneously."""

        all_pins = []
        all_angles = []
        for leg in self.leg_names:

            # Map angles
            angles = joint_values[leg]
            new_angles = [self.kinematic_space_to_servo_space(leg, i, angles[i]) for i in range(len(angles))]
            pins = [self.pin[leg][i] for i in range(len(angles))]

            all_pins.extend(pins)
            all_angles.extend(new_angles)

        # Bulk update
        values = [(pin, angle) for pin, angle in zip(all_pins, all_angles)]
        return self.kernel.set_servo_angles(values)

    def get_joint(self, leg: str, joint: int) -> float:
        """Get the current angle of a joint (joint space)."""
        pin = self.pin[leg][joint]
        return self.kernel.get_servo_angle(pin)

    def get_leg(self, leg: str) -> tuple:
        """Get the current angles of a leg (joint space)."""
        pins = [self.pin[leg][joint] for joint in range(3)]
        return self.kernel.get_servo_angles(pins)

    def get_all_legs(self) -> dict:
        """Get the current angles for all legs (joint space)."""
        return {
            leg_name: self.get_leg(leg_name) for leg_name in self.leg_names
        }

    def get_voltage(self) -> float:
        """Get battery voltage."""
        return self.kernel.get_voltage()
    
    def get_current(self) -> float:
        """Get total current draw."""
        return self.kernel.get_current()

    def check(self) -> bool:
        """Check if robot is operating within safety limits."""

        if self.get_voltage() < self.voltage_min:
            return False

        if self.get_current() > self.current_max:
            return False

        return True

    def set_led(self, pin: int, r: int, g: int, b: int):
        """Set LED to color."""
        self.kernel.set_led(pin, r, g, b)