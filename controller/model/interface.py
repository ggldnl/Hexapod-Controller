"""
Interfaces with kernel to control servos and read sensors.
Handles servo mapping, calibration, and safety limits.
"""

import numpy as np
from typing import Dict, Tuple


class Interface:
    """Hardware abstraction layer for robot control"""
    
    def __init__(self, kernel, config: dict):
        self.kernel = kernel
        self.config = config

        # Fix leg names order
        self.leg_names = [k for k, v in config['kinematics']['legs'].items() if isinstance(v, dict)]

        self.servos = {}
        for leg_name in self.leg_names:

            self.servos[leg_name] = {}
            pins = self.config['hardware']['pins'][leg_name]
            trim = self.config['hardware']['trim'][leg_name]
            direction = self.config['hardware']['direction'][leg_name]

            for i, joint in enumerate(['coxa', 'femur', 'tibia']):
                range_min, range_max = tuple(self.config['safety'][f'{joint}_range'])
                self.servos[leg_name][joint] = {
                    'pin': pins[i],
                    'trim': trim[i],
                    'direction': direction[i],
                    'min': range_min,
                    'max': range_max
                }

        # Safety limits
        self.voltage_min = self.config['safety']['voltage_min']
        self.current_max = self.config['safety']['current_max']

        self.enabled = True

    def enable(self):
        """Enable all servos"""
        self.kernel.attach_servos()
        self.kernel.connect_power()
        self.enabled = True
    
    def disable(self):
        """Disable all servos"""
        self.kernel.detach_servos()
        self.kernel.disconnect_power()
        self.enabled = False
    
    def convert_angle(self, leg: str, joint: str, angle: float) -> float:
        """Apply trim and direction to convert kinematic angle to servo angle"""

        angle *= self.servos[leg][joint]['direction']
        angle += self.servos[leg][joint]['trim']
        servo_min = self.servos[leg][joint]['min']
        servo_max = self.servos[leg][joint]['max']
        angle = np.clip(angle, servo_min, servo_max)
        
        return angle
    
    def set_angle(self, leg: str, joint: str, value: float) -> bool:
        """Set single joint angle (degrees)"""

        # Map kinematic angle to servo angle
        new_angle = self.convert_angle(leg, joint, value)

        pin = self.servos[leg][joint]['pin']
        result = self.kernel.set_servo_angle(pin, new_angle)

        return result

    def set_leg_angles(self, leg: str, coxa: float, femur: float, tibia: float) -> bool:
        """Set joint angles (degrees) for single leg"""

        # Map kinematic angle to servo angle
        joints = {'coxa': coxa, 'femur': femur, 'tibia': tibia}
        for joint, angle in joints.items():
            new_angle = self.convert_angle(leg, joint, angle)
            joints[joint] = new_angle

        # Bulk update angles
        values = [(self.servos[leg][joint]['pin'], joints[joint]) for joint in ['coxa', 'femur', 'tibia']]
        return self.kernel.set_servo_angles(values)

    def set_all_legs(self, leg_angles: Dict[str, Tuple[float, float, float]]) -> bool:
        """Set angles (degrees) for all legs simultaneously"""

        # Prepare bulk update
        all_pins = []
        all_angles = []

        for leg_name in self.leg_names:
            for i, joint_name in enumerate(['coxa', 'femur', 'tibia']):
                all_pins.append(self.servos[leg_name][joint_name]['pin'])
                all_angles.append(self.convert_angle(leg_name, joint_name, leg_angles[leg_name][i]))

        values = [(pin, angle) for pin, angle in zip(all_pins, all_angles)]
        return self.kernel.set_servo_angles(values)
    
    def get_voltage(self) -> float:
        """Get battery voltage"""
        return self.kernel.get_voltage()
    
    def get_current(self) -> float:
        """Get total current draw"""
        return self.kernel.get_current()

    def check(self) -> bool:
        """Check if robot is operating within safety limits"""

        if self.get_voltage() < self.voltage_min:
            return False

        if self.get_current() > self.current_max:
            return False

        return True

    def set_led(self, pin: int, r: int, g: int, b: int):
        """Set LED color"""
        self.kernel.set_led(pin, r, g, b)