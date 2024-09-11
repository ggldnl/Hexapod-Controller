from interpolator import Interpolator
from controller import Controller
import json
import time

from src.interpolator import InterpolationMethod

def reach_configuration(leg, current_angles, target_angles, controller, speed=0.3):

    pins = [leg[link]['pin'] for link in ['coxa', 'femur', 'tibia']]

    coxa = Interpolator(method=InterpolationMethod.PROPORTIONAL)
    femur = Interpolator(method=InterpolationMethod.PROPORTIONAL)
    tibia = Interpolator(method=InterpolationMethod.PROPORTIONAL)

    coxa.set_current(current_angles[0])
    femur.set_current(current_angles[1])
    tibia.set_current(current_angles[2])

    coxa.set_speed(speed)
    femur.set_speed(speed)
    tibia.set_speed(speed)

    coxa.set_target(target_angles[0])
    femur.set_target(target_angles[1])
    tibia.set_target(target_angles[2])

    while not coxa.is_at_target() or not femur.is_at_target() or not tibia.is_at_target():

        new_coxa = coxa.update()
        new_femur = femur.update()
        new_tibia = tibia.update()

        print(f'Angles: {round(new_coxa, 2)}, {round(new_femur, 2)}, {round(new_tibia, 2)}')

        """
        controller.set_angle(pins[0], new_coxa)
        controller.set_angle(pins[1], new_femur)
        controller.set_angle(pins[2], new_tibia)
        """

        controller.set_angles(pins, [new_coxa, new_femur, new_tibia])

        time.sleep(0.05)


if __name__ == '__main__':

    """
    Simulate the interpolation from one position of the leg to another
    (without using inverse kinematics).
    """

    controller = Controller()
    with controller:

        # Load config.json from file
        with open('config/config.json', 'r') as f:
            config = json.load(f)
            leg_config = config['leg_6']

        # Attach the servos
        controller.attach_servos()
        time.sleep(1)

        # Reach the target configuration
        reach_configuration(leg_config, [0, 0, 0], [-45, 45, 45], controller, speed=0.6)

        time.sleep(1)

        # Go back
        reach_configuration(leg_config, [-45, 45, 45], [0, 0, 0], controller, speed=0.6)

        # Detach the servos
        controller.detach_servos()
