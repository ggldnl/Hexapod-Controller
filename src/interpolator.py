from enum import Enum


class InterpolationMethod(Enum):
    """
    Enum representing the interpolation methods.
    - PROPORTIONAL: Step size is proportional to the difference between the current and target angle.
    - CONSTANT: A constant step size is used based on the speed, regardless of the distance to the target.
    """
    PROPORTIONAL = 1
    CONSTANT = 2


class Interpolator:
    """
    Class to interpolate the angle of a servo from its current angle to a target angle at a specified speed.

    Attributes:
        _current_angle (float): The current angle of the servo.
        _target_angle (float): The target angle to reach.
        _speed (float): The speed factor for the interpolation (between 0 and 1).
        _eps (float): The tolerance for angle matching (default is 5 degrees).
        _method (InterpolationMethod): The interpolation method, either PROPORTIONAL or CONSTANT.
        _step (float): The constant step size when using the CONSTANT interpolation method.
    """

    def __init__(self, current_angle=0.0, speed=1.0, method=InterpolationMethod.PROPORTIONAL):
        """
        Initializes the Interpolator with the current angle, speed, and interpolation method.

        Args:
            current_angle (float): The initial angle of the servo (default is 0.0).
            speed (float): The interpolation speed factor (default is 1.0).
            method (InterpolationMethod): The interpolation method (default is PROPORTIONAL).
        """
        self._current_angle = current_angle
        self._target_angle = current_angle
        self._speed = speed
        self._eps = 5  # Less than 5 degrees mismatch will be compensated
        self._method = method
        self._step = 0.0

    @property
    def target_angle(self):
        """Returns the target angle of the servo."""
        return self._target_angle

    @property
    def current_angle(self):
        """Returns the current angle of the servo."""
        return self._current_angle

    @property
    def speed(self):
        """Returns the current speed of interpolation."""
        return self._speed

    def is_at_target(self):
        """Checks whether the servo has reached the target angle."""
        return self._current_angle == self._target_angle

    def set_speed(self, speed):
        """Sets the interpolation speed."""
        self._speed = speed

    def set_method(self, method):
        """Sets the interpolation method."""
        self._method = method

    def set_target(self, target_angle):
        """
        Sets the target angle for the interpolation. If the method is CONSTANT, the step size is calculated.

        Args:
            target_angle (float): The target angle to reach.
        """
        self._target_angle = target_angle

        if self._method.value == InterpolationMethod.CONSTANT.value:
            # Set the constant step size as a fraction of the difference between current and target angle
            self._step = (self._target_angle - self._current_angle) * self._speed

    def set_current(self, current_angle):
        """Sets the current angle."""
        self._current_angle = current_angle

    def update(self):
        """
        Updates the current angle by interpolating toward the target angle based on the selected method.

        If the method is PROPORTIONAL, the step size is proportional to the difference between the current and
        target angles, scaled by the speed. If the method is CONSTANT, the step size is fixed and calculated when
        setting the target.

        Returns:
            float: The updated current angle.
        """

        # Just to be sure, do this only if we need to reach the target
        if not self.is_at_target():

            if self._method.value == InterpolationMethod.PROPORTIONAL.value:
                # Proportional interpolation
                step = (self._target_angle - self._current_angle) * self._speed
                if step < self._eps:
                    step = self.target_angle - self.current_angle
            else:
                # Constant interpolation
                step = self._step

            if abs(self._current_angle + step) < abs(self._target_angle):
                self._current_angle += step
            else:
                self._current_angle = self._target_angle

        return self._current_angle


if __name__ == '__main__':

    """
    Interpolate to the target angle with the specified velocity.
    """

    interpolator = Interpolator(method=InterpolationMethod.CONSTANT)
    interpolator.set_target(-90.0)
    interpolator.set_speed(0.5)

    while not interpolator.is_at_target():
        new_angle = interpolator.update()
        print(f'New value: {new_angle}')

    print(f'Done')
