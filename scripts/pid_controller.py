#!/usr/bin/env python3



class PIDController:
    """
    A simple PID controller.

    Attributes:
        kp (float): Proportional gain.
        ki (float): Integral gain.
        kd (float): Derivative gain.
        max_output (float): Maximum output value.
        min_output (float): Minimum output value.
        integral (float): Integral accumulator.
        previous_error (float): Previous error value.
    """
    def __init__(self, kp, ki, kd, max_output=float('inf'), min_output=-float('inf')):
        """
        Initialize the PID controller with specified gains and output limits.

        Parameters:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            max_output (float): Maximum output allowed.
            min_output (float): Minimum output allowed.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.integral = 0
        self.previous_error = 0


    def compute(self, error, dt):
        """
        Compute the PID control output.

        Parameters:
            error (float): The current error value.
            dt (float): Time difference since last update.

        Returns:
            float: The control output.
        """
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(self.min_output, min(output, self.max_output))

        self.previous_error = error

        return output
