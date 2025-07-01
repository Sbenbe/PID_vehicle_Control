
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.error = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0

    def update(self, measured_value):
        self.error = self.setpoint - measured_value
        self.integral += self.error
        self.derivative = self.error - self.last_error
        self.last_error = self.error
        return self.Kp*self.error + self.Ki*self.integral + self.Kd*self.derivative
