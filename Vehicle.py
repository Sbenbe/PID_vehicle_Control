from .const import *
from .PID import PID

class Vehicle:
    def __init__(self):
        super().__init__('acc_node')
        self.pid = PID()
        err = self.pid.GetError()

    # Parameters
    name = ''
    position0 = 0    # initial position (m)
    velocity0 = 0    # initial velocity (m/s)
    position = 0
    velocity = 0
    acceleration = 0  # m/s^2
    control = 0     # value output from the controller
    minimum = 0
    maximum = 0
    # vehicle parameters
    mass = 1200   # vehicle mass, (kg)
    Cd = 0.3      # coefficient of drag
    rho = 1.225   # air density (kg/m^3)
    A = 2.2       # frontal vehicle area (m^2)
    g = 9.81      # gravitational acceleration (m/s^2)
    Crr = 0.01    # rolling resistance coefficient
    
    

    def __init__(self, name, position0, velocity0):
        self.name = name
        self.position0 = position0
        self.velocity0 = velocity0
        self.pid = PID()

    def GetPosition(self):
        return self.position

    def UpdateState(self, control):
        self.control = control

        # control is either brake force or gas force
        # F = ma rearranged with Fcontrol, Fdrag, Ffriction
        acceleration = 1/self.mass * \
            (control - 0.5*self.rho*pow(self.velocity, 2)*self.A*self.Cd-self.Crr*self.mass*self.g)

        # check if acceleration bounds are violated
        if (self.CheckBounds(acceleration) == False):
            if (acceleration >= 0):
                acceleration = self.maximum
            else:
                acceleration = self.minimum

        # v = v + a * dt
        self.velocity = self.velocity + acceleration*DT

        # p = p + v * dt
        self.position = self.position + self.velocity*DT

    # manually sets acceleration
    def UpdateStateManual(self, acceleration):
        self.acceleration = acceleration
        self.velocity = self.velocity + acceleration * DT
        self.position = self.position + self.velocity * DT

    # get vehicle velocity
    def GetVelocity(self):
        return self.velocity

    # get vehicle acceleration
    def GetAcceleration(self):
        return self.acceleration

    def SetBounds(self, minimum, maximum):
        self.minimum = minimum
        self.maximum = maximum

    
    
    # compute acceleration based on ACC model
    # A new method to compute acceleration using the vehicle model
    def ComputeAcceleration(self, actual_velocity):
        # Compute ACC control using the vehicle model
        # error = lead_distance - ego_distance
        control = self.pid.Compute(actual_velocity, self.velocity)
        return control / self.mass - self.Crr * self.g - 0.5 * self.rho * pow(self.velocity, 2) * self.A * self.Cd / self.mass

    def CheckBounds(self, acceleration):
        if (self.maximum > acceleration and self.minimum < acceleration):
            return True
        else:
            return False
