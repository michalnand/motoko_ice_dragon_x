import numpy
import LibsControl

class DifferentialRobot:
    def __init__(self, dt = 0.001):

        self.sensors_distance  = 49.0  #mm
        self.width = 87.0
        self.height = 88.0

        self.dt = dt

        #time constant for yaw angle robot rotation (steering)
        tau_turn    = 6.3

        #time constant for robot acceleration (forward direction)
        tau_forward = 0.9

        #amplification, ratio between measured value amplitude : controll variable amplitude
        k_turn      = 1.5
        k_forward   = 0.2   


        self.mat_a = numpy.zeros((4, 4))
        self.mat_b = numpy.zeros((4, 2))


        self.mat_a[0][0] =  -tau_turn
        self.mat_a[1][0] =  1.0
        self.mat_a[2][2] = -tau_forward
        self.mat_a[3][2] = 1.0


        self.mat_b[0][0] =  -k_turn*tau_turn
        self.mat_b[0][1] =   k_turn*tau_turn      
        self.mat_b[2][0] =  k_forward*tau_forward
        self.mat_b[2][1] =  k_forward*tau_forward
  

        #cartesian state space
        self.theta = 0
        self.x_pos = 0
        self.y_pos = 0


        self.x = numpy.zeros((4, 1))
        self.robot_state = numpy.zeros((3, 1))


    def forward(self, u):
        self.x, _  = LibsControl.ODESolverRK4(self._forward_func, self.x, u, self.dt)

        #angular velocity
        w  = self.x[0, 0]
        
        #velocity
        v = self.x[2, 0]

        #position in cartesian
        self.x_pos+= v*numpy.cos(self.theta)*self.dt
        self.y_pos+= v*numpy.sin(self.theta)*self.dt
        self.theta+= w*self.dt
         
        #wrap into -pi .. pi
        #self.theta = numpy.arctan2(numpy.sin(self.theta), numpy.cos(self.theta))

        #wrap into 0 .. 2pi
        self.theta = numpy.mod(self.theta, 2.0*numpy.pi)

        return self.get_state()
    
    def get_state(self):
        self.robot_state = numpy.array([[self.x_pos], [self.y_pos], [self.theta]])
        return self.x, self.robot_state
    
    def _forward_func(self, x, u):
        dx = self.mat_a@x + self.mat_b@u
        return dx, x 

