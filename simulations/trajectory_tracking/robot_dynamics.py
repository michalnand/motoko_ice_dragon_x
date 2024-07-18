import numpy
import LibsControl


#two wheels diffetential drive robot model
#x state = (forward_position, angle, forward_velocity, angular_velocity)
class RobotDynamicsModel(LibsControl.DynamicalSystem):
    def __init__(self, dt, mat_a = None, mat_b = None):

        tau_forward = 0.1
        k_forward   = 2100.0
        tau_turn    = 0.02
        k_turn      = 150.0

        if mat_a is None:
            mat_a = numpy.zeros((4, 4))
            mat_a[0][2] = 1.0
            mat_a[1][3] = 1.0
            mat_a[2][2] = 1.0/tau_forward
            mat_a[3][3] = 1.0/tau_turn

        if mat_b is None:
            mat_b = numpy.zeros((4, 2))
            mat_b[2][0] = k_forward/tau_forward
            mat_b[3][1] = k_turn/tau_forward


        mat_c = numpy.eye(4)

        LibsControl.DynamicalSystem.__init__(self, mat_a, mat_b, mat_c, dt)

        #cartesian state space
        self.theta = 0
        self.x_pos = 0
        self.y_pos = 0

    def step(self, u):
        self.forward_state(u)

        #velocity
        v = self.x[2, 0]

        #angular velocity
        w = self.x[3, 0]
        

        #position in cartesian
        self.x_pos+= v*numpy.cos(self.theta)*self.dt
        self.y_pos+= v*numpy.sin(self.theta)*self.dt
        self.theta+= w*self.dt
         
        #wrap into -pi .. pi
        #self.theta = numpy.arctan2(numpy.sin(self.theta), numpy.cos(self.theta))

        #wrap into 0 .. 2pi
        self.theta = numpy.mod(self.theta, 2.0*numpy.pi)

        return self.x
    

    def get_state(self):
        return self.x,  numpy.array([[self.x_pos], [self.y_pos], [self.theta]])
    
