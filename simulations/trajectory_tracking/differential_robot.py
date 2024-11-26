import numpy
import LibsControl

class DifferentialRobot:
    def __init__(self, dt = 0.001):
        self.dt = dt

        # robot dimensions in m
        self.sensors_distance  = 49.0*0.001
        self.width  = 87.0*0.001
        self.height = 88.0*0.001

        # wheel brace
        self.wheels_brace = 88.0*0.001

        #wheel diameter
        self.wheel_diameter = 22*0.001

        #max rpm
        self.max_rpm = 1000


        #amplification, ratio between measured value amplitude : controll variable amplitude

        v_max     = numpy.pi * self.wheel_diameter * (self.max_rpm / 60.0)
        omega_max = (2.0 * v_max) / (self.wheels_brace)


        k_forward   = v_max
        k_turn      = omega_max

        print("K = ", k_forward, k_turn)
        
        #time constant for yaw angle robot rotation (steering)
        tau_turn    = 0.05

        #time constant for robot acceleration (forward direction)
        tau_forward = 0.1

    
        self.mat_a = numpy.zeros((4, 4))
        self.mat_b = numpy.zeros((4, 2))


        self.mat_a[0][0] =  -1.0/tau_forward
        self.mat_a[1][0] =  1.0
        self.mat_a[2][2] =  -1.0/tau_turn
        self.mat_a[3][2] =  1.0

        self.mat_b[0][0] =  k_forward/tau_forward
        self.mat_b[2][1] =  k_turn/tau_turn

        #cartesian state space
        self.theta = 0
        self.x_pos = 0
        self.y_pos = 0

        print(self.mat_a)
        print(self.mat_b)


        self.reset()
        


    def reset(self):
        self.x = numpy.zeros((4, 1))
        self.robot_state = numpy.zeros((3, 1))


    def forward(self, u):
        #u = self.constrain_inputs(u)
        self.x, _  = LibsControl.ODESolverRK4(self._forward_func, self.x, u, self.dt)

        #velocity
        v = self.x[0, 0]

        #angular velocity
        w  = self.x[2, 0]
        
        
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


    def constrain_inputs(self, u_in):

        u_v     = u_in[0][0]
        u_omega = u_in[1][0]

        # Maximum linear velocity
        v_max = numpy.pi * self.wheel_diameter * (self.max_rpm / 60.0)
        
        # Compute constraints on u_v
        v_upper = min(v_max, -0.5 * self.wheels_brace * u_omega + v_max)
        v_lower = max(-v_max, -0.5 * self.wheels_brace * u_omega - v_max)
        
        # Constrain u_v to ensure motor velocities stay within max RPM
        u_v_constrained = numpy.clip(u_v, v_lower, v_upper)

        result = numpy.array([[u_v_constrained], [u_omega]])
        
        return result
    

    def constrain_inputs(self, u_in):
        
        velocity = u_in[0][0]
        omega    = u_in[1][0]

        # turn is prioritised in maximum range
        omega_min = -1.0
        omega_max =  1.0

        # omega clipping
        omega_clipped = numpy.clip(omega, omega_min, omega_max)

        # maximum linear velocity
        v_max = numpy.pi * self.wheel_diameter * (self.max_rpm / 60.0)
        
        # compute constraints for linear velocity
        v_upper = min(v_max, -0.5 * self.wheels_brace * omega_clipped + v_max)
        v_lower = max(-v_max, -0.5 * self.wheels_brace * omega_clipped - v_max)
        
        v_clipped = numpy.clip(velocity, v_lower, v_upper)

        result = numpy.array([[v_clipped], [omega_clipped]])

        return result
    