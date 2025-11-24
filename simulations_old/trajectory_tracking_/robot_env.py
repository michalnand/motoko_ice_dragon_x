import numpy
import LibsControl
from robot_dynamics import *
from trajectory     import *
from render         import *

import time


'''
robot state : 
pos_x,  pos_y,  theta, 
line_x, line_y, line_angle,
distance, angle, velocity, angular_velocity
'''

class RobotEnv:
    def __init__(self, robot_dynamics : RobotDynamicsModel, trajectory : Trajectory):
        self.dynamics       = robot_dynamics
        self.trajectory     = trajectory
        self.renderer       = Render(700)

        self.wheels_brace        = 76.0  #mm
        self.wheel_diameter      = 23.0  #mm
        self.sensors_distance    = 49.0  #mm
        self.sensors_brace       = 70.0  #mm
        self.weight              = 71.0  #g, batt incl, no batt : 59.5
        
        self.height              = 40.0
        self.width               = 80.0
     

    def step(self, u):
        self.dynamics.step(u)
        return self.get_state()

    def get_state(self):    
        self.sensor_x = self.dynamics.pos_x + self.sensors_distance*numpy.cos(self.dynamics.theta)
        self.sensor_y = self.dynamics.pos_y + self.sensors_distance*numpy.sin(self.dynamics.theta)

        line_x, line_y, min_idx = self._line_nearest(self.sensor_x, self.sensor_y)
         
        line_angle = self._signed_angle(self.dynamics.pos_x, self.dynamics.pos_y, self.sensor_x, self.sensor_y, line_x, line_y)

        line_state = [[line_x], [line_y], [line_angle], [min_idx]]

        dynamics_state, robot_state = self.dynamics.get_state()

        return dynamics_state, robot_state, line_state

    def reset(self):

        x0      = self.trajectory.key_points[0][0]
        y0      = self.trajectory.key_points[0][1]
        theta0  = 0

        self.dynamics.reset()
        self.dynamics.pos_x = x0
        self.dynamics.pos_y = y0
        self.dynamics.theta = theta0


    

    def render(self):
        self.renderer.render(self.trajectory, self)

    def _line_nearest(self, x, y):
        robot_pos = numpy.array([[x, y]])

        d = ((self.trajectory.points - robot_pos)**2).sum(axis=-1)

        min_idx = numpy.argmin(d)
        x = self.trajectory.points[min_idx][0]
        y = self.trajectory.points[min_idx][1]

        print(min_idx)
        
        return x, y, min_idx

    def _signed_angle(self, x0, y0, x1, y1, x, y):
        # Define vectors
        v1 = numpy.array([x1 - x0, y1 - y0])
        v2 = numpy.array([x - x0, y - y0])
        
        # Normalize vectors
        v1_normalized = v1 / numpy.linalg.norm(v1)
        v2_normalized = v2 / numpy.linalg.norm(v2)
        
        # Compute dot product and cross product
        dot_product = numpy.dot(v1_normalized, v2_normalized)
        cross_product = numpy.cross(v1_normalized, v2_normalized)
        
        # Compute the angle using arctan2
        angle = numpy.arctan2(cross_product, dot_product)
        
        return angle