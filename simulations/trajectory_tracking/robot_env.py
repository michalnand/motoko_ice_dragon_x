import numpy
import LibsControl
from differential_robot import *
from trajectory         import *
from render             import *

import time

class RobotEnv:
    def __init__(self, robot : DifferentialRobot, trajectory : Trajectory):
        self.robot          = robot
        self.trajectory     = trajectory
        self.renderer       = Render(700)

        self.reset()


    def reset(self):
        self.current_target_idx = 1

        self.robot.x = numpy.zeros((4, 1))

        self.robot.x_pos = self.trajectory.points[0][0]
        self.robot.y_pos = self.trajectory.points[0][1]
        self.robot.theta = 0.0

        return self.step(numpy.zeros([2, 1]))


    def render(self):
        self.renderer.render(self.trajectory, self.robot, self.target_state)


    def step(self, u):
        self.robot.forward( u)

        target_x, target_y, d, self.current_target_idx = self._step_target()

        self.target_state   = numpy.array([[target_x], [target_y], [d], [self.current_target_idx]])

        return self.get_state()

    def get_state(self):
        return self.robot.x, self.robot.robot_state, self.target_state


    def _step_target(self, reach_threshold = 150.0):
        sensor_x = self.robot.x_pos + self.robot.sensors_distance*numpy.cos(self.robot.theta)
        sensor_y = self.robot.y_pos + self.robot.sensors_distance*numpy.sin(self.robot.theta)

        target_x = self.trajectory.points[self.current_target_idx][0]
        target_y = self.trajectory.points[self.current_target_idx][1]

        d = self._distance(target_x, target_y, sensor_x, sensor_y)
        if d < reach_threshold:
            self.current_target_idx = (self.current_target_idx + 1)%self.trajectory.points.shape[0]

        return target_x, target_y, d, self.current_target_idx

    def _distance(self, ax, ay, bx, by):
        d = ((ax - bx)**2) + ((ay - by)**2)
        d = d**0.5
        return d

    def _line_nearest(self, x, y):
        robot_pos = numpy.array([[x, y]])

        d = ((self.trajectory.points - robot_pos)**2).sum(axis=-1)

        min_idx = numpy.argmin(d)
        x = self.trajectory.points[min_idx][0]
        y = self.trajectory.points[min_idx][1]
        
        return x, y, min_idx