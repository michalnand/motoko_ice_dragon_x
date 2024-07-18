import numpy
import LibsControl
from robot_dynamics import *
from trajectory     import *
from render         import *

import time

class RobotEnv:
    def __init__(self, robot_dynamics : RobotDynamicsModel, trajectory : Trajectory):
        self.dynamics       = robot_dynamics
        self.trajectory     = trajectory
        self.renderer       = Render(700)

    def step(self, u):
        self.dynamics.step(u)
        return self.get_state()

    def reset(self):
        self.dynamics.reset()

    def get_state(self):
        return self.dynamics.get_state()
    

    def render(self):
        self.renderer.render(self.trajectory)
        time.sleep(0.1)
