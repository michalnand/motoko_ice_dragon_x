from robot_motion  import *
from visualisation import *
from line_slam     import *

import numpy
import time

if __name__ == "__main__":
    print("\n\n\n\n")

    distance_uncertainty = 0.0025
    theta_uncertainty    = 0.0025
    
    
    robot_motion  = RobotMotion(distance_uncertainty, theta_uncertainty, random_landmarks=False)
    slam          = LineSlam(distance_uncertainty, theta_uncertainty)
    visualisation = Visualisation()


    while True:
        dDistance, dTheta, landmark = robot_motion.step()

        slam.step(dDistance, dTheta, landmark)

        visualisation.step(robot_motion, slam)

        time.sleep(0.001)