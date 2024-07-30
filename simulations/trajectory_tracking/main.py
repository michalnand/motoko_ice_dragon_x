import numpy
from robot_env import *
import LibsControl


def get_required(target_x, target_y, robot_x, robot_y, robot_theta):
    d_distance = (target_x - robot_x)**2 + (target_y - robot_y)**2
    d_distance = d_distance**0.5

    required_theta    = numpy.arctan2(target_y - robot_y, target_x - robot_x)
    required_theta    = numpy.mod(required_theta, 2.0*numpy.pi)

    d_theta = required_theta - robot_theta

    if d_theta < 0:
        d_theta+= 2.0*numpy.pi
    if d_theta > numpy.pi:
        d_theta-= 2.0*numpy.pi

    return d_distance, d_theta

if __name__ == "__main__":

    dt    = 1.0/256.0
    
    robot       = DifferentialRobot(dt)
    trajectory  = Trajectory()
    robot_env   = RobotEnv(robot, trajectory)

    
    # sythetise discrete controller
    q = [0.0, 10.0, 0.0, 1.0]
    q = numpy.diag(q)

    
    r = [0.0001, 0.01] 
    r =  numpy.diag(r)

    mat_a, mat_b, _ = LibsControl.c2d(robot.mat_a, robot.mat_b, None, dt)
    
    di_max = numpy.array([1000, 10])
    lqri     = LibsControl.LQRDiscrete(mat_a, mat_b, q, r, 10**10, di_max)

    integral_action = numpy.zeros((2, 1))

    #print solved controller matrices
    print("controller\n\n")
    print("k=\n", numpy.round(lqri.k, 5), "\n")
    print("ki=\n", numpy.round(lqri.ki, 5), "\n")
    print("\n\n")

    x, robot_state, target_state = robot_env.reset()

    starting_idx = target_state[2][0]

    steps = 0

    min_idx = 0

    while(True):
        target_x     = target_state[0][0]
        target_y     = target_state[1][0]
        robot_x      = robot_state[0][0]
        robot_y      = robot_state[1][0]
        robot_theta  = robot_state[2][0]
        
        d_distance, d_theta = get_required(target_x, target_y, robot_x, robot_y, robot_theta)
      
        d_distance = 2.5*d_distance
       
        xr = numpy.array([[0.0, x[1, 0] + d_theta, 0.0,  x[3, 0] + d_distance]]).T

        u, integral_action = lqri.forward(xr, x, integral_action)
                 
        x, robot_state, target_state = robot_env.step(u)
        

        if steps%80 == 0:
            robot_env.render()
            print("v = ", x[2][0], x[3][0], target_state[2][0])

        steps+= 1

        
        if steps > 1000 and robot_env.is_close_to_start():
            break
        

    print("total distance : ", x[3][0])
    print("total time     : ", steps*dt)
    print("average speed  : ", x[3][0]/(steps*dt))