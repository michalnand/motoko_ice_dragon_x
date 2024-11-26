import numpy
from robot_env import *
import LibsControl
import matplotlib.pyplot as plt


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


def step_response(robot):

    robot.reset()
    u = numpy.zeros((2, 1))

    u_log = []
    x_log = []

    n_steps = 500
    for n in range(n_steps):

        x, _ = robot.forward(u)

        if n >= 0.7*n_steps:
            u[:, 0] = 0.0
        elif n >= 0.3*n_steps:
            u[:, 0] = 1.0
            
                
        u_log.append(u[:, 0].copy())
        x_log.append(x[:, 0].copy())

    u_log = numpy.array(u_log)
    x_log = numpy.array(x_log)

    #fig, axs = plt.subplots(6, figsize=(3.5, 2.5), layout="constrained")
    fig, axs = plt.subplots(6)
    
    axs[0].plot(u_log[:, 0], label="u_forward", color="red")
    axs[1].plot(u_log[:, 1], label="u_turn", color="red")

    axs[2].plot(x_log[:, 0], label="velocity")
    axs[3].plot(x_log[:, 1], label="distance")

    axs[4].plot(x_log[:, 2], label="omega")
    axs[5].plot(x_log[:, 3], label="angle")
    

    for n in range(len(axs)):
        axs[n].legend()

    plt.show()

if __name__ == "__main__":

    dt    = 1.0/250.0
    
    robot       = DifferentialRobot(dt)

    #step_response(robot)


    trajectory  = Trajectory()
    robot_env   = RobotEnv(robot, trajectory, 1, "video_lqr.mp4")

    
    # sythetise discrete controller
    q = [0.0, 3.0, 0.0, 1.0]
    q = numpy.diag(q)

    
    r = [1.0, 1.0] 
    r =  numpy.diag(r)  

    mat_a, mat_b, _ = LibsControl.c2d(robot.mat_a, robot.mat_b, None, dt)
    
    di_max  = numpy.array([[1.0], [1.0]])
    lqr     = LibsControl.LQRDiscrete(mat_a, mat_b, q, r, 10**3, di_max)

    integral_action = numpy.zeros((2, 1))

    #print solved controller matrices
    print("controller\n\n")
    print("k=\n", numpy.round(lqr.k, 5), "\n")
    print("ki=\n", numpy.round(lqr.ki, 5), "\n")
    print("\n\n")

    x, robot_state, target_state = robot_env.reset()

    steps = 0

    min_idx = 0

    while(True):
        target_x     = target_state[0][0]
        target_y     = target_state[0][1]
        robot_x      = robot_state[0][0]
        robot_y      = robot_state[1][0]
        robot_theta  = robot_state[2][0]
        
        d_distance, d_theta = get_required(target_x, target_y, robot_x, robot_y, robot_theta)
      
        d_distance = 2.0*d_distance
       
        xr = numpy.array([[0.0, x[1, 0] + d_distance, 0.0, x[3, 0] + d_theta]]).T

        u, integral_action = lqr.forward(xr, x, integral_action, robot_env.robot.constrain_inputs)

                 
        x, robot_state, target_state = robot_env.step(u)
        

        if steps%10 == 0:
            robot_env.render()
            print("v = ",  x[0][0], x[2][0])

        steps+= 1

        
        if steps > 1000 and robot_env.is_close_to_start():
            break

       
    print("total distance : ", round(x[3][0], 3))
    print("total time     : ", round(steps*dt, 3))
    print("average speed  : ", round(x[3][0]/(steps*dt), 3))