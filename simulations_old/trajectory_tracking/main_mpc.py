import numpy
from robot_env import *
import LibsControl
import matplotlib.pyplot as plt


def get_required(target_trajectory, x, robot_x, robot_y, robot_theta, prediction_horizon):
    result = []

    for n in range(prediction_horizon):
        target = target_trajectory[n]
        d_distance = (target[0] - robot_x)**2 + (target[1] - robot_y)**2
        d_distance = d_distance**0.5

        required_theta    = numpy.arctan2(target[1] - robot_y, target[0] - robot_x)
        required_theta    = numpy.mod(required_theta, 2.0*numpy.pi)

        d_theta = required_theta - robot_theta

        if d_theta < 0:
            d_theta+= 2.0*numpy.pi
        if d_theta > numpy.pi:
            d_theta-= 2.0*numpy.pi

        result.append([0, d_distance + x[1, 0], 0, d_theta + x[3, 0]])

    result = numpy.array(result)

    return result


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
    robot_env   = RobotEnv(robot, trajectory, video_file_name = "video_mpc.mp4")

    
    # synthetise discrete controller
    q = [0.0, 500.0, 0.0, 500.0]
    q = numpy.diag(q)

    
    r = [1.0, 1.0] 
    r =  numpy.diag(r)  

    mat_a, mat_b, _ = LibsControl.c2d(robot.mat_a, robot.mat_b, None, dt)

    prediction_horizon = 128
    
    di_max  = numpy.array([[1.0], [1.0]])
    mpc     = LibsControl.libs_mpc.AnalyticalMPC(mat_a, mat_b, q, r, prediction_horizon, 32)
    u = numpy.zeros((2, 1))


    x, robot_state, target_trajectory = robot_env.reset()

    steps = 0

    min_idx = 0

    while(True):
        robot_x      = robot_state[0][0]
        robot_y      = robot_state[1][0]
        robot_theta  = robot_state[2][0]
        
        xr = get_required(target_trajectory, x, robot_x, robot_y, robot_theta, prediction_horizon)
      
        u = mpc.forward(xr, x, u, robot_env.robot.constrain_inputs)
     
        x, robot_state, target_trajectory = robot_env.step(u)
        
        if steps%10 == 0:
            robot_env.render()
            print("v = ",  x[0][0], x[2][0])

        steps+= 1

        
        if steps > 1000 and robot_env.is_close_to_start():
            break

       
    print("total distance : ", round(x[3][0], 3))
    print("total time     : ", round(steps*dt, 3))
    print("average speed  : ", round(x[3][0]/(steps*dt), 3))