import numpy
from robot_env import *



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



dt = 1.0/250.0

q = numpy.diag([ 1.0, 1.0, 0.0, 0.0] )
r = numpy.diag( [10000000.0, 1000.0])  


robot_dynamics  = RobotDynamicsModel(dt)
trajectory      = Trajectory()
robot_env       = RobotEnv(robot_dynamics, trajectory)

mat_a, mat_b, _ = LibsControl.c2d(robot_env.dynamics.a, robot_env.dynamics.b, robot_env.dynamics.c, dt)


#solve LQG controller 
controller = LibsControl.LQRDiscrete(mat_a, mat_b, q, r, 10**10, 0.1)

print("controller")
print("k  = \n", controller.k, "\n")
print("ki = \n", controller.ki, "\n")
print("\n\n")




n_max = int(2.0/dt)

#required output, 1 meter, 100degrees
xr = numpy.array([[1000.0, 100.0*numpy.pi/180.0, 0.0, 0.0]]).T

#initial integral action
integral_action = numpy.zeros((robot_env.dynamics.b.shape[1], 1))

#result log
t_result = [] 
u_result = []
x_result = []


robot_env.reset()
x, _ = robot_env.get_state()

while True:
    #compute controller output
    u, integral_action = controller.forward(xr, x, integral_action)

    #compute dynamical system output
    x, _ = robot_env.step(u)
    
    #t_result.append(n*dt)
    #u_result.append(u[:, 0].copy())
    #x_result.append(x[:, 0].copy())

    robot_env.render()

t_result = numpy.array(t_result) 
u_result = numpy.array(u_result) 
x_result = numpy.array(x_result) 

# radian angle to degrees
x_result[:, 1]*= 180.0/numpy.pi 
# radian angle to degrees
x_result[:, 3]*= 180.0/numpy.pi 

LibsControl.plot_closed_loop_response(t_result, u_result, x_result, x_hat = None, file_name = "lqr_controller_output.png", u_labels = ["control forward", "control turn"], x_labels = ["distance [mm]", "angle [deg]", "distance velocity [mm/s]", "angular velocity [deg/s]"])

