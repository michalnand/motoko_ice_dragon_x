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


'''
def get_required(targets, current_target_idx, robot_x, robot_y, robot_theta):
        
        idx      = current_target_idx%targets.shape[0]
        
        x_pos    = targets[idx][0]
        y_pos    = targets[idx][1]

        d_distance = (x_pos - robot_x)**2 + (y_pos - robot_y)**2
        d_distance = d_distance**0.5

        required_theta    = numpy.arctan2(y_pos - model.y_pos, x_pos - model.x_pos)
        required_theta    = numpy.mod(required_theta, 2.0*numpy.pi)

        d_theta = required_theta - robot_theta

        if d_theta < 0:
            d_theta+= 2.0*numpy.pi
        if d_theta > numpy.pi:
            d_theta-= 2.0*numpy.pi

        return d_distance, d_theta
'''

dt = 1.0/250.0

#q = numpy.diag([ 1.0, 1.0, 0.0, 0.0] )
#r = numpy.diag( [10000000.0, 1000.0])  

q = [1.0, 10.0, 0.0, 0.0]
q = numpy.diag(q)
    
r = [0.001, 0.001] 
r =  numpy.diag(r)


robot_dynamics  = RobotDynamicsModel(dt)
trajectory      = Trajectory()
robot_env       = RobotEnv(robot_dynamics, trajectory)

mat_a, mat_b, _ = LibsControl.c2d(robot_env.dynamics.a, robot_env.dynamics.b, robot_env.dynamics.c, dt)


#solve LQG controller 
controller = LibsControl.LQRDiscrete(mat_a, mat_b, q, r)

print("controller")
print("k  = \n", controller.k, "\n")
print("ki = \n", controller.ki, "\n")
print("\n\n")




#required output, 1 meter, 100degrees
#xr = numpy.array([[1000.0, 100.0*numpy.pi/180.0, 0.0, 0.0]]).T
xr = numpy.zeros((4, 1))

#initial integral action
integral_action = numpy.zeros((robot_env.dynamics.b.shape[1], 1))

#result log
t_result = [] 
u_result = []
x_result = []


robot_env.reset()

x, robot_state, line_state = robot_env.get_state()

steps = 0
while True:
    #compute controller output

    rx = robot_state[0][0]
    ry = robot_state[1][0]
    rtheta = robot_state[2][0]  

    lx = line_state[0][0]
    ly = line_state[1][0]

    d_distance, d_theta = get_required(lx, ly, rx, ry, rtheta)

    '''
    xr[0][0] = xr[0][0] + 0.1*d_distance
    xr[1][0] = xr[1][0] + d_theta


    u, integral_action = controller.forward(xr, x, integral_action)
    '''

    u = numpy.zeros((2, 1))

    u[0][0] = 1000.0
    u[1][0] = d_theta

    #compute dynamical system output
    x, robot_state, line_state = robot_env.step(u)
    
    #t_result.append(n*dt)
    #u_result.append(u[:, 0].copy())
    #x_result.append(x[:, 0].copy())
    
    if steps%2 == 0:
        robot_env.render()
    steps+= 1

t_result = numpy.array(t_result) 
u_result = numpy.array(u_result) 
x_result = numpy.array(x_result) 

# radian angle to degrees
x_result[:, 1]*= 180.0/numpy.pi 
# radian angle to degrees
x_result[:, 3]*= 180.0/numpy.pi 

LibsControl.plot_closed_loop_response(t_result, u_result, x_result, x_hat = None, file_name = "lqr_controller_output.png", u_labels = ["control forward", "control turn"], x_labels = ["distance [mm]", "angle [deg]", "distance velocity [mm/s]", "angular velocity [deg/s]"])

