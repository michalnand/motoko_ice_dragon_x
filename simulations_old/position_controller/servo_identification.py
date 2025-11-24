import LibsControl
import numpy


#load dynamics from log file
def load_from_file(file_name, dt):
    v = numpy.loadtxt(file_name)

    u = numpy.array(v[:, 1:3])
    x = numpy.array(v[:, 3:5])


    t = numpy.arange(x.shape[0])*dt

    return u, x, t

def _difference(x):
    #add velocity terms
    x_pad = numpy.zeros((1, x.shape[1]))
    x_tmp = numpy.concatenate([x_pad, x], axis=0)

    #estimate velocity term
    dx = x_tmp[1:, :]  - x_tmp[0:-1, :]

    return dx

def servo_identification(u, x):

    if len(u.shape) != 2:
        u = numpy.expand_dims(u, 1)

    if len(x.shape) != 2:
        x = numpy.expand_dims(x, 1)

    # initial position
    x_initial = x[0, 0]

    # remove initial bias
    x_aug = x - x_initial

    # compute velocity term
    v_aug = _difference(x_aug)


    a_est, b_est = LibsControl.rls_identification(u, v_aug)

    return a_est[0][0], b_est[0][0]


def robot_identification(file_name, dt = 1.0/250.0):
    dt = 1.0/250.0
    u_result, x_result, t_result = load_from_file("./data/run_1.txt", dt)

    n_samples = 4000

    u_result = u_result[0:n_samples, :]
    x_result = x_result[0:n_samples, :]


    af, bf = servo_identification(u_result[:, 0], x_result[:, 0])
    at, bt = servo_identification(u_result[:, 1], x_result[:, 1])

    a_est = numpy.zeros((4, 4))
    b_est = numpy.zeros((4, 2))

    a_est[0][0] = 1.0
    a_est[1][1] = 1.0
    a_est[0][2] = 1.0
    a_est[1][3] = 1.0
    a_est[2][2] = af
    a_est[3][3] = at

    b_est[2][0] = bf
    b_est[3][1] = bt

    return a_est, b_est
   
if __name__ == "__main__":

    dt = 1.0/250.0

    a_est, b_est = robot_identification("./data/run_1.txt", dt)
    
    print(a_est)
    print(b_est)