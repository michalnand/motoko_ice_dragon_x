import numpy

def point_following(x_d,  a_d, dt = 1.0/250.0):

    v_max     = 1000.0**3
    w_max     = 3600.0*numpy.pi/180.0

    a_max     = 10000.0
    o_max     = 10000.0


    x = 0.0
    v = 100.0/dt

    a = 0.0
    w = 100.0/dt

    # Compute required velocity and angular rate to reach the desired state
    v_req = (x_d - x) / dt
    w_req = (a_d - a) / dt

    # Limit velocity and angular rate
    #v_req = numpy.clip(v_req, -v_max, v_max)
    #w_req = numpy.clip(w_req, -w_max, w_max)

    # Compute acceleration and angular acceleration limits
    delta_v = numpy.clip(v_req - v, -a_max * dt, a_max * dt)
    delta_w = numpy.clip(w_req - w, -o_max * dt, o_max * dt)

    # Update velocity and angular rate
    v_new = v + delta_v
    w_new = w + delta_w

    # Compute new reference position and angle using trapezoidal integration
    x_ref = x + 0.5 * (v + v_new) * dt
    a_ref = a + 0.5 * (w + w_new) * dt

    x_ref = x + v_new*dt 
    a_ref = a + w_new*dt

    print(x_ref, a_ref)




if __name__ == "__main__":

    point_following(1000, 90.0)