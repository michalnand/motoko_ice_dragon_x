import numpy

class RobotMotion:

    def __init__(self, distance_uncertainty = 0.01, theta_uncertainty = 0.01, n_landmarks = 10, random_landmarks = True):

        self.distance_uncertainty = distance_uncertainty
        self.theta_uncertainty    = theta_uncertainty
        
        # initial robot state
        self.x     = 0
        self.y     = 0
        self.theta = 0


        # create landmarks, random or circular placed
        if random_landmarks:
            self.landmarks = 2.0*numpy.random.rand(n_landmarks, 2) - 1.0
        else:
            self.landmarks = numpy.zeros((n_landmarks, 2))

            for i in range(n_landmarks):
                phi = i*numpy.pi*2/(n_landmarks-1)

                self.landmarks[i, 0] = numpy.cos(phi)
                self.landmarks[i, 1] = numpy.sin(phi)

        self.landmarks[0, 0]  = self.x
        self.landmarks[0, 1]  = self.y

        self.curr_landmark = 0


    def step(self, reaching_treshold = 0.02):
        
        x_target = self.landmarks[self.curr_landmark, 0]
        y_target = self.landmarks[self.curr_landmark, 1]

        d = ((y_target - self.y)**2 + (x_target - self.x)**2)**0.5

        velocity = 0.001 + 0.05*numpy.random.rand()

        dtheta     = numpy.arctan2(y_target - self.y, x_target - self.x)
        self.x     = self.x + velocity*numpy.cos(dtheta)
        self.y     = self.y + velocity*numpy.sin(dtheta)
        self.theta = self.theta + dtheta

        if d < reaching_treshold:
            landmark = self.curr_landmark

            self.curr_landmark = (self.curr_landmark + 1)%self.landmarks.shape[0]
        else:
            landmark = -1

        dDistance = velocity + self.distance_uncertainty*numpy.random.randn()
        dTheta    = dtheta   + self.theta_uncertainty*numpy.random.randn()


        return dDistance, dTheta, landmark