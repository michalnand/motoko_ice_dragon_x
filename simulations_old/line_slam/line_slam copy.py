import numpy

class LineSlam:

    def __init__(self, distance_uncertainty, theta_uncertainty, max_landmarks = 10):

        self.distance_uncertainty   = distance_uncertainty
        self.theta_uncertainty      = theta_uncertainty
        
        self.curr_land_mark         = -1

        self.landmarks_mu           = numpy.zeros((max_landmarks, 2)) + 10**10
        self.landmarks_var          = (10**10)*numpy.ones((max_landmarks, 1))

        self.accumulated_distance_uncertainty = 0
        self.accumulated_theta_uncertainty    = 0

        self.visited_landmars = []

        self.x_est     = 0
        self.y_est     = 0
        self.theta_est = 0

    def step(self, dDistance, dTheta, landmark):
        
        # odometry position estimation
        self.x_est = self.x_est + dDistance*numpy.cos(dTheta)
        self.y_est = self.y_est + dDistance*numpy.sin(dTheta)
        self.theta_est = self.theta_est + dTheta

        # accumulate uncertainties
        self.accumulated_distance_uncertainty = self.accumulated_distance_uncertainty + self.distance_uncertainty
        self.accumulated_theta_uncertainty    = self.accumulated_theta_uncertainty    + self.theta_uncertainty
    

        if landmark != -1 and self.curr_land_mark != landmark:
            self.curr_land_mark = landmark

            self.landmarks_mu[self.curr_land_mark][0] = self.x_est
            self.landmarks_mu[self.curr_land_mark][1] = self.y_est

            # check if robot returned on previously visited landmark
            if landmark in self.visited_landmars:
                self._loop_closure(landmark)
                self.visited_landmars = []
            else:
                if len(self.visited_landmars) > 100:
                    self.visited_landmars.pop(0)
                self.visited_landmars.append(landmark)


    def _loop_closure(self, landmark):
        pass

