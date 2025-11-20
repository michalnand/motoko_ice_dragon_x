import numpy

class LineSlam:

    def __init__(self, distance_var, theta_var, max_landmarks = 100, distance_treshold = 0.2):

        self.distance_var   = distance_var
        self.theta_var      = theta_var
        self.distance_treshold      = distance_treshold
        
        self.landmarks_mu           = (10**10)*numpy.ones((max_landmarks, 2))
        self.landmarks_var          = (10**3)*numpy.ones((max_landmarks))
        self.curr_land_mark_ptr     = 0

        self.robot_var = 0.0


        self.path = []

        self.x_est     = 0
        self.y_est     = 0
        self.theta_est = 0

    def step(self, dDistance, dTheta, landmark):
        
        # odometry position estimation
        self.x_est     = self.x_est + dDistance*numpy.cos(dTheta)
        self.y_est     = self.y_est + dDistance*numpy.sin(dTheta)
        self.theta_est = self.theta_est + dTheta

        # accumulated variance
        self.robot_var+= self.distance_var + (dDistance**2)*self.theta_var

        # robot incomes to landmark
        if landmark != -1:
            # find closest
            min_idx, min_dist = self._closest_landmark(self.x_est, self.y_est)

            # add new landmark position if not close enough
            if min_dist > self.distance_treshold:
                self._add_landmark(self.x_est, self.y_est, self.robot_var)
                min_idx  = self.curr_land_mark_ptr - 1 
                min_dist = 0
          
            # loop closure detection
            if min_idx not in self.path:
                self.path.append(min_idx)
            else:
                self._loop_closure(self.path, min_idx)

                self.path.clear()   
                self.path.append(min_idx)

            # correction
            lm_var         = self.landmarks_var[min_idx]
            self.x_est     = (self.x_est*self.robot_var + lm_var*self.landmarks_mu[min_idx][0])/(self.robot_var + lm_var)
            self.y_est     = (self.y_est*self.robot_var + lm_var*self.landmarks_mu[min_idx][1])/(self.robot_var + lm_var)
            self.robot_var = 2*(self.robot_var*lm_var)/(self.robot_var + lm_var)


    def _closest_landmark(self, x_est, y_est):
        pos = numpy.array([[x_est, y_est]])

        dist = self.landmarks_mu - pos
        dist = (dist**2).sum(axis=-1)**0.5

        min_idx  = numpy.argmin(dist)
        min_dist = dist[min_idx]

        return min_idx, min_dist
    
    def _add_landmark(self, x_est, y_est, variance):
        self.landmarks_mu[self.curr_land_mark_ptr][0] = x_est
        self.landmarks_mu[self.curr_land_mark_ptr][1] = y_est
        self.landmarks_var[self.curr_land_mark_ptr]   = variance

        self.curr_land_mark_ptr = self.curr_land_mark_ptr+1

    def _loop_closure(self, path, min_idx):
        print("loop closure at ", path, min_idx)

        '''
        # Normalize the accumulated uncertainties
        distance_weights = accumulated_distance_uncertainty / accumulated_distance_uncertainty[-1]
        theta_weights    = accumulated_theta_uncertainty / accumulated_theta_uncertainty[-1]
        
        # Calculate the drift
        drift_x = x[-1] - x[0]
        drift_y = y[-1] - y[0]
        drift_theta = theta[-1] - theta[0]
        
        # Apply non-uniform correction based on the respective weights
        x_corrected = x - distance_weights * drift_x
        y_corrected = y - distance_weights * drift_y
        '''

