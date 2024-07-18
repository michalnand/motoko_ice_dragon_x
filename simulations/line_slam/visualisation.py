import numpy
import cv2

class Visualisation:

    def __init__(self, size = 700, max_steps = 300):
        
        
        self.window_size = size
        self.max_steps   = max_steps

        self.robot_est   = []

        


    def step(self, robot, slam = None):
        result_image = numpy.zeros((self.window_size, self.window_size, 3), dtype=numpy.float32)
        
        # plot landmarks
        n_landmarks = robot.landmarks.shape[0]
        for n in range(n_landmarks):
            x = robot.landmarks[n][0]
            y = robot.landmarks[n][1]

            x = self._scale_position(x)
            y = self._scale_position(y)

            cv2.circle(result_image, (x, y), 6, (0, 0, 1), -1)

        # plot groud truth route
        for n in range(n_landmarks):

            x0 = robot.landmarks[n][0]
            y0 = robot.landmarks[n][1]
            x0 = self._scale_position(x0)
            y0 = self._scale_position(y0)

            x1 = robot.landmarks[(n+1)%n_landmarks][0]
            y1 = robot.landmarks[(n+1)%n_landmarks][1]
            x1 = self._scale_position(x1)
            y1 = self._scale_position(y1)
        
            cv2.line(result_image, (x0, y0), (x1, y1), (0, 0, 1), 1)

        # plot mapped landmarks
        for n in range(slam.landmarks_mu.shape[0]):
            x = slam.landmarks_mu[n][0]
            y = slam.landmarks_mu[n][1]

            if numpy.abs(x) < 10**6 and numpy.abs(y) < 10**6:
                x = self._scale_position(x)
                y = self._scale_position(y)

                r = self._scale_var(slam.landmarks_var[n])

                cv2.circle(result_image, (x, y), r, (0, 0.25, 0), -1)

                cv2.circle(result_image, (x, y), 9, (0, 1, 0), -1)

            
        

        # plot robot current position
        x = robot.x
        y = robot.y
        x = self._scale_position(x)
        y = self._scale_position(y)

        r = self._scale_var(slam.robot_var)
        cv2.circle(result_image, (x, y), r, (0.25, 0.25, 0.25), -1)
        cv2.circle(result_image, (x, y), 10, (1, 1, 1), -1)

        if len(self.robot_est) >= self.max_steps:
            self.robot_est.pop(0)    
        self.robot_est.append([slam.x_est, slam.y_est])

        # plot robot trajectory
        for n in range(len(self.robot_est)):
            x = self._scale_position(self.robot_est[n][0])
            y = self._scale_position(self.robot_est[n][1])
            cv2.circle(result_image, (x, y), 2, (0, 1, 0), -1)
        
        cv2.imshow("visualisation", result_image)
        cv2.waitKey(1)

    def _scale_position(self, x):

        result = self.window_size*(x + 1.0)/2.0

        result = int(numpy.clip(result, 0, self.window_size))
        return result
    
  
    def _scale_var(self, x, scale = 100):
        result = scale*x

        result = int(numpy.clip(result, 0, 50))

        return result