import numpy
import cv2


class Render:
    def __init__(self, window_size, max_position_range = 1.0): 

              
        self.window_size        = window_size
        self.max_position_range = max_position_range

        self.video_writer = None
       
    
    def render(self, trajectory, robot, target_state, robot_path = None):

        padding    = 100.0

        self.min_x = trajectory.min_x - padding
        self.max_x = trajectory.max_x + padding
        self.min_y = trajectory.min_y - padding
        self.max_y = trajectory.max_y + padding

        self.window_width  = int(800)
        self.window_height = int(0.75*self.window_width)

        result_image = numpy.zeros((self.window_height, self.window_width, 3), dtype=numpy.float32)
        
       
        # plot line
        n_points = trajectory.points.shape[0]
        for n in range(n_points):
            if n%20 == 0:
                x0 = trajectory.points[n][0]
                y0 = trajectory.points[n][1]
                x0, y0 = self._scale_position(x0, y0)

                cv2.circle(result_image, (x0, y0), 2, (0.9, 0.9, 0.9), -1)
            

        # plot line key points
        n_keypoints = trajectory.key_points.shape[0]
        for n in range(n_keypoints):
              
            x0 = trajectory.key_points[n][0]
            y0 = trajectory.key_points[n][1]
            x0, y0 = self._scale_position(x0, y0)

            x1 = trajectory.key_points[(n+1)%n_keypoints][0]
            y1 = trajectory.key_points[(n+1)%n_keypoints][1]
            x1, y1 = self._scale_position(x1, y1)

            #cv2.line(result_image, (x0, y0), (x1, y1), (1, 1, 1), 1)
            cv2.circle(result_image, (x0, y0), 3, (0.5, 0.5, 0.5), -1)

        # plot robot
        self._plot_robot(result_image, robot)

        
        # plot grid, 100mm
        for y in range(0, round(self.max_y), 100):
            for x in range(0, round(self.max_x), 100):
                
                x0, y0 = self._scale_position(x, y)
                cv2.circle(result_image, (x0, y0), 2, (0.0, 0.9, 0.0), -1)
        
        # plot target point
        x0 = target_state[0][0]
        y0 = target_state[1][0]
        x0, y0 = self._scale_position(x0, y0)
        cv2.circle(result_image, (x0, y0), 10, (1, 0, 0), -1)

        if robot_path is not None:
            for n in range(len(robot_path) - 1):
                x0 = robot_path[n][0]
                y0 = robot_path[n][1]
                x0, y0 = self._scale_position(x0, y0)

                x1 = robot_path[n+1][0]
                y1 = robot_path[n+1][1]
                x1, y1 = self._scale_position(x1, y1)

                cv2.line(result_image, (x0, y0), (x1, y1), (0, 0, 1), 2)
        

        cv2.imshow("visualisation", result_image)
        cv2.waitKey(1)

        '''
        if self.video_writer is None:
            fourcc = cv2.VideoWriter_fourcc(*'XVID') 
            self.video_writer = cv2.VideoWriter("video.mp4", fourcc, 50.0, (self.window_width, self.window_height)) 
        else:
            result_image = numpy.array(255*result_image, dtype=numpy.uint8)
            self.video_writer.write(result_image) 
        '''
     
    def _scale_position(self, x, y):
        x_out = self._scale(x, self.min_x, self.max_x, 0, self.window_width)
        y_out = self.window_height - self._scale(y, self.min_y, self.max_y, 0, self.window_height)

        x_out = numpy.clip(x_out, 0, self.window_width-1)
        y_out = numpy.clip(y_out, 0, self.window_height-1)

        return int(x_out), int(y_out)
    
    def _scale(self, x, x_min, x_max, y_min, y_max):
        k = (y_max - y_min)/(x_max - x_min)
        q = y_max - k*x_max

        return k*x + q
    

    def _plot_robot(self, result_image, robot):
        
        dynamics_state, position_state = robot.get_state()

        x0  = position_state[0][0] - robot.width/2
        y0  = position_state[1][0] - robot.height/2
        x1  = position_state[0][0] + robot.width/2
        y1  = position_state[1][0] + robot.height/2

        theta = -position_state[2][0]*180.0/numpy.pi

        x0, y0 = self._scale_position(x0, y0)
        x1, y1 = self._scale_position(x1, y1)

        center_x = (x0 + x1) // 2
        center_y = (y0 + y1) // 2   
        width = abs(x1 - x0)
        height = abs(y1 - y0)

        rect = ((center_x, center_y), (width, height), theta)

        # Get the 4 vertices of the rotated rectangle
        box = cv2.boxPoints(rect)
        box = numpy.int0(box)  # Convert to integer

        # Draw the filled rotated rectangle
        result_image = cv2.fillPoly(result_image, [box], (0, 0, 1))  # Fill color is green

        return result_image

