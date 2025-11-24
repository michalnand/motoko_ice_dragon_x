import numpy


class Trajectory:
    def __init__(self):
        f_name = "paths/map_02.txt"        

        # load data
        result_tmp = []
        with open(f_name, "r") as file:
            for line in file:
                y, x = line.split(",") 
                y = float(y)
                x = float(x)
                result_tmp.append([x, y])

        # insert helping points before straigth part
        result = []
        for n in range(len(result_tmp)-1):
            point = result_tmp[n]
            point_next = result_tmp[n+1]
            
            result.append(point)

            dx = point_next[0] - point[0]
            dy = point_next[1] - point[1]

            
            eps =  0.001
            if abs(dx) < eps or abs(dy) < eps:
                x = point[0] + 0.02*dx
                y = point[1] + 0.02*dy
                result.append([x, y])

                x = point[0] + 0.5*dx
                y = point[1] + 0.5*dy
                result.append([x, y])

                x = point[0] + 0.98*dx
                y = point[1] + 0.98*dy
                result.append([x, y])
            

        self.key_points = numpy.array(result, dtype=numpy.float32)
      
        
        n_key_points = self.key_points.shape[0]

        self.points = []

        
        n = 0
        while n < n_key_points:
            
            polynome_order = 4

            xp = numpy.zeros(polynome_order)
            yp = numpy.zeros(polynome_order)
            for i in range(polynome_order):
                xp[i] = self.key_points[(n+i)%n_key_points, 0]
                yp[i] = self.key_points[(n+i)%n_key_points, 1]

            t = numpy.zeros(polynome_order)
            t_acc = 0.0
            for i in range(len(xp)):
                t[i] = t_acc
                d = ((self.key_points[(n+i+0)%n_key_points] - self.key_points[(n+i+1)%n_key_points])**2).sum()**0.5
                t_acc+=d

            t = numpy.array(t)
           
            coeffs_x = numpy.polyfit(t, xp, len(xp)-1)
            coeffs_y = numpy.polyfit(t, yp, len(yp)-1)

            d_max = t[2] - t[1]
            d_max = 10*d_max
            
            for i in range(int(d_max)):
                p = i/d_max
                p = (1.0 - p)*t[1] + p*t[2]
                
                x = numpy.polyval(coeffs_x, p)
                y = numpy.polyval(coeffs_y, p)

                self.points.append([x, y])

            n+=1
        
       
        self.points = numpy.array(self.points)

        # convert from cm to m
        self.key_points = 0.01*numpy.array(self.key_points, dtype=numpy.float32)
        self.points     = 0.01*numpy.array(self.points, dtype=numpy.float32)

        # scaling information
        self.min_x = numpy.min(self.key_points[:, 0])
        self.max_x = numpy.max(self.key_points[:, 0])

        self.min_y = numpy.min(self.key_points[:, 1])
        self.max_y = numpy.max(self.key_points[:, 1])

      
    def _find_circle(self, x0, y0, x1, y1, x2, y2):
        # Form the matrices to solve the linear system
        A = numpy.array([
            [x0, y0, 1],
            [x1, y1, 1],
            [x2, y2, 1]
        ])
        
        B = numpy.array([
            [-(x0**2 + y0**2)],
            [-(x1**2 + y1**2)],
            [-(x2**2 + y2**2)]
        ])
        
        try:
            # Solving the system of linear equations A * [D, E, F].T = B
            D, E, F = numpy.linalg.solve(A, B)
            
            # Calculate the center (h, k) and the radius r
            xc = -D[0] / 2
            yc = -E[0] / 2
            r = numpy.sqrt(xc**2 + yc**2 - F[0])
        except:
            xc = 0
            yc = 0
            r   = 10**10
        
        return xc, yc, r
    
    def _line_fit(self, x0, y0, x1, y1, t):
        x = t*x0 + (1.0 - t)*x1
        y = t*y0 + (1.0 - t)*y1

        return x, y


    def _circle_fit(self, xc, yc, r, t):
        x = xc + r*numpy.cos(t)
        y = yc + r*numpy.sin(t)

        return x, y
    
    
    def _find_angle(self, x, y):
        
        theta = numpy.arctan2(y, x)

        return theta
        if theta < 0.0:
            theta += 2.0*numpy.pi

        return theta
    
    def _signed_angle(self, x0, y0, x1, y1, x, y):
        # Define vectors
        v1 = numpy.array([x1 - x0, y1 - y0])
        v2 = numpy.array([x - x0, y - y0])
        
        # Normalize vectors
        v1_normalized = v1 / numpy.linalg.norm(v1)
        v2_normalized = v2 / numpy.linalg.norm(v2)
        
        # Compute dot product and cross product
        dot_product = numpy.dot(v1_normalized, v2_normalized)
        cross_product = numpy.cross(v1_normalized, v2_normalized)
        
        # Compute the angle using arctan2
        angle = numpy.arctan2(cross_product, dot_product)
        
        return angle