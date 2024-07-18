import numpy


class Trajectory:
    def __init__(self):
        f_name = "paths/map_01.txt"        

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
          
            eps = 0.001
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
            
        # convert to mm
        self.key_points = 10.0*numpy.array(result, dtype=numpy.float32)

        # scaling information
        self.min_x = numpy.min(self.key_points[:, 0])
        self.max_x = numpy.max(self.key_points[:, 0])

        self.min_y = numpy.min(self.key_points[:, 1])
        self.max_y = numpy.max(self.key_points[:, 1])

        
        n_key_points = self.key_points.shape[0]

        self.points = []

        n = 0
        while n < n_key_points:
            
            polynome_order = 3

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

            n+= 1

        self.points = numpy.array(self.points)
    


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
    
    def _find_angle(self, x, y):
        
        theta = numpy.arctan2(y, x)
        if theta < 0.0:
            theta += 2.0*numpy.pi

        return theta