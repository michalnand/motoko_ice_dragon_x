import numpy


class Trajectory:
    def __init__(self, n_points, n_key_points, max_range):
        
        self.min_x = numpy.min(self.key_points[:, 0])
        self.max_x = numpy.max(self.key_points[:, 0])

        self.min_y = numpy.min(self.key_points[:, 1])
        self.max_y = numpy.max(self.key_points[:, 1])

        self.points = numpy.zeros((n_points, 2), dtype=numpy.float32)

        r = 0.8
        dr_max = 100.0/n_points
        for n in range(n_points):
            phi = (2.0*numpy.pi*n)/(n_points - 1)
            
            r = r + dr_max*(2.0*numpy.random.rand()-1.0)
            r = numpy.clip(r, 0.0, max_range)

            x = r*numpy.cos(phi)
            y = r*numpy.sin(phi)

            self.points[n][0] = x
            self.points[n][1] = y


  
