from Point import *

class Path_Point():
    def __init__(self, point, curvature, theta = 0, velocity = 0):
        self.x = point.x
        self.y = point.y
        self.curvature = curvature
        self.velocity = velocity
        self.theta = theta


