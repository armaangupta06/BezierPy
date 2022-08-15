from Point import *

class Pose(Point):
    def __init__(self, x, y, heading):
        Point.__init__(self, x, y)
        self.heading = heading * math.pi / 180
