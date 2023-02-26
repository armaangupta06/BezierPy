from Point import *

import numpy as np
class Quintic_Bezier:
    def __init__(self, point0, point1, point2, point3, point4, point5):
        self.point0 = point0
        self.point1 = point1
        self.point2 = point2
        self.point3 = point3
        self.point4 = point4
        self.point5 = point5

    def get_control_points(self):
        return self.point0, self.point1, self.point2, self.point3, self.point4, self.point5

    def calc_first_derivative(self, t):
        p = ((5 * ((1 - t) ** 4) * (self.point1 - self.point0)) + (20 * t * ((1 - t) ** 3) * (self.point2 - self.point1)) +
            (30 * (t ** 2) * ((1 - t) ** 2) * (self.point3 - self.point2)) + (20 * (t ** 3) * (1 - t) * (self.point4 - self.point3)) +
            (5 * (t ** 4) * (self.point5 - self.point4)))

        return p

    def calc_second_derivative(self, t):
        p = ((20 * ((1 - t) ** 3) * (self.point2 - (2 * self.point1) + self.point0)) +
             (60 * t * ((1 - t) ** 2) * (self.point3 - (2 * self.point2) + self.point1)) +
             (60 * (t ** 2) * (1 - t) * (self.point4 - (2 * self.point3) + self.point2) +
              (20 * (t ** 3) * (self.point5 - (2 * self.point4) + self.point3))))

        return p


    def calc_curvature(self, t):
        # print("d", (determinant(self.calc_first_derivative(t), self.calc_second_derivative(t))))
        if magnitude(self.calc_first_derivative(t)) == 0:
            return 0
        return (determinant(self.calc_first_derivative(t), self.calc_second_derivative(t)))/(magnitude(self.calc_first_derivative(t)) ** 3)

    def get_point(self, t):
        """Returns point in quintic bezier curve. """
        p = (((1 - t) ** 5) * self.point0 + (5 * ((1 - t) ** 4) * t * self.point1) + (10 * ((1 - t) ** 3) * (t ** 2) * self.point2) +
             (10 * ((1 - t) ** 2) * (t ** 3) * self.point3 + 5 * (1 - t) * (t ** 4) * self.point4) + ((t ** 5) * self.point5))

        return p

    def calc_arc_length(self):
        distance = 0
        prev_point = self.get_point(0);
        for t in np.arange(0, 1.01, 0.01):
            curr_point = self.get_point(t)
            distance += distance_formula(curr_point, prev_point)
            prev_point = curr_point
        return distance

