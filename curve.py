import numpy as np
from Pose import *
import matplotlib.pyplot as plt

def graph_path(path):
    """Graphs points of path using matplotlib"""
    for P in path:
        plt.scatter(P.x, P.y, color="black")

    plt.show()


def path_with_points(*points, initial_heading, final_heading=None):
    """Spline interpolation between waypoints using an arbitrary number of waypoints and initial/final heading."""

    # List which holds discrete points from the quintic beziér curves
    path = []

    # Converts initial and final heading to radians from degrees
    initial_heading *= (math.pi / 180)
    if final_heading is not None:
        final_heading *= (math.pi / 180)

    # Iterates through waypoints and creates path
    for curr in range(len(points) - 1):
        control_points = calc_control_points_with_points(curr, points, initial_heading, final_heading)
        for i in control_points:
            plt.scatter(i.x, i.y)
        # Change increment to change number of discrete points per curve
        for t in np.arange(0, 1, 0.01):
            p = quintic_bezier(control_points[0], control_points[1], control_points[2], control_points[3], control_points[4],
                               control_points[5], t)
            path.append(p)

    return path


def path_with_poses(*poses):
    """Spline interpolation between waypoints using an arbitrary number of poses."""

    # List which holds discrete points from quintic beziér curves
    path = []

    # Iterates through waypoints and creates path
    for curr in range(len(poses) - 1):
        control_points = calc_control_points_with_poses(curr, poses)
        for i in control_points:
            plt.scatter(i.x, i.y)
        for t in np.arange(0, 1, 0.01):
            p = quintic_bezier(control_points[0], control_points[1], control_points[2], control_points[3], control_points[4],
                               control_points[5], t)
            path.append(p)

    return path


def calc_control_points_with_poses(curr, path):
    """Calculates control points of quintic beziér curve if given an arbitrary number of poses."""

    # Unit vector expressing robot's initial and final heading. Used to calculate first derivative.
    v0 = Point(math.sin(path[curr].heading), math.cos(path[curr + 1].heading))
    v1 = Point(math.sin(path[curr + 1].heading), math.cos(path[curr + 1].heading))

    # If the current point is the starting point
    if curr == 0:
        # If the path length is only two points
        if len(path) == 2:
            # Calculate magnitude between nearest point and initial point.
            magnitudev0 = (1 / 2) * magnitude(path[curr + 1] - path[curr])

            # Multiply unit vector by magnitude to calculate first derivative of robot
            v0 *= magnitudev0
            v1 *= magnitudev0

            # Calculate second derivative
            acc0 = -6 * path[curr] - 4 * v0 - 2 * v1 + 6 * path[curr + 1]
            acc1 = 6 * path[curr] + 2 * v0 + 4 * v1 - 6 * path[curr + 1]

        else:
            magnitudev0 = (1 / 2) * magnitude(path[curr + 1] - path[curr])
            magnitudev1 = (1 / 2) * min(magnitude(path[curr + 1] - path[curr]),
                                        magnitude(path[curr + 2] - path[curr + 1]))
            v0 *= magnitudev0
            v1 *= magnitudev1

            # Calculate weight of each line segment
            alpha = (magnitude(path[curr + 2] - path[curr + 1])) / (
                        magnitude(path[curr + 1] - path[curr]) + magnitude(path[curr + 2] - path[curr + 1]))
            beta = (magnitude(path[curr + 1] - path[curr])) / (
                        magnitude(path[curr + 1] - path[curr]) + magnitude(path[curr + 2] - path[curr + 1]))

            acc0 = -6 * path[curr] - 4 * v0 - 2 * v1 + 6 * path[curr + 1]
            acc1 = alpha * (6 * path[curr] + 2 * v0 + 4 * v1 - 6 * path[curr + 1]) + beta * (
                        -6 * path[curr + 1] - 4 * v0 - 2 * v1 + 6 * path[curr + 2])

    # If the current point is the last point
    elif curr == len(path) - 2:
        magnitudev0 = (1 / 2) * min(magnitude(path[curr] - path[curr - 1]), magnitude(path[curr + 1] - path[curr]))
        magnitudev1 = (1 / 2) * magnitude(path[curr] - path[curr - 1])

        v0 *= magnitudev0
        v1 *= magnitudev1

        alpha = (magnitude(path[curr + 1] - path[curr])) / (
                    magnitude(path[curr] - path[curr - 1]) + magnitude(path[curr + 1] - path[curr]))
        beta = (magnitude(path[curr] - path[curr - 1])) / (
                    magnitude(path[curr] - path[curr - 1]) + magnitude(path[curr + 1] - path[curr]))

        acc0 = alpha * (6 * path[curr - 1] + 2 * v0 + 4 * v1 - 6 * path[curr]) + beta * (
                    -6 * path[curr] - 4 * v0 - 2 * v1 + 6 * path[curr + 1])
        acc1 = 6 * path[curr] + 2 * v0 + 4 * v1 - 6 * path[curr + 1]

    #If the current point is in the middle of the path
    else:
        magnitudev0 = (1 / 2) * min(magnitude(path[curr] - path[curr - 1]), magnitude(path[curr + 1] - path[curr]))
        magnitudev1 = (1 / 2) * min(magnitude(path[curr + 1] - path[curr]), magnitude(path[curr + 2] - path[curr + 1]))

        v0 *= magnitudev0
        v1 *= magnitudev1

        alpha0 = (magnitude(path[curr + 1] - path[curr])) / (
                    magnitude(path[curr] - path[curr - 1]) + magnitude(path[curr + 1] - path[curr]))
        beta0 = (magnitude(path[curr] - path[curr - 1])) / (
                    magnitude(path[curr] - path[curr - 1]) + magnitude(path[curr + 1] - path[curr]))
        alpha1 = (magnitude(path[curr + 2] - path[curr + 1])) / (
                    magnitude(path[curr + 1] - path[curr]) + magnitude(path[curr + 2] - path[curr + 1]))
        beta1 = (magnitude(path[curr + 1] - path[curr])) / (
                    magnitude(path[curr + 1] - path[curr]) + magnitude(path[curr + 2] - path[curr + 1]))

        acc0 = alpha0 * (6 * path[curr - 1] + 2 * v0 + 4 * v1 - 6 * path[curr]) + beta0 * (
                    -6 * path[curr] - 4 * v0 - 2 * v1 + 6 * path[curr + 1])
        acc1 = alpha1 * (6 * path[curr] + 2 * v0 + 4 * v1 - 6 * path[curr + 1]) + beta1 * (
                    -6 * path[curr + 1] - 4 * v0 - 2 * v1 + 6 * path[curr + 2])

    #Calculate control points using first and second derivatives
    point0 = path[curr]
    point5 = path[curr + 1]
    point1 = (1 / 5) * v0 + point0
    point2 = (1 / 20) * acc0 + 2 * point1 - point0
    point4 = point5 - (1 / 5) * v1
    point3 = (1 / 20) * acc1 + 2 * point4 - point5

    return point0, point1, point2, point3, point4, point5


def calc_control_points_with_points(curr, path, initial_heading, final_heading):
    """Calculates control points of quintic bezier curve if given an arbitrary number of points and
    initial/final heading. """

    if curr == 0:
        if len(path) == 2:
            magnitudev0 = (1 / 2) * magnitude(path[curr + 1] - path[curr])

            v0 = Point(math.sin(initial_heading), math.cos(initial_heading)) * magnitudev0

            # If final heading exists, using final heading to calculate first derivative. Otherwise, match direction
            # to straight line connection from previous point

            if final_heading is None:
                v1 = (path[curr + 1] - path[curr]) / (magnitude(path[curr + 1] - path[curr])) * magnitudev0
            else:
                v0 = Point(math.sin(final_heading), math.cos(final_heading)) * magnitudev0


            acc0 = -6 * path[curr] - 4 * v0 - 2 * v1 + 6 * path[curr + 1]
            acc1 = 6 * path[curr] + 2 * v0 + 4 * v1 - 6 * path[curr + 1]
        else:
            magnitudev0 = (1 / 2) * magnitude(path[curr + 1] - path[curr])

            magnitudev1 = (1 / 2) * min(magnitude(path[curr + 1] - path[curr]),
                                        magnitude(path[curr + 2] - path[curr + 1]))

            v0 = Point(math.sin(initial_heading), math.cos(initial_heading)) * magnitudev0

            # Calculate vector perpendicular to the angle bisector of surrounding points
            v1 = getPerpendicularVector(path[curr], path[curr + 1], path[curr + 2]) * magnitudev1

            alpha = (magnitude(path[curr + 2] - path[curr + 1])) / (
                        magnitude(path[curr + 1] - path[curr]) + magnitude(path[curr + 2] - path[curr + 1]))
            beta = (magnitude(path[curr + 1] - path[curr])) / (
                        magnitude(path[curr + 1] - path[curr]) + magnitude(path[curr + 2] - path[curr + 1]))

            acc0 = -6 * path[curr] - 4 * v0 - 2 * v1 + 6 * path[curr + 1]
            acc1 = alpha * (6 * path[curr] + 2 * v0 + 4 * v1 - 6 * path[curr + 1]) + beta * (
                        -6 * path[curr + 1] - 4 * v0 - 2 * v1 + 6 * path[curr + 2])

    elif curr == len(path) - 2:
        magnitudev0 = (1 / 2) * min(magnitude(path[curr] - path[curr - 1]), magnitude(path[curr + 1] - path[curr]))
        magnitudev1 = (1 / 2) * magnitude(path[curr] - path[curr - 1])

        v0 = getPerpendicularVector(path[curr - 1], path[curr], path[curr + 1]) * magnitudev0
        if final_heading is None:
            v1 = (path[curr + 1] - path[curr]) / (magnitude(path[curr + 1] - path[curr])) * magnitudev1
        else:
            v1 = Point(math.sin(final_heading), math.cos(final_heading)) * magnitudev0

        alpha = (magnitude(path[curr + 1] - path[curr])) / (
                    magnitude(path[curr] - path[curr - 1]) + magnitude(path[curr + 1] - path[curr]))
        beta = (magnitude(path[curr] - path[curr - 1])) / (
                    magnitude(path[curr] - path[curr - 1]) + magnitude(path[curr + 1] - path[curr]))

        acc0 = alpha * (6 * path[curr - 1] + 2 * v0 + 4 * v1 - 6 * path[curr]) + beta * (
                    -6 * path[curr] - 4 * v0 - 2 * v1 + 6 * path[curr + 1])
        acc1 = 6 * path[curr] + 2 * v0 + 4 * v1 - 6 * path[curr + 1]

    else:
        magnitudev0 = (1 / 2) * min(magnitude(path[curr] - path[curr - 1]), magnitude(path[curr + 1] - path[curr]))
        magnitudev1 = (1 / 2) * min(magnitude(path[curr + 1] - path[curr]), magnitude(path[curr + 2] - path[curr + 1]))

        v0 = getPerpendicularVector(path[curr - 1], path[curr], path[curr + 1]) * magnitudev0
        v1 = getPerpendicularVector(path[curr], path[curr + 1], path[curr + 2]) * magnitudev1

        alpha0 = (magnitude(path[curr + 1] - path[curr])) / (
                    magnitude(path[curr] - path[curr - 1]) + magnitude(path[curr + 1] - path[curr]))
        beta0 = (magnitude(path[curr] - path[curr - 1])) / (
                    magnitude(path[curr] - path[curr - 1]) + magnitude(path[curr + 1] - path[curr]))
        alpha1 = (magnitude(path[curr + 2] - path[curr + 1])) / (
                    magnitude(path[curr + 1] - path[curr]) + magnitude(path[curr + 2] - path[curr + 1]))
        beta1 = (magnitude(path[curr + 1] - path[curr])) / (
                    magnitude(path[curr + 1] - path[curr]) + magnitude(path[curr + 2] - path[curr + 1]))

        acc0 = alpha0 * (6 * path[curr - 1] + 2 * v0 + 4 * v1 - 6 * path[curr]) + beta0 * (
                    -6 * path[curr] - 4 * v0 - 2 * v1 + 6 * path[curr + 1])
        acc1 = alpha1 * (6 * path[curr] + 2 * v0 + 4 * v1 - 6 * path[curr + 1]) + beta1 * (
                    -6 * path[curr + 1] - 4 * v0 - 2 * v1 + 6 * path[curr + 2])

    point0 = path[curr]
    point5 = path[curr + 1]
    point1 = (1 / 5) * v0 + point0
    point2 = (1 / 20) * acc0 + 2 * point1 - point0
    point4 = point5 - (1 / 5) * v1
    point3 = (1 / 20) * acc1 + 2 * point4 - point5

    return point0, point1, point2, point3, point4, point5


def quintic_bezier(point0, point1, point2, point3, point4, point5, t):
    """Returns point in quintic bezier curve. """
    p = ((1 - t) ** 5) * point0 + (5 * ((1 - t) ** 4) * t * point1) + (10 * ((1 - t) ** 3) * (t ** 2) * point2) + (
                10 * ((1 - t) ** 2) * (t ** 3) * point3 + 5 * (1 - t) * (t ** 4) * point4) + ((t ** 5) * point5)

    return p

def lerp(point0, point1, t):

    return (1 - t) * point0 + t * point1

