import numpy as np
from Pose import *
import matplotlib.pyplot as plt
from Quintic_Bezier import Quintic_Bezier
from Path_Point import *

def graph_path(data):
    """Graphs points of path using matplotlib"""

    fig, axis = plt.subplots(3)

    for point in data:
        axis[0].scatter(point.x, point.y, color="black")

    axis[0].set_title("Points")

    for t in range(len(data)):
        # print(C, data[1][C])
        axis[1].scatter(t, data[t].curvature, color="black")
        axis[2].scatter(t, data[t].velocity, color="black")


    axis[1].set_title("Curvature")
    axis[2].set_title("Velocity")
    axis[2].set_ylim([0, 115])
    plt.show()


def path_with_points(*points, initial_heading, final_heading=None, v = 100, a = 10, tangent_magnitude = 1/2):
    """Spline interpolation between waypoints using an arbitrary number of waypoints and initial/final heading."""

    # List which holds discrete points from the quintic beziér curves
    path = []

    # Converts initial and final heading to radians from degrees
    initial_heading *= (math.pi / 180)
    if final_heading is not None:
        final_heading *= (math.pi / 180)

    # Iterates through waypoints and creates path
    for curr in range(len(points) - 1):
        curve = calc_bezier_curve_with_points(curr, points, initial_heading, final_heading, tangent_magnitude)

        # Change increment to change number of discrete points per curve
        for t in np.arange(0, 1.01, 0.01):
            p = curve.get_point(t)
            w = curve.calc_curvature(t)
            path.append(Path_Point(curve.get_point(t), curve.calc_curvature(t)))

    path = calc_velocity(path, v, a)
    return path

def path_with_poses(*poses, v = 100, a = 10, tangent_magnitude = 1/2):
    """Spline interpolation between waypoints using an arbitrary number of poses."""

    # List which holds discrete points from quintic beziér curves
    path = []

    # Iterates through waypoints and creates path
    for curr in range(len(poses) - 1):
        curve = calc_bezier_curve_with_poses(curr, poses, tangent_magnitude=tangent_magnitude)
        for t in np.arange(0, 1, 0.01):
            p = curve.get_point(t)
            w = curve.calc_curvature(t)
            path.append(Path_Point(curve.get_point(t), curve.calc_curvature(t)))

    path = calc_velocity(path, v, a)
    return path


def calc_velocity(path, v, a):
    """Calculates velocity profile of path with max velocity and max acceleration."""
    if len(path) == 0:
        return path
    path[-1].velocity = 0
    for i in range(len(path)-2, -1, -1):
        if path[i].curvature == 0:
            curv = 0.001
        else:
            curv = path[i].curvature
        desired_velocity = min(v, 3.0/abs(curv))
        distance = distance_formula(path[i+1], path[i])
        limited_velocity = math.sqrt(path[i+1].velocity ** 2 + 2 * a * distance)

        path[i].velocity = min(desired_velocity, limited_velocity)

    return path


def calc_bezier_curve_with_poses(curr, path, tangent_magnitude):
    """Calculates control points of quintic bezier curve if given an arbitrary number of points and
    initial/final heading. """

    # Unit vector expressing robot's initial and final heading. Used to calculate first and second derivatives
    v0 = Point(math.sin(path[curr].heading), math.cos(path[curr].heading))
    v1 = Point(math.sin(path[curr + 1].heading), math.cos(path[curr + 1].heading))

    # If current point is the first point of the path
    if curr == 0:

        #If the path is only two points long
        if len(path) == 2:
            magnitudev0 = tangent_magnitude * magnitude(path[curr + 1] - path[curr])

            # Multiply unit vector by magnitude to calculate first derivative of robot
            v0 *= magnitudev0
            v1 *= magnitudev0

            acc0 = -6 * path[curr] - 4 * v0 - 2 * v1 + 6 * path[curr + 1]
            acc1 = 6 * path[curr] + 2 * v0 + 4 * v1 - 6 * path[curr + 1]
        else:

            #Calculate magnitude of tangent vectors. Equal to tangent_magnitude * the minimum of the distance to
            #the neighboring points.

            if curr + 3 == len(path):
                magnitudevD = tangent_magnitude * magnitude(path[curr + 2] - path[curr + 1])
                vD = Point(math.sin(path[curr+2].heading), math.cos(path[curr+2].heading)) * magnitudevD
            else:
                magnitudevD = tangent_magnitude * min(magnitude(path[curr + 2] - path[curr + 1]),
                                                      magnitude(path[curr + 3] - path[curr + 2]))
                vD = Point(math.sin(path[curr+2].heading), math.cos(path[curr+2].heading)) * magnitudevD

            magnitudev0 = tangent_magnitude * magnitude(path[curr + 1] - path[curr])
            magnitudev1 = tangent_magnitude * min(magnitude(path[curr + 1] - path[curr]),
                                        magnitude(path[curr + 2] - path[curr + 1]))
            v0 *= magnitudev0
            v1 *= magnitudev1

            alpha = (magnitude(path[curr + 2] - path[curr + 1])) / (
                        magnitude(path[curr + 1] - path[curr]) + magnitude(path[curr + 2] - path[curr + 1]))
            beta = (magnitude(path[curr + 1] - path[curr])) / (
                        magnitude(path[curr + 1] - path[curr]) + magnitude(path[curr + 2] - path[curr + 1]))

            #Calculate second derivative
            acc0 = -6 * path[curr] - 4 * v0 - 2 * v1 + 6 * path[curr + 1]
            acc1 = alpha * (6 * path[curr] + 2 * v0 + 4 * v1 - 6 * path[curr + 1]) + beta * (
                    -6 * path[curr + 1] - 4 * v1 - 2 * vD + 6 * path[curr + 2])

    # If point is second to last point on path
    elif curr == len(path) - 2:
        if curr - 1 == 0:
            magnitudevA = tangent_magnitude * magnitude(path[curr] - path[curr-1])
            vA = Point(math.sin(path[curr-1].heading), math.cos(path[curr-1].heading)) * magnitudevA
        else:
            magnitudevA = tangent_magnitude * min(magnitude(path[curr-2] - path[curr - 1]), magnitude(path[curr] - path[curr - 1]))
            vA = Point(math.sin(path[curr-1].heading), math.cos(path[curr-1].heading)) * magnitudevA

        magnitudev0 = tangent_magnitude * min(magnitude(path[curr] - path[curr - 1]), magnitude(path[curr + 1] - path[curr]))
        magnitudev1 = tangent_magnitude * magnitude(path[curr+1] - path[curr])

        v0 *= magnitudev0
        v1 *= magnitudev1

        alpha = (magnitude(path[curr + 1] - path[curr])) / (
                    magnitude(path[curr] - path[curr - 1]) + magnitude(path[curr + 1] - path[curr]))
        beta = (magnitude(path[curr] - path[curr - 1])) / (
                    magnitude(path[curr] - path[curr - 1]) + magnitude(path[curr + 1] - path[curr]))

        acc0 = alpha * (6 * path[curr - 1] + 2 * vA + 4 * v0 - 6 * path[curr]) + beta * (
                    -6 * path[curr] - 4 * v0 - 2 * v1 + 6 * path[curr + 1])
        acc1 = 6 * path[curr] + 2 * v0 + 4 * v1 - 6 * path[curr + 1]

    #If point is in the middle of the path
    else:
        if curr - 1 == 0:
            magnitudevA = tangent_magnitude * magnitude(path[curr] - path[curr-1])
            vA = Point(math.sin(path[curr-1].heading), math.cos(path[curr-1].heading)) * magnitudevA
        else:
            magnitudevA = tangent_magnitude * min(magnitude(path[curr-2] - path[curr - 1]), magnitude(path[curr] - path[curr - 1]))
            vA = Point(math.sin(path[curr-1].heading), math.cos(path[curr-1].heading)) * magnitudevA

        # If two points ahead is last point on path
        if curr + 2 == len(path) - 1:
            magnitudevD = tangent_magnitude * magnitude(path[curr + 2] - path[curr + 1])
            vD = Point(math.sin(path[curr + 2].heading), math.cos(path[curr + 2].heading)) * magnitudevD
        else:
            magnitudevD = tangent_magnitude * min(magnitude(path[curr + 2] - path[curr + 1]),
                                                  magnitude(path[curr + 3] - path[curr + 2]))
            vD = Point(math.sin(path[curr + 2].heading), math.cos(path[curr + 2].heading)) * magnitudevD

        magnitudev0 = tangent_magnitude * min(magnitude(path[curr] - path[curr - 1]), magnitude(path[curr + 1] - path[curr]))
        magnitudev1 = tangent_magnitude * min(magnitude(path[curr + 1] - path[curr]), magnitude(path[curr + 2] - path[curr + 1]))

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

        acc0 = alpha0 * (6 * path[curr - 1] + 2 * vA + 4 * v0 - 6 * path[curr]) + beta0 * (
                    -6 * path[curr] - 4 * v0 - 2 * v1 + 6 * path[curr + 1])
        acc1 = alpha1 * (6 * path[curr] + 2 * v0 + 4 * v1 - 6 * path[curr + 1]) + beta1 * (
                    -6 * path[curr + 1] - 4 * v1 - 2 * vD + 6 * path[curr + 2])

    #Calculate control points based on first and second derivative
    point0 = path[curr]
    point5 = path[curr + 1]
    point1 = (1 / 5) * v0 + point0
    point2 = (1 / 20) * acc0 + 2 * point1 - point0
    point4 = point5 - (1 / 5) * v1
    point3 = (1 / 20) * acc1 + 2 * point4 - point5

    #Return Quintic Bezier for current point
    return Quintic_Bezier(point0, point1, point2, point3, point4, point5)


def calc_bezier_curve_with_points(curr, path, initial_heading, final_heading, tangent_magnitude):
    """Calculates control points of quintic bezier curve if given an arbitrary number of points and
    initial/final heading. """

    if curr == 0:
        if len(path) == 2:
            magnitudev0 = tangent_magnitude * magnitude(path[curr + 1] - path[curr])

            v0 = Point(math.sin(initial_heading), math.cos(initial_heading)) * magnitudev0

            # If final heading exists, using final heading to calculate first derivative. Otherwise, match direction
            # to straight line connection from previous point

            if final_heading is None:
                v1 = ((path[curr + 1] - path[curr]) / (magnitude(path[curr + 1] - path[curr]))) * magnitudev0
            else:
                v1 = Point(math.sin(final_heading), math.cos(final_heading)) * magnitudev0

            acc0 = -6 * path[curr] - 4 * v0 - 2 * v1 + 6 * path[curr + 1]
            acc1 = 6 * path[curr] + 2 * v0 + 4 * v1 - 6 * path[curr + 1]
        else:

            if curr + 2 == len(path) - 1:
                magnitudevD = tangent_magnitude * magnitude(path[curr + 2] - path[curr + 1])
                if final_heading is None:
                    vD = (path[curr + 2] - path[curr + 1]) / (magnitude(path[curr + 2] - path[curr + 1])) * magnitudevD
                else:
                    vD = Point(math.sin(final_heading), math.cos(final_heading)) * magnitudevD
            else:
                magnitudevD = tangent_magnitude * min(magnitude(path[curr + 2] - path[curr + 1]),
                                                      magnitude(path[curr + 3] - path[curr + 2]))
                vD = getPerpendicularVector(path[curr + 1], path[curr + 2], path[curr + 3]) * magnitudevD

            magnitudev0 = tangent_magnitude * magnitude(path[curr + 1] - path[curr])
            magnitudev1 = tangent_magnitude * min(magnitude(path[curr + 1] - path[curr]),
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
                    -6 * path[curr + 1] - 4 * v1 - 2 * vD + 6 * path[curr + 2])

    elif curr == len(path) - 2:
        if curr - 1 == 0:
            magnitudevA = tangent_magnitude * magnitude(path[curr] - path[curr-1])
            vA = Point(math.sin(initial_heading), math.cos(initial_heading)) * magnitudevA
        else:

            magnitudevA = tangent_magnitude * min(magnitude(path[curr-2] - path[curr - 1]), magnitude(path[curr] - path[curr - 1]))
            vA = getPerpendicularVector(path[curr-2], path[curr - 1], path[curr]) * magnitudevA


        magnitudev0 = tangent_magnitude * min(magnitude(path[curr] - path[curr - 1]), magnitude(path[curr + 1] - path[curr]))
        magnitudev1 = tangent_magnitude * magnitude(path[curr+1] - path[curr])

        v0 = getPerpendicularVector(path[curr - 1], path[curr], path[curr + 1]) * magnitudev0
        if final_heading is None:
            v1 = (path[curr + 1] - path[curr]) / (magnitude(path[curr + 1] - path[curr])) * magnitudev1
        else:
            v1 = Point(math.sin(final_heading), math.cos(final_heading)) * magnitudev0

        alpha = (magnitude(path[curr + 1] - path[curr])) / (
                    magnitude(path[curr] - path[curr - 1]) + magnitude(path[curr + 1] - path[curr]))
        beta = (magnitude(path[curr] - path[curr - 1])) / (
                    magnitude(path[curr] - path[curr - 1]) + magnitude(path[curr + 1] - path[curr]))
        acc0 = alpha * (6 * path[curr - 1] + 2 * vA + 4 * v0 - 6 * path[curr]) + beta * (
                    -6 * path[curr] - 4 * v0 - 2 * v1 + 6 * path[curr + 1])
        acc1 = 6 * path[curr] + 2 * v0 + 4 * v1 - 6 * path[curr + 1]

    else:

        if curr - 1 == 0:
            magnitudevA = tangent_magnitude * magnitude(path[curr] - path[curr-1])
            vA = Point(math.sin(initial_heading), math.cos(initial_heading)) * magnitudevA
        else:
            magnitudevA = tangent_magnitude * min(magnitude(path[curr-2] - path[curr - 1]), magnitude(path[curr] - path[curr - 1]))
            vA = getPerpendicularVector(path[curr-2], path[curr - 1], path[curr]) * magnitudevA

        # If two points ahead is last point on path
        if curr + 2 == len(path) - 1:
            magnitudevD = tangent_magnitude * magnitude(path[curr + 2] - path[curr + 1])
            if final_heading is None:
                vD = (path[curr + 2] - path[curr+1]) / (magnitude(path[curr + 2] - path[curr+1])) * magnitudevD
            else:
                vD = Point(math.sin(final_heading), math.cos(final_heading)) * magnitudevD
        else:
            magnitudevD = tangent_magnitude * min(magnitude(path[curr + 2] - path[curr + 1]), magnitude(path[curr + 3] - path[curr + 2]))
            vD = getPerpendicularVector(path[curr + 1], path[curr + 2], path[curr + 3]) * magnitudevD

        magnitudev0 = tangent_magnitude * min(magnitude(path[curr] - path[curr - 1]), magnitude(path[curr + 1] - path[curr]))
        magnitudev1 = tangent_magnitude * min(magnitude(path[curr + 1] - path[curr]), magnitude(path[curr + 2] - path[curr + 1]))

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

        acc0 = alpha0 * (6 * path[curr - 1] + 2 * vA + 4 * v0 - 6 * path[curr]) + beta0 * (
                    -6 * path[curr] - 4 * v0 - 2 * v1 + 6 * path[curr + 1])
        acc1 = alpha1 * (6 * path[curr] + 2 * v0 + 4 * v1 - 6 * path[curr + 1]) + beta1 * (
                    -6 * path[curr + 1] - 4 * v1 - 2 * vD + 6 * path[curr + 2])


    point0 = path[curr]
    point5 = path[curr + 1]
    point1 = (1 / 5) * v0 + point0
    point2 = (1 / 20) * acc0 + 2 * point1 - point0
    point4 = point5 - (1 / 5) * v1
    point3 = (1 / 20) * acc1 + 2 * point4 - point5

    return Quintic_Bezier(point0, point1, point2, point3, point4, point5)

def lerp(point0, point1, t):

    return (1 - t) * point0 + t * point1

