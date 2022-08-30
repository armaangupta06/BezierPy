import numpy as np
from Pose import *
import matplotlib.pyplot as plt
from Quintic_bezier import Quintic_bezier

def graph_path(data):
    """Graphs points of path using matplotlib"""

    fig, axis = plt.subplots(2)

    for P in data[0]:
        axis[0].scatter(P.x, P.y, color="black")

    axis[0].set_xlim([-0.25, 4])
    axis[0].set_ylim([0, 4])
    axis[0].set_title("Points")

    for C in range(len(data[1])):
        # print(C, data[1][C])
        axis[1].scatter(C, data[1][C], color="black")

    axis[1].set_xlim([0, 200])
    axis[1].set_ylim([-10, 8])
    axis[1].set_title("Curvature")
    plt.show()


def path_with_points(*points, initial_heading, final_heading=None, tangent_magnitude = 1/2):
    """Spline interpolation between waypoints using an arbitrary number of waypoints and initial/final heading."""

    # List which holds discrete points from the quintic beziér curves
    path = []
    curvature = []

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
            path.append(p)
            curvature.append(w)


    return path, curvature


def path_with_poses(*poses, tangent_magnitude = 1/2):
    """Spline interpolation between waypoints using an arbitrary number of poses."""

    # List which holds discrete points from quintic beziér curves
    path = []
    curvature = []

    # Iterates through waypoints and creates path
    for curr in range(len(poses) - 1):
        curve = calc_bezier_curve_with_poses(curr, poses, tangent_magnitude=tangent_magnitude)
        for t in np.arange(0, 1, 0.01):
            p = curve.get_point(t)
            w = curve.calc_curvature(t)
            path.append(p)
            curvature.append(w)

    return path, curvature


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
    return Quintic_bezier(point0, point1, point2, point3, point4, point5)


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

    return Quintic_bezier(point0, point1, point2, point3, point4, point5)

def lerp(point0, point1, t):

    return (1 - t) * point0 + t * point1

