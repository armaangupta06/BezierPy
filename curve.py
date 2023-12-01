import numpy as np
import matplotlib.pyplot as plt
from Quintic_Bezier import Quintic_Bezier
from Path_Point import *

def generate_points(path):
    """Generates points from list of Quintic Beziér objects."""
    points = []
    for curve in path:
        for t in np.arange(0, 1.01, 0.01):
            # Calculate first derivative vector at point
            dydx = curve.calc_first_derivative(t)

            #Calculate direction of vector.
            theta = math.atan2(dydx.x, dydx.y) * (180 / math.pi)

            velocity = magnitude(curve.calc_first_derivative(t))
            points.append(Path_Point(curve.get_point(t), curve.calc_curvature(t), velocity = velocity, theta = theta))

    return points
def calculate_trajectory(path, max_v, max_w, max_a, max_j=0, trap=True):
    steps = []
    arcLength = 0
    s = 0.01
    last_velocity = 0
    last_curvature = 0
    last_w = 0

    # Calculate cumulative arclength
    for curve in path:
        arcLength += curve.calc_arc_length()

    for curve in path:
        dt = 0.01
        t = 0

        while t < 1.0 and s < arcLength:

            ang_acc = 100
            # Calculates current curvature, velocity, and point with last value of t.
            curvature = curve.calc_curvature(t)

            max_reachable_velocity = (max_v * max_w)/(math.fabs(curvature) * max_v + max_w)
            if trap:
                velocity = min(trapezoidal_motion_profile(s, arcLength, max_v, max_a), max_reachable_velocity)
            else:
                velocity = calculate_s_curve(0, arcLength, 0, 0, max_v, max_a, max_j, s)[0]


            if last_velocity == 0:
                acceleration = velocity/dt
            else:
                acceleration = (velocity ** 2 - last_velocity ** 2)/(2 * last_velocity * dt)

            print((ang_acc*dt)/curvature + last_velocity)

            pose = curve.get_point(t)
            w = curvature * velocity


            # Increase arc length by integrating velocity at timestep.
            deltaS = velocity * dt
            s += deltaS

            # Increase t by dividing change in distance in arc length by magnitude of velocity vector.
            magnitude_velocity = magnitude(curve.calc_first_derivative(t))

            delta_t = deltaS/magnitude_velocity
            t += delta_t

            last_velocity = velocity
            last_curvature = curvature
            last_w = w

            # Append PathPoint to path
            steps.append(Path_Point(pose, curvature = curvature, velocity=velocity));

    return steps

def graph_path(data):
    """Graphs points of path using matplotlib"""

    fig, axis = plt.subplots(3)

    #Plot points in data.
    axis[0].plot([point.x for point in data], [point.y for point in data])
    axis[0].set_title("Points")

    #Plot velocity/curvature in data.
    axis[1].plot([t for t in range(len(data))], [data[t].velocity for t in range(len(data))])
    axis[2].plot([t for t in range(len(data))], [data[t].curvature for t in range(len(data))])

    # for t in range(len(data)):
    #     print(data[t].curvature)
    #Set titles of subplots.
    axis[1].set_title("Velocity")
    axis[2].set_title("Angular Velocity")
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
        # print(curve.calc_first_derivative(0))
        # print(curve.calc_first_derivative(1))
        # print(curve.calc_second_derivative(0))
        # print(curve.calc_second_derivative(1))
        path.append(curve)

    return path

def path_with_poses(*poses, v = 100, a = 10, tangent_magnitude = 1/2):
    """Spline interpolation between waypoints using an arbitrary number of poses."""

    # List which holds discrete points from quintic beziér curves
    path = []

    # Iterates through waypoints and creates path
    for curr in range(len(poses) - 1):
        curve = calc_bezier_curve_with_poses(curr, poses, tangent_magnitude=tangent_magnitude)
        path.append(curve)

    return path

def trapezoidal_motion_profile(distance, totalDist, maxVelocity, maxAcceleration):

    # Initializes variables 'plateauDist' and 'distToAccel'
    plateauDist = totalDist - (maxVelocity * maxVelocity) / maxAcceleration
    distToAccel = (totalDist - plateauDist) / 2

    # Case where distance is too short for robot to reach 'maxVelocity'
    if totalDist <= 2 * distToAccel:
        plateauDist = 0
        distToAccel = totalDist / 2

        # recalculates 'maxVelocity' to appropriate value
        maxVelocity = math.sqrt(2 * maxAcceleration * distToAccel)

    if distance < distToAccel:

        velocity = math.sqrt(2 * maxAcceleration * distance)
    elif distance < (plateauDist + distToAccel):
        velocity = maxVelocity
    else:
        velocity = math.sqrt(maxVelocity * maxVelocity - 2 * maxAcceleration * (distance - (distToAccel + plateauDist)))

    return velocity


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
            v1 = Point(math.sin(final_heading), math.cos(final_heading)) * magnitudev1

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

    # print(acc0, acc1)
    # print(v0, v1)
    point0 = path[curr]
    point5 = path[curr + 1]
    point1 = (1 / 5) * v0 + point0
    point2 = (1 / 20) * acc0 + 2 * point1 - point0
    point4 = point5 - (1 / 5) * v1
    point3 = (1 / 20) * acc1 + 2 * point4 - point5

    return Quintic_Bezier(point0, point1, point2, point3, point4, point5)

def solve(a, b, c, d):
    if (a == 0 and b == 0):  # Case for handling Liner Equation
        return [(-d * 1.0) / c]  # Returning linear root as numpy array.

    elif (a == 0):  # Case for handling Quadratic Equations

        D = c * c - 4.0 * b * d  # Helper Temporary Variable
        if D >= 0:
            D = math.sqrt(D)
            x1 = (-c + D) / (2.0 * b)
            x2 = (-c - D) / (2.0 * b)
        else:
            D = math.sqrt(-D)
            x1 = (-c + D * 1j) / (2.0 * b)
            x2 = (-c - D * 1j) / (2.0 * b)

        return [x1, x2] # Returning Quadratic Roots as numpy array.

    f = findF(a, b, c)  # Helper Temporary Variable
    g = findG(a, b, c, d)  # Helper Temporary Variable
    h = findH(g, f)  # Helper Temporary Variable

    if f == 0 and g == 0 and h == 0:  # All 3 Roots are Real and Equal
        if (d / a) >= 0:
            x = (d / (1.0 * a)) ** (1 / 3.0) * -1
        else:
            x = (-d / (1.0 * a)) ** (1 / 3.0)
        return [x, x, x]  # Returning Equal Roots as numpy array.

    elif h <= 0:  # All 3 roots are Real

        i = math.sqrt(((g ** 2.0) / 4.0) - h)  # Helper Temporary Variable
        j = i ** (1 / 3.0)  # Helper Temporary Variable
        k = math.acos(-(g / (2 * i)))  # Helper Temporary Variable
        L = j * -1  # Helper Temporary Variable
        M = math.cos(k / 3.0)  # Helper Temporary Variable
        N = math.sqrt(3) * math.sin(k / 3.0)  # Helper Temporary Variable
        P = (b / (3.0 * a)) * -1  # Helper Temporary Variable

        x1 = 2 * j * math.cos(k / 3.0) - (b / (3.0 * a))
        x2 = L * (M + N) + P
        x3 = L * (M - N) + P

        return [x1, x2, x3]  # Returning Real Roots as numpy array.

    elif h > 0:  # One Real Root and two Complex Roots
        R = -(g / 2.0) + math.sqrt(h)  # Helper Temporary Variable
        if R >= 0:
            S = R ** (1 / 3.0)  # Helper Temporary Variable
        else:
            S = (-R) ** (1 / 3.0) * -1  # Helper Temporary Variable
        T = -(g / 2.0) - math.sqrt(h)
        if T >= 0:
            U = (T ** (1 / 3.0))  # Helper Temporary Variable
        else:
            U = ((-T) ** (1 / 3.0)) * -1  # Helper Temporary Variable

        x1 = (S + U) - (b / (3.0 * a))


        return [x1]  # Returning One Real Root and two Complex Roots as numpy array.


# Helper function to return float value of f.
def findF(a, b, c):
    return ((3.0 * c / a) - ((b ** 2.0) / (a ** 2.0))) / 3.0


# Helper function to return float value of g.
def findG(a, b, c, d):
    return (((2.0 * (b ** 3.0)) / (a ** 3.0)) - ((9.0 * b * c) / (a ** 2.0)) + (27.0 * d / a)) / 27.0


# Helper function to return float value of h.
def findH(g, f):
    return ((g ** 2.0) / 4.0 + (f ** 3.0) / 27.0)

def findV(s0, s1, v0, a0, j):
    vals = solve((1/6) * j, (1/2) * a0, v0, s0-s1)

    if len(vals) > 1:
        for t in vals:
            v = v0 + a0 * t + (1 / 2) * j * (t ** 2)
            if v > 0:

                return [v, a0 + j*t]
    t = vals[0]

    v = v0 + a0 * t + (1/2) * j * (t ** 2)

    return [v, a0 + j* t]

def calculate_s_curve(q0, q1, v0, v1, v_max, a_max, j_max, d):
    dv = abs(v1 - v0)
    dq = abs(q1 - q0)

    time_to_reach_max_a = a_max / j_max
    time_to_set_set_speeds = math.sqrt(dv / j_max)
    Tj = min(time_to_reach_max_a, time_to_set_set_speeds)

    if Tj == time_to_reach_max_a:
        if (dq > 0.5*(v0 + v1)*(Tj+dv/a_max)) == False:
            raise Exception("Something went wrong")

    elif Tj < time_to_reach_max_a:
        # print(dq, Tj*(v0+v1))
        # print("hi")
        if (dq > Tj*(v0+v1)) == False:
            raise Exception("Something went wrong")


    else:
        raise Exception("Something went wrong")

    # Acceleration period
    if (v_max - v0) * j_max < a_max ** 2:
        # a_max is not reached
        Tj1 = math.sqrt((v_max - v0) / j_max)
        Ta = 2 * Tj1
    else:
        # a_max is reached
        Tj1 = a_max / j_max
        Ta = Tj1 + (v_max - v0) / a_max

    # Deceleration period
    if (v_max - v1) * j_max < a_max ** 2:
        # a_min is not reached
        Tj2 = math.sqrt((v_max - v1) / j_max)
        Td = 2 * Tj2
    else:
        # a_min is reached
        Tj2 = a_max / j_max
        Td = Tj2 + (v_max - v1) / a_max

    Tv = (q1 - q0) / v_max - (Ta / 2) * (1 + v0 / v_max) - (Td / 2) * (1 + v1 / v_max)
    if Tv < 0:
        Tj1 = Tj2 = Tj = a_max/j_max
        Tv = 0

        v = (a_max**2)/j_max
        delta = ((a_max**4)/(j_max**2)) + 2*((v0**2)+(v1**2)) +\
            a_max*(4*(q1-q0)-2*(a_max/j_max)*(v0+v1))

        Ta = (v - 2*v0 + math.sqrt(delta))/(2*a_max)
        Td = (v - 2*v1 + math.sqrt(delta))/(2*a_max)

    T = Ta + Td + Tv
    a_lim_a = j_max*Tj1
    a_lim_d = -j_max*Tj2
    v_lim = v0 + (Ta-Tj1)*a_lim_a
    EPSILON = 0.00001
    if(Ta - 2 * Tj < EPSILON) or (Td - 2 * Tj < EPSILON):
        raise Exception("Something went wrong")
    x1 = q0 + v0*Tj1 + a_lim_a*(Tj1**2)/6
    x2 = q0 + (v_lim+v0)*Ta/2 - v_lim*Tj1 + j_max*(Tj1**3)/6
    x3 = q0 + (v_lim+v0)*Ta/2
    x4 = q1 - (v_lim + v1) * Td / 2
    x5 = q1 - (v_lim+v1)*Td/2 + v_lim*Tj2 +a_lim_d*( Tj2**2)/6
    x6 = q1 - v1*Tj2 - j_max*(Tj2**3)/6
    x7 = q1

    # print(x1, x2, x3, x4, x5, x6, x7)
    # print("hi")

    if 0 <= d < x1:
        v, a = findV(0, d, v0, 0, j_max)
        # print(1)
    elif x1 <= d < x2:
        v0 = v0 + a_lim_a * (Tj1/2)
        v, a = findV(x1, d, v0, a_lim_a, 0)
        # print(2)
    elif x2 <= d < x3:
        v0 = v_lim - j_max * (Tj1 ** 2) / 2
        v, a = findV(x2, d, v0, a_lim_a, -j_max)
        # print(3)

        # Constant velocity phase
    elif x3 <= d < x4:
        v, a = findV(x3, d, v_lim, 0, 0)
        # print(4)

        # Deceleration phase
    elif x4 <= d < x5:
        v, a = findV(x4, d, v_lim, 0, -j_max)
        # print(5)

    elif x5 <= d < x6:
        v0 = v_lim + a_lim_d * (Tj2 / 2)

        v, a = findV(x5, d, v0, a_lim_d, 0)
        # print(v)
        # print(6)

    elif x6 <= d < x7:
        # print("hi")
        v0 = v1 + j_max * (Tj2 ** 2) / 2

        v, a = findV(x6, d, v0, a_lim_d, j_max)
        # print(x6, d, v0, a_lim_d, -j_max)
        # print(v, 7)

    else:
        a = 0
        v = v1
        q = q1
        # print(8)

    # print(v, a)

    return v, a

def lerp(point0, point1, t):

    return (1 - t) * point0 + t * point1

