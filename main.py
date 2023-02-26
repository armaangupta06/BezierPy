from curve import *
from Pose import *
import time


def main():
    path = path_with_points( Point(0, 0), Point(36, 36), initial_heading=0, final_heading=0,  tangent_magnitude=1.5)
    steps = calculate_trajectory(path, 50, 2, 25)

    graph_path(steps)
main()