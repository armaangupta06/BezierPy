from curve import *
from Pose import *
import time


def main():
    # tw = 10
    p = path_with_points(Point(0, 0), Point(36, 36), initial_heading=0, final_heading=0, tangent_magnitude=1)
    # p = path_with_poses(Pose(0, 0, -90), Pose(0, 36, 90), Pose(72, 36, 90), tangent_magnitude=1)
    t = calculate_trajectory(p, 0, 0, 40, 20, -20, 0, 10, True)
    for p in t:
        print(p.x, p.y, p.velocity)
    graph_path(t)


main()

