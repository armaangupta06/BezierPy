from curve import *
from Pose import *
import time


def main():
    start = time.time()
    path = path_with_points( Point(0, 0), Point(36, 36), Point(0, 36), Point(-100, 100), initial_heading=0, final_heading=0,  tangent_magnitude=1)
    steps = calculate_trajectory(path, 50, 2, 25, max_j=20, trap=False)

    end = time.time()
    print(end-start)

    print(len(steps))
    graph_path(steps)
main()
