from curve import *
from Pose import *

def main():
    graph_path(path_with_poses(Pose(0, 0, 0), Pose(0.5, 2, 90), Pose(4, 2, 90),  tangent_magnitude=1))

main()