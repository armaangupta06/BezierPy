from curve import *
from Pose import *

def main():
    graph_path(path_with_points(Point(0, 0), Point(100, 100), initial_heading=0, final_heading=0, tangent_magnitude=2,v= 100, a=10))

main()