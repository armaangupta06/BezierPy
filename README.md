
# BezierPy
Generates a Quintic Beziér Spline from an arbitrary number of waypoints (user has the option to manipulate heading at each waypoint).

This project is purposed for path generation for Vex Robotics.

Quintic Bezíer splines were chosen because of their C1 continuity (continuous heading changes) and their C2 continuity (continuity in curvature; without this, the 
robot would be subject to infinitely high acceleraetion).

The first derivative vector at each waypoint
if found by using the robot's heading (if supplied) or by its neighboring points. Specifically,
the direction is perpendicular to the angle bisector of the angle inscribed 
with its neighbor points. Furthermore, the magnitude is determined by the distance to its closest point.

The second derivative vector at each waypoint is computed by a weighted average of the second derivatives
that two Cubic Beziér Splines would have at the point. This results in a smooth transition between each segment of the spline.





## Features

- Spline interpolation between an arbitrary number of poses (point + heading).
- Spline interpolation between an abitrary number of points (with initial and final heading).
- Linear interpolation


## Usage/Examples

```python
graph_path(path_with_points(Point(0, 0), Point(1, 1), Point(0, 1), initial_heading=0, final_heading=-90))
```
## Acknowledgements

 - [Planning Motion Trajectories for Mobile Robots Using Splines](https://awesomeopensource.com/project/elangosundar/awesome-README-templates)
