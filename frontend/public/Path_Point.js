class Path_Point {
    constructor(point, curvature, theta = 0, velocity = 0) {
      this.x = point.x;
      this.y = point.y;
      this.curvature = curvature;
      this.velocity = velocity;
      this.theta = theta;
    }
  }