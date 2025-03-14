// -----------------------------
// Helper Math and Geometry Functions
// -----------------------------

export class Point {
    constructor(x, y) {
      this.x = x;
      this.y = y;
    }
  
    // Addition
    add(other) {
      return new Point(this.x + other.x, this.y + other.y);
    }
  
    // Subtraction
    sub(other) {
      return new Point(this.x - other.x, this.y - other.y);
    }
  
    // Multiplication by a scalar
    mul(scalar) {
      return new Point(this.x * scalar, this.y * scalar);
    }
  
    // Division by a scalar
    div(scalar) {
      return new Point(this.x / scalar, this.y / scalar);
    }
  
    // Rounds both coordinates to given number of decimals
    round(decimals) {
      const factor = Math.pow(10, decimals);
      return new Point(
        Math.round(this.x * factor) / factor,
        Math.round(this.y * factor) / factor
      );
    }
  
    toString() {
      return `(${this.x}, ${this.y})`;
    }
  }
  
  function distanceFormula(p1, p2) {
    return Math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2);
  }
  
  function magnitude(p) {
    return Math.sqrt(p.x ** 2 + p.y ** 2);
  }
  
  function dotProduct(v1, v2) {
    return v1.x * v2.x + v1.y * v2.y;
  }
  
  function determinant(v1, v2) {
    return v1.x * v2.y - v2.x * v1.y;
  }
  
  function slope(p1, p2) {
    return (p2.y - p1.y) / (p2.x - p1.x);
  }
  
  // Returns a unit vector perpendicular to the angle bisector
  // defined by points A, B, and C.
  function getPerpendicularVector(A, B, C) {
    const AB = B.sub(A);
    const BC = C.sub(B);
    const v1 = AB.div(magnitude(AB));
    const v2 = BC.div(magnitude(BC));
    const sum = v1.add(v2);
    const magSum = magnitude(sum);
    if (magSum === 0) return new Point(1, 0);
    return sum.div(magSum);
  }
  
  function lerp(point0, point1, t) {
    return point0.mul(1 - t).add(point1.mul(t));
  }
  
  // -----------------------------
  // Data Classes
  // -----------------------------
  
  // Represents a path point with curvature, velocity, and (optionally) a heading.
  export class Path_Point {
    constructor(point, curvature, theta = 0, velocity = 0) {
      this.x = point.x;
      this.y = point.y;
      this.curvature = curvature;
      this.velocity = velocity;
      this.theta = theta;
    }
  }
  
  // Pose extends Point by adding a heading (in radians)
  export class Pose extends Point {
    constructor(x, y, headingDegrees) {
      super(x, y);
      this.heading = headingDegrees * Math.PI / 180;
    }
  }
  
  // Quintic_Bezier represents a quintic Bézier curve defined by six control points.
  export class Quintic_Bezier {
    constructor(point0, point1, point2, point3, point4, point5) {
      this.point0 = point0;
      this.point1 = point1;
      this.point2 = point2;
      this.point3 = point3;
      this.point4 = point4;
      this.point5 = point5;
    }
  
    getControlPoints() {
      return [this.point0, this.point1, this.point2, this.point3, this.point4, this.point5];
    }
  
    calcFirstDerivative(t) {
      // p = 5*(1-t)^4*(P1-P0) + 20*t*(1-t)^3*(P2-P1) +
      //     30*t^2*(1-t)^2*(P3-P2) + 20*t^3*(1-t)*(P4-P3) +
      //     5*t^4*(P5-P4)
      const term1 = this.point1.sub(this.point0).mul(5 * Math.pow(1 - t, 4));
      const term2 = this.point2.sub(this.point1).mul(20 * t * Math.pow(1 - t, 3));
      const term3 = this.point3.sub(this.point2).mul(30 * Math.pow(t, 2) * Math.pow(1 - t, 2));
      const term4 = this.point4.sub(this.point3).mul(20 * Math.pow(t, 3) * (1 - t));
      const term5 = this.point5.sub(this.point4).mul(5 * Math.pow(t, 4));
      return term1.add(term2).add(term3).add(term4).add(term5);
    }
  
    calcSecondDerivative(t) {
      // The second derivative is computed as a sum of four terms.
      const term1 = this.point2.sub(this.point1.mul(2)).add(this.point0).mul(20 * Math.pow(1 - t, 3));
      const term2 = this.point3.sub(this.point2.mul(2)).add(this.point1).mul(60 * t * Math.pow(1 - t, 2));
      const term3 = this.point4.sub(this.point3.mul(2)).add(this.point2).mul(60 * Math.pow(t, 2) * (1 - t));
      const term4 = this.point5.sub(this.point4.mul(2)).add(this.point3).mul(20 * Math.pow(t, 3));
      return term1.add(term2).add(term3).add(term4);
    }
  
    calcCurvature(t, firstDeriv = null) {
      if (firstDeriv === null) {
        firstDeriv = this.calcFirstDerivative(t);
      }
      const m = magnitude(firstDeriv);
      if (m === 0) return 0;
      const numer = determinant(firstDeriv, this.calcSecondDerivative(t));
      return numer / Math.pow(m, 3);
    }
  
    getPoint(t) {
      // p = (1-t)^5*P0 + 5*(1-t)^4*t*P1 + 10*(1-t)^3*t^2*P2 +
      //     10*(1-t)^2*t^3*P3 + 5*(1-t)*t^4*P4 + t^5*P5
      const p0 = this.point0.mul(Math.pow(1 - t, 5));
      const p1 = this.point1.mul(5 * Math.pow(1 - t, 4) * t);
      const p2 = this.point2.mul(10 * Math.pow(1 - t, 3) * Math.pow(t, 2));
      const p3 = this.point3.mul(10 * Math.pow(1 - t, 2) * Math.pow(t, 3));
      const p4 = this.point4.mul(5 * (1 - t) * Math.pow(t, 4));
      const p5 = this.point5.mul(Math.pow(t, 5));
      return p0.add(p1).add(p2).add(p3).add(p4).add(p5);
    }
  
    calcArcLength() {
      let distance = 0;
      let prevPoint = this.getPoint(0);
      for (let t = 0; t <= 1.0001; t += 0.01) {
        const currPoint = this.getPoint(t);
        distance += distanceFormula(currPoint, prevPoint);
        prevPoint = currPoint;
      }
      return distance;
    }
  }
  
  // -----------------------------
  // Main Functions (Trajectory, Spline, etc.)
  // -----------------------------
  
  // Generates discrete Path_Point objects from an array of Quintic_Bezier curves.
  export function generatePoints(path) {
    const points = [];
    for (const curve of path) {
      for (let t = 0; t <= 1.0001; t += 0.01) {
        const deriv = curve.calcFirstDerivative(t);
        // Theta: note the order of arguments in atan2 (here using x then y as in the Python code)
        const theta = Math.atan2(deriv.x, deriv.y) * (180 / Math.PI);
        const velocity = magnitude(curve.calcFirstDerivative(t));
        points.push(new Path_Point(curve.getPoint(t), curve.calcCurvature(t), theta, velocity));
      }
    }
    return points;
  }
  
  // Calculates the trajectory along the path (returns an array of Path_Point objects).
  export function calculateTrajectory(path, v0, v1, max_v, a_accel, a_decel, max_j, max_w, trap = true) {
    const steps = [];
    let arcLength = 0;
    let s = 0.01;
    let last_velocity = 0;
    let last_curvature = 0;
    let last_w = 0;
  
    // Sum cumulative arc length over all curves.
    for (const curve of path) {
      arcLength += curve.calcArcLength();
    }
  
    for (const curve of path) {
      const dt = 0.01;
      let t = 0;
      while (t < 1.0 && s < arcLength) {
        const deriv = curve.calcFirstDerivative(t);
        const curvature = curve.calcCurvature(t, deriv);
        const max_reachable_velocity = (max_v * max_w) / (Math.abs(curvature) * max_v + max_w);
  
        let velocity;
        if (trap) {
          velocity = Math.min(
            trapezoidalMotionProfile(s, arcLength, v0, v1, max_v, a_accel, a_decel),
            max_reachable_velocity
          );
        } else {
          velocity = calculateSCurve(0, arcLength, v0, v1, max_v, a_accel, max_j, s)[0];
        }
  
        const pose = curve.getPoint(t);
        const w = curvature * velocity;
        const deltaS = velocity * dt;
        s += deltaS;
        const magnitude_velocity = magnitude(deriv);
        const delta_t = deltaS / magnitude_velocity;
        t += delta_t;
        last_velocity = velocity;
        last_curvature = curvature;
        last_w = w;
        steps.push(new Path_Point(pose, curvature, 0, velocity));
      }
    }
    return steps;
  }
  
  // Graphs the path (here we simply log the data; you may integrate with a plotting library)
  export function graphPath(data) {
    console.log("Graphing path points:");
    console.log("Coordinates:");
    data.forEach((pt, i) => {
      console.log(`t=${i}: (${pt.x.toFixed(2)}, ${pt.y.toFixed(2)})`);
    });
    console.log("Velocity profile:");
    data.forEach((pt, i) => {
      console.log(`t=${i}: velocity=${pt.velocity}`);
    });
    console.log("Curvature profile:");
    data.forEach((pt, i) => {
      console.log(`t=${i}: curvature=${pt.curvature}`);
    });
    // For real plotting, integrate with a library like Chart.js, Plotly, or D3.
  }
  
  // Spline interpolation between waypoints given points and initial/final headings (headings in degrees).
  export function pathWithPoints(points, { initial_heading, final_heading = null, v = 100, a = 10, tangent_magnitude = 0.5 } = {}) {
    const path = [];
    initial_heading = initial_heading * Math.PI / 180;
    if (final_heading !== null) {
      final_heading = final_heading * Math.PI / 180;
    }
    for (let curr = 0; curr < points.length - 1; curr++) {
      const curve = calcBezierCurveWithPoints(curr, points, initial_heading, final_heading, tangent_magnitude);
      path.push(curve);
    }
    return path;
  }
  
  // Spline interpolation between poses (an array of Pose objects).
  export function pathWithPoses(poses, { v = 100, a = 10, tangent_magnitude = 0.5 } = {}) {
    const path = [];
    for (let curr = 0; curr < poses.length - 1; curr++) {
      const curve = calcBezierCurveWithPoses(curr, poses, tangent_magnitude);
      path.push(curve);
    }
    return path;
  }
  
  // -----------------------------
  // Bézier Curve Calculation Helpers
  // -----------------------------
  
  // Calculates control points for a quintic Bézier curve given an arbitrary number of poses.
  function calcBezierCurveWithPoses(curr, path, tangent_magnitude) {
    // Unit vectors from the heading of the poses.
    let v0 = new Point(Math.sin(path[curr].heading), Math.cos(path[curr].heading));
    let v1 = new Point(Math.sin(path[curr + 1].heading), Math.cos(path[curr + 1].heading));
    let acc0, acc1;
  
    if (curr === 0) {
      if (path.length === 2) {
        const magnitudev0 = tangent_magnitude * magnitude(path[curr + 1].sub(path[curr]));
        v0 = v0.mul(magnitudev0);
        v1 = v1.mul(magnitudev0);
        acc0 = path[curr].mul(-6).add(v0.mul(-4)).add(v1.mul(-2)).add(path[curr + 1].mul(6));
        acc1 = path[curr].mul(6).add(v0.mul(2)).add(v1.mul(4)).sub(path[curr + 1].mul(6));
      } else {
        let magnitudevD, vD;
        if (curr + 3 === path.length) {
          magnitudevD = tangent_magnitude * magnitude(path[curr + 2].sub(path[curr + 1]));
          vD = new Point(Math.sin(path[curr + 2].heading), Math.cos(path[curr + 2].heading)).mul(magnitudevD);
        } else {
          magnitudevD = tangent_magnitude * Math.min(
            magnitude(path[curr + 2].sub(path[curr + 1])),
            magnitude(path[curr + 3].sub(path[curr + 2]))
          );
          vD = new Point(Math.sin(path[curr + 2].heading), Math.cos(path[curr + 2].heading)).mul(magnitudevD);
        }
        const magnitudev0 = tangent_magnitude * magnitude(path[curr + 1].sub(path[curr]));
        const magnitudev1 = tangent_magnitude * Math.min(
          magnitude(path[curr + 1].sub(path[curr])),
          magnitude(path[curr + 2].sub(path[curr + 1]))
        );
        v0 = v0.mul(magnitudev0);
        v1 = v1.mul(magnitudev1);
        const denom = magnitude(path[curr + 1].sub(path[curr])) + magnitude(path[curr + 2].sub(path[curr + 1]));
        const alpha = magnitude(path[curr + 2].sub(path[curr + 1])) / denom;
        const beta = magnitude(path[curr + 1].sub(path[curr])) / denom;
        acc0 = path[curr].mul(-6).add(v0.mul(-4)).add(v1.mul(-2)).add(path[curr + 1].mul(6));
        acc1 = (path[curr].mul(6).add(v0.mul(2)).add(v1.mul(4)).sub(path[curr + 1].mul(6))).mul(alpha)
             .add((path[curr + 1].mul(-6).add(v1.mul(-4)).sub(vD.mul(2)).add(path[curr + 2].mul(6))).mul(beta));
      }
    } else if (curr === path.length - 2) {
      let magnitudevA, vA;
      if (curr - 1 === 0) {
        magnitudevA = tangent_magnitude * magnitude(path[curr].sub(path[curr - 1]));
        vA = new Point(Math.sin(path[curr - 1].heading), Math.cos(path[curr - 1].heading)).mul(magnitudevA);
      } else {
        magnitudevA = tangent_magnitude * Math.min(
          magnitude(path[curr - 2].sub(path[curr - 1])),
          magnitude(path[curr].sub(path[curr - 1]))
        );
        vA = new Point(Math.sin(path[curr - 1].heading), Math.cos(path[curr - 1].heading)).mul(magnitudevA);
      }
      const magnitudev0 = tangent_magnitude * Math.min(
        magnitude(path[curr].sub(path[curr - 1])),
        magnitude(path[curr + 1].sub(path[curr]))
      );
      const magnitudev1 = tangent_magnitude * magnitude(path[curr + 1].sub(path[curr]));
      v0 = v0.mul(magnitudev0);
      v1 = v1.mul(magnitudev1);
      const denom = magnitude(path[curr].sub(path[curr - 1])) + magnitude(path[curr + 1].sub(path[curr]));
      const alpha = magnitude(path[curr + 1].sub(path[curr])) / denom;
      const beta = magnitude(path[curr].sub(path[curr - 1])) / denom;
      acc0 = (path[curr - 1].mul(6).add(vA.mul(2)).add(v0.mul(4)).sub(path[curr].mul(6))).mul(alpha)
           .add((path[curr].mul(-6).sub(v0.mul(4)).sub(v1.mul(2)).add(path[curr + 1].mul(6))).mul(beta));
      acc1 = path[curr].mul(6).add(v0.mul(2)).add(v1.mul(4)).sub(path[curr + 1].mul(6));
    } else {
      let magnitudevA, vA;
      if (curr - 1 === 0) {
        magnitudevA = tangent_magnitude * magnitude(path[curr].sub(path[curr - 1]));
        vA = new Point(Math.sin(path[curr - 1].heading), Math.cos(path[curr - 1].heading)).mul(magnitudevA);
      } else {
        magnitudevA = tangent_magnitude * Math.min(
          magnitude(path[curr - 2].sub(path[curr - 1])),
          magnitude(path[curr].sub(path[curr - 1]))
        );
        vA = new Point(Math.sin(path[curr - 1].heading), Math.cos(path[curr - 1].heading)).mul(magnitudevA);
      }
      let magnitudevD, vD;
      if (curr + 2 === path.length - 1) {
        magnitudevD = tangent_magnitude * magnitude(path[curr + 2].sub(path[curr + 1]));
        vD = new Point(Math.sin(path[curr + 2].heading), Math.cos(path[curr + 2].heading)).mul(magnitudevD);
      } else {
        magnitudevD = tangent_magnitude * Math.min(
          magnitude(path[curr + 2].sub(path[curr + 1])),
          magnitude(path[curr + 3].sub(path[curr + 2]))
        );
        vD = new Point(Math.sin(path[curr + 2].heading), Math.cos(path[curr + 2].heading)).mul(magnitudevD);
      }
      const magnitudev0 = tangent_magnitude * Math.min(
        magnitude(path[curr].sub(path[curr - 1])),
        magnitude(path[curr + 1].sub(path[curr]))
      );
      const magnitudev1 = tangent_magnitude * Math.min(
        magnitude(path[curr + 1].sub(path[curr])),
        magnitude(path[curr + 2].sub(path[curr + 1]))
      );
      v0 = v0.mul(magnitudev0);
      v1 = v1.mul(magnitudev1);
      const denom0 = magnitude(path[curr].sub(path[curr - 1])) + magnitude(path[curr + 1].sub(path[curr]));
      const alpha0 = magnitude(path[curr + 1].sub(path[curr])) / denom0;
      const beta0 = magnitude(path[curr].sub(path[curr - 1])) / denom0;
      const denom1 = magnitude(path[curr + 1].sub(path[curr])) + magnitude(path[curr + 2].sub(path[curr + 1]));
      const alpha1 = magnitude(path[curr + 2].sub(path[curr + 1])) / denom1;
      const beta1 = magnitude(path[curr + 1].sub(path[curr])) / denom1;
      acc0 = (path[curr - 1].mul(6).add(vA.mul(2)).add(v0.mul(4)).sub(path[curr].mul(6))).mul(alpha0)
           .add((path[curr].mul(-6).sub(v0.mul(4)).sub(v1.mul(2)).add(path[curr + 1].mul(6))).mul(beta0));
      acc1 = (path[curr].mul(6).add(v0.mul(2)).add(v1.mul(4)).sub(path[curr + 1].mul(6))).mul(alpha1)
           .add((path[curr + 1].mul(-6).sub(v1.mul(4)).sub(vD.mul(2)).add(path[curr + 2].mul(6))).mul(beta1));
    }
  
    // Round the computed vectors and accelerations.
    v0 = v0.round(10);
    v1 = v1.round(10);
    acc0 = acc0.round(10);
    acc1 = acc1.round(10);
  
    const point0 = path[curr];
    const point5 = path[curr + 1];
    const point1 = point0.add(v0.mul(1 / 5));
    const point2 = point1.mul(2).sub(point0).add(acc0.mul(1 / 20));
    const point4 = point5.sub(v1.mul(1 / 5));
    const point3 = point4.mul(2).sub(point5).add(acc1.mul(1 / 20));
    return new Quintic_Bezier(point0, point1, point2, point3, point4, point5);
  }
  
  // Calculates control points for a quintic Bézier curve given waypoints and specified initial/final headings.
  function calcBezierCurveWithPoints(curr, path, initial_heading, final_heading, tangent_magnitude) {
    // initial_heading and final_heading are in radians.
    let v0, v1, acc0, acc1;
    if (curr === 0) {
      if (path.length === 2) {
        const magnitudev0 = tangent_magnitude * magnitude(path[curr + 1].sub(path[curr]));
        v0 = new Point(Math.sin(initial_heading), Math.cos(initial_heading)).mul(magnitudev0);
        if (final_heading === null) {
          const diff = path[curr + 1].sub(path[curr]);
          v1 = diff.div(magnitude(diff)).mul(magnitudev0);
        } else {
          v1 = new Point(Math.sin(final_heading), Math.cos(final_heading)).mul(magnitudev0);
        }
        acc0 = path[curr].mul(-6).add(v0.mul(-4)).add(v1.mul(-2)).add(path[curr + 1].mul(6));
        acc1 = path[curr].mul(6).add(v0.mul(2)).add(v1.mul(4)).sub(path[curr + 1].mul(6));
      } else {
        let magnitudevD, vD;
        if (curr + 2 === path.length - 1) {
          magnitudevD = tangent_magnitude * magnitude(path[curr + 2].sub(path[curr + 1]));
          if (final_heading === null) {
            const diff = path[curr + 2].sub(path[curr + 1]);
            vD = diff.div(magnitude(diff)).mul(magnitudevD);
          } else {
            vD = new Point(Math.sin(final_heading), Math.cos(final_heading)).mul(magnitudevD);
          }
        } else {
          magnitudevD = tangent_magnitude * Math.min(
            magnitude(path[curr + 2].sub(path[curr + 1])),
            magnitude(path[curr + 3].sub(path[curr + 2]))
          );
          vD = getPerpendicularVector(path[curr + 1], path[curr + 2], path[curr + 3]).mul(magnitudevD);
        }
        const magnitudev0 = tangent_magnitude * magnitude(path[curr + 1].sub(path[curr]));
        const magnitudev1 = tangent_magnitude * Math.min(
          magnitude(path[curr + 1].sub(path[curr])),
          magnitude(path[curr + 2].sub(path[curr + 1]))
        );
        v0 = new Point(Math.sin(initial_heading), Math.cos(initial_heading)).mul(magnitudev0);
        v1 = getPerpendicularVector(path[curr], path[curr + 1], path[curr + 2]).mul(magnitudev1);
        const denom = magnitude(path[curr + 1].sub(path[curr])) + magnitude(path[curr + 2].sub(path[curr + 1]));
        const alpha = magnitude(path[curr + 2].sub(path[curr + 1])) / denom;
        const beta = magnitude(path[curr + 1].sub(path[curr])) / denom;
        acc0 = path[curr].mul(-6).add(v0.mul(-4)).add(v1.mul(-2)).add(path[curr + 1].mul(6));
        acc1 = (path[curr].mul(6).add(v0.mul(2)).add(v1.mul(4)).sub(path[curr + 1].mul(6))).mul(alpha)
             .add((path[curr + 1].mul(-6).add(v1.mul(-4)).sub(vD.mul(2)).add(path[curr + 2].mul(6))).mul(beta));
      }
    } else if (curr === path.length - 2) {
      let magnitudevA, vA;
      if (curr - 1 === 0) {
        magnitudevA = tangent_magnitude * magnitude(path[curr].sub(path[curr - 1]));
        vA = new Point(Math.sin(initial_heading), Math.cos(initial_heading)).mul(magnitudevA);
      } else {
        magnitudevA = tangent_magnitude * Math.min(
          magnitude(path[curr - 2].sub(path[curr - 1])),
          magnitude(path[curr].sub(path[curr - 1]))
        );
        vA = getPerpendicularVector(path[curr - 2], path[curr - 1], path[curr]).mul(magnitudevA);
      }
      const magnitudev0 = tangent_magnitude * Math.min(
        magnitude(path[curr].sub(path[curr - 1])),
        magnitude(path[curr + 1].sub(path[curr]))
      );
      const magnitudev1 = tangent_magnitude * magnitude(path[curr + 1].sub(path[curr]));
      v0 = getPerpendicularVector(path[curr - 1], path[curr], path[curr + 1]).mul(magnitudev0);
      if (final_heading === null) {
        const diff = path[curr + 1].sub(path[curr]);
        v1 = diff.div(magnitude(diff)).mul(magnitudev1);
      } else {
        v1 = new Point(Math.sin(final_heading), Math.cos(final_heading)).mul(magnitudev1);
      }
      const denom = magnitude(path[curr].sub(path[curr - 1])) + magnitude(path[curr + 1].sub(path[curr]));
      const alpha = magnitude(path[curr + 1].sub(path[curr])) / denom;
      const beta = magnitude(path[curr].sub(path[curr - 1])) / denom;
      acc0 = (path[curr - 1].mul(6).add(vA.mul(2)).add(v0.mul(4)).sub(path[curr].mul(6))).mul(alpha)
           .add((path[curr].mul(-6).sub(v0.mul(4)).sub(v1.mul(2)).add(path[curr + 1].mul(6))).mul(beta));
      acc1 = path[curr].mul(6).add(v0.mul(2)).add(v1.mul(4)).sub(path[curr + 1].mul(6));
    } else {
      let magnitudevA, vA;
      if (curr - 1 === 0) {
        magnitudevA = tangent_magnitude * magnitude(path[curr].sub(path[curr - 1]));
        vA = new Point(Math.sin(initial_heading), Math.cos(initial_heading)).mul(magnitudevA);
      } else {
        magnitudevA = tangent_magnitude * Math.min(
          magnitude(path[curr - 2].sub(path[curr - 1])),
          magnitude(path[curr].sub(path[curr - 1]))
        );
        vA = getPerpendicularVector(path[curr - 2], path[curr - 1], path[curr]).mul(magnitudevA);
      }
      let magnitudevD, vD;
      if (curr + 2 === path.length - 1) {
        magnitudevD = tangent_magnitude * magnitude(path[curr + 2].sub(path[curr + 1]));
        if (final_heading === null) {
          const diff = path[curr + 2].sub(path[curr + 1]);
          vD = diff.div(magnitude(diff)).mul(magnitudevD);
        } else {
          vD = new Point(Math.sin(final_heading), Math.cos(final_heading)).mul(magnitudevD);
        }
      } else {
        magnitudevD = tangent_magnitude * Math.min(
          magnitude(path[curr + 2].sub(path[curr + 1])),
          magnitude(path[curr + 3].sub(path[curr + 2]))
        );
        vD = getPerpendicularVector(path[curr + 1], path[curr + 2], path[curr + 3]).mul(magnitudevD);
      }
      const magnitudev0 = tangent_magnitude * Math.min(
        magnitude(path[curr].sub(path[curr - 1])),
        magnitude(path[curr + 1].sub(path[curr]))
      );
      const magnitudev1 = tangent_magnitude * Math.min(
        magnitude(path[curr + 1].sub(path[curr])),
        magnitude(path[curr + 2].sub(path[curr + 1]))
      );
      v0 = getPerpendicularVector(path[curr - 1], path[curr], path[curr + 1]).mul(magnitudev0);
      v1 = getPerpendicularVector(path[curr], path[curr + 1], path[curr + 2]).mul(magnitudev1);
      const denom0 = magnitude(path[curr].sub(path[curr - 1])) + magnitude(path[curr + 1].sub(path[curr]));
      const alpha0 = magnitude(path[curr + 1].sub(path[curr])) / denom0;
      const beta0 = magnitude(path[curr].sub(path[curr - 1])) / denom0;
      const denom1 = magnitude(path[curr + 1].sub(path[curr])) + magnitude(path[curr + 2].sub(path[curr + 1]));
      const alpha1 = magnitude(path[curr + 2].sub(path[curr + 1])) / denom1;
      const beta1 = magnitude(path[curr + 1].sub(path[curr])) / denom1;
      acc0 = (path[curr - 1].mul(6).add(vA.mul(2)).add(v0.mul(4)).sub(path[curr].mul(6))).mul(alpha0)
           .add((path[curr].mul(-6).sub(v0.mul(4)).sub(v1.mul(2)).add(path[curr + 1].mul(6))).mul(beta0));
      acc1 = (path[curr].mul(6).add(v0.mul(2)).add(v1.mul(4)).sub(path[curr + 1].mul(6))).mul(alpha1)
           .add((path[curr + 1].mul(-6).sub(v1.mul(4)).sub(vD.mul(2)).add(path[curr + 2].mul(6))).mul(beta1));
    }
    v0 = v0.round(10);
    v1 = v1.round(10);
    acc0 = acc0.round(10);
    acc1 = acc1.round(10);
    const point0 = path[curr];
    const point5 = path[curr + 1];
    const point1 = point0.add(v0.mul(1 / 5));
    const point2 = point1.mul(2).sub(point0).add(acc0.mul(1 / 20));
    const point4 = point5.sub(v1.mul(1 / 5));
    const point3 = point4.mul(2).sub(point5).add(acc1.mul(1 / 20));
    return new Quintic_Bezier(point0, point1, point2, point3, point4, point5);
  }
  
  // -----------------------------
  // Motion Profile & Equation Solvers
  // -----------------------------
  
  export function trapezoidalMotionProfile(distance, total_dist, v0, v1, v_max, a_accel, a_decel) {
    let cruise_velocity = Math.sqrt((2 * total_dist * a_accel * a_decel + a_decel * v0 * v0 - a_accel * v1 * v1) / (a_decel - a_accel));
    cruise_velocity = Math.min(v_max, cruise_velocity);
    const v_forward = Math.sqrt(v0 * v0 + 2 * a_accel * distance);
    const v_back = Math.sqrt(v1 * v1 - 2 * a_decel * (total_dist - distance));
    return Math.min(cruise_velocity, v_forward, v_back);
  }
  
  export function calcVelocity(path, v, a) {
    if (path.length === 0) return path;
    path[path.length - 1].velocity = 0;
    for (let i = path.length - 2; i >= 0; i--) {
      const curv = (path[i].curvature === 0 ? 0.001 : path[i].curvature);
      const desired_velocity = Math.min(v, 3.0 / Math.abs(curv));
      const dist = distanceFormula(new Point(path[i + 1].x, path[i + 1].y), new Point(path[i].x, path[i].y));
      const limited_velocity = Math.sqrt(Math.pow(path[i + 1].velocity, 2) + 2 * a * dist);
      path[i].velocity = Math.min(desired_velocity, limited_velocity);
    }
    return path;
  }
  
  // Solves a cubic (or lower degree) polynomial equation.
  // Returns an array of roots (real numbers; complex roots are returned as objects with {real, imag}).
  function solve(a, b, c, d) {
    if (a === 0 && b === 0) {
      return [(-d) / c];
    } else if (a === 0) {
      const D = c * c - 4 * b * d;
      if (D >= 0) {
        const sqrtD = Math.sqrt(D);
        const x1 = (-c + sqrtD) / (2 * b);
        const x2 = (-c - sqrtD) / (2 * b);
        return [x1, x2];
      } else {
        const sqrtD = Math.sqrt(-D);
        const x1 = { real: (-c) / (2 * b), imag: sqrtD / (2 * b) };
        const x2 = { real: (-c) / (2 * b), imag: -sqrtD / (2 * b) };
        return [x1, x2];
      }
    }
    const f = findF(a, b, c);
    const g = findG(a, b, c, d);
    const h = findH(g, f);
    if (f === 0 && g === 0 && h === 0) {
      let x;
      if (d / a >= 0) {
        x = -Math.pow(d / a, 1 / 3);
      } else {
        x = Math.pow(-d / a, 1 / 3);
      }
      return [x, x, x];
    } else if (h <= 0) {
      const i = Math.sqrt((g * g) / 4 - h);
      const j = Math.pow(i, 1 / 3);
      const k = Math.acos(-(g / (2 * i)));
      const L = -j;
      const M = Math.cos(k / 3);
      const N = Math.sqrt(3) * Math.sin(k / 3);
      const P = -b / (3 * a);
      const x1 = 2 * j * Math.cos(k / 3) - (b / (3 * a));
      const x2 = L * (M + N) + P;
      const x3 = L * (M - N) + P;
      return [x1, x2, x3];
    } else if (h > 0) {
      const R = -(g / 2) + Math.sqrt(h);
      const S = (R >= 0) ? Math.pow(R, 1 / 3) : -Math.pow(-R, 1 / 3);
      const T = -(g / 2) - Math.sqrt(h);
      const U = (T >= 0) ? Math.pow(T, 1 / 3) : -Math.pow(-T, 1 / 3);
      const x1 = (S + U) - (b / (3 * a));
      return [x1];
    }
  }
  
  function findF(a, b, c) {
    return ((3 * c / a) - ((b ** 2) / (a ** 2))) / 3;
  }
  
  function findG(a, b, c, d) {
    return (((2 * (b ** 3)) / (a ** 3)) - ((9 * b * c) / (a ** 2)) + (27 * d / a)) / 27;
  }
  
  function findH(g, f) {
    return ((g ** 2) / 4 + (f ** 3) / 27);
  }
  
  function findV(s0, s1, v0, a0, j) {
    const vals = solve((1 / 6) * j, (1 / 2) * a0, v0, s0 - s1);
    let t;
    if (vals.length > 1) {
      for (const candidate of vals) {
        t = candidate;
        const v = v0 + a0 * t + 0.5 * j * (t ** 2);
        if (v > 0) {
          return [v, a0 + j * t];
        }
      }
    }
    t = vals[0];
    const v = v0 + a0 * t + 0.5 * j * (t ** 2);
    return [v, a0 + j * t];
  }
  
  export function calculateSCurve(q0, q1, v0, v1, v_max, a_max, j_max, d) {
    const dv = Math.abs(v1 - v0);
    const dq = Math.abs(q1 - q0);
    const time_to_reach_max_a = a_max / j_max;
    const time_to_set_speeds = Math.sqrt(dv / j_max);
    let Tj = Math.min(time_to_reach_max_a, time_to_set_speeds);
    
    if (Tj === time_to_reach_max_a) {
      if (!(dq > 0.5 * (v0 + v1) * (Tj + dv / a_max))) {
        throw new Error("Something went wrong");
      }
    } else if (Tj < time_to_reach_max_a) {
      if (!(dq > Tj * (v0 + v1))) {
        throw new Error("Something went wrong");
      }
    } else {
      throw new Error("Something went wrong");
    }
    
    let Tj1, Ta;
    if ((v_max - v0) * j_max < a_max ** 2) {
      Tj1 = Math.sqrt((v_max - v0) / j_max);
      Ta = 2 * Tj1;
    } else {
      Tj1 = a_max / j_max;
      Ta = Tj1 + (v_max - v0) / a_max;
    }
    
    let Tj2, Td;
    if ((v_max - v1) * j_max < a_max ** 2) {
      Tj2 = Math.sqrt((v_max - v1) / j_max);
      Td = 2 * Tj2;
    } else {
      Tj2 = a_max / j_max;
      Td = Tj2 + (v_max - v1) / a_max;
    }
    
    let Tv = (q1 - q0) / v_max - ((Ta / 2) * (1 + v0 / v_max)) - ((Td / 2) * (1 + v1 / v_max));
    if (Tv < 0) {
      Tj1 = Tj2 = Tj = a_max / j_max;
      Tv = 0;
      const v = (a_max ** 2) / j_max;
      const delta = ((a_max ** 4) / (j_max ** 2)) + 2 * (v0 ** 2 + v1 ** 2) +
                    a_max * (4 * (q1 - q0) - 2 * (a_max / j_max) * (v0 + v1));
      Ta = (v - 2 * v0 + Math.sqrt(delta)) / (2 * a_max);
      Td = (v - 2 * v1 + Math.sqrt(delta)) / (2 * a_max);
    }
    const T = Ta + Td + Tv;
    const a_lim_a = j_max * Tj1;
    const a_lim_d = -j_max * Tj2;
    const v_lim = v0 + (Ta - Tj1) * a_lim_a;
    const EPSILON = 0.00001;
    if ((Ta - 2 * Tj) < EPSILON || (Td - 2 * Tj) < EPSILON) {
      throw new Error("Something went wrong");
    }
    const x1 = q0 + v0 * Tj1 + (a_lim_a * Math.pow(Tj1, 2)) / 6;
    const x2 = q0 + ((v_lim + v0) * Ta) / 2 - v_lim * Tj1 + j_max * Math.pow(Tj1, 3) / 6;
    const x3 = q0 + ((v_lim + v0) * Ta) / 2;
    const x4 = q1 - ((v_lim + v1) * Td) / 2;
    const x5 = q1 - ((v_lim + v1) * Td) / 2 + v_lim * Tj2 + (a_lim_d * Math.pow(Tj2, 2)) / 6;
    const x6 = q1 - v1 * Tj2 - j_max * Math.pow(Tj2, 3) / 6;
    const x7 = q1;
    
    let v_out, a_out;
    if (0 <= d && d < x1) {
      [v_out, a_out] = findV(0, d, v0, 0, j_max);
    } else if (d < x2) {
      v0 = v0 + a_lim_a * (Tj1 / 2);
      [v_out, a_out] = findV(x1, d, v0, a_lim_a, 0);
    } else if (d < x3) {
      v0 = v_lim - j_max * (Tj1 ** 2) / 2;
      [v_out, a_out] = findV(x2, d, v0, a_lim_a, -j_max);
    } else if (d < x4) {
      [v_out, a_out] = findV(x3, d, v_lim, 0, 0);
    } else if (d < x5) {
      [v_out, a_out] = findV(x4, d, v_lim, 0, -j_max);
    } else if (d < x6) {
      v0 = v_lim + a_lim_d * (Tj2 / 2);
      [v_out, a_out] = findV(x5, d, v0, a_lim_d, 0);
    } else if (d < x7) {
      v0 = v1 + j_max * (Tj2 ** 2) / 2;
      [v_out, a_out] = findV(x6, d, v0, a_lim_d, j_max);
    } else {
      a_out = 0;
      v_out = v1;
    }
    return [v_out, a_out];
  }