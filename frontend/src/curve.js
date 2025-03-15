import { Point, magnitude, distance_formula, dot_product, determinant, getPerpendicularVector } from './Point.js';
import { Pose } from './Pose.js';
import { Path_Point } from './Path_Point.js';
import { Quintic_Bezier } from './Quintic_Bezier.js';

/**
 * Generates points from a list of Quintic_Bezier objects.
 * Matches Python's 'generate_points' function.
 */
export function generatePoints(path) {
    const points = [];
    for (const curve of path) {
      // Loop t from 0 to 1 (inclusive) in steps of 0.01
      for (let t = 0; t <= 1.001; t += 0.01) {
        const dydx = curve.calcFirstDerivative(t);
        // Matches Python: atan2(dydx.x, dydx.y)
        const theta = Math.atan2(dydx.x, dydx.y) * (180 / Math.PI);
        const velocity = magnitude(dydx);
        
        // In Python: Path_Point(curve.get_point(t), curvature, velocity=velocity, theta=theta)
        // => param order = (point, curvature, velocity, theta)
        points.push(
          new Path_Point(
            curve.getPoint(t),
            curve.calcCurvature(t),
            velocity,
            theta
          )
        );
      }
    }
    return points;
  }
  
  /**
   * Calculates the trajectory along a path.
   * Matches Python's 'calculate_trajectory' function.
   */
  export function calculateTrajectory(path, v0, v1, max_v, a_accel, a_decel, max_j, max_w, trap = true) {
    const steps = [];
    let arcLength = 0.0;
    let s = 0.01;  // ‘s’ tracks distance traveled along the total path
    
    // Sum total arc length over all curves (Python: for curve in path: arcLength += curve.calc_arc_length())
    for (const curve of path) {
      arcLength += curve.calc_arc_length();
    }
    
    // Go curve by curve
    for (const curve of path) {
      const dt = 0.01;
      let t = 0.0;
      
      // While we have not traversed the curve fully, and haven't exceeded total arcLength
      while (t < 1.0 && s < arcLength) {
        const deriv = curve.calcFirstDerivative(t);
        const curvature = curve.calcCurvature(t, deriv);
        
        // Same "max reachable velocity" logic
        const max_reachable_velocity =
          (max_v * max_w) / (Math.abs(curvature) * max_v + max_w);
  
        let velocity;
        if (trap) {
          velocity = Math.min(
            trapezoidalMotionProfile(s, arcLength, v0, v1, max_v, a_accel, a_decel),
            max_reachable_velocity
          );
        } else {
          // SCurve returns [velocity, acceleration]
          velocity = calculateSCurve(0, arcLength, v0, v1, max_v, a_accel, max_j, s)[0];
        }
        
        const pose = curve.getPoint(t);
        
        // Increase the “distance” traveled along the path
        const deltaS = velocity * dt;
        s += deltaS;
        
        // Convert ΔS to Δt by dividing by magnitude of derivative (the speed in parametric space)
        const magVelocity = magnitude(deriv);
        const delta_t = deltaS / magVelocity;
        t += delta_t;
        
        // In Python, they just do: steps.append(Path_Point(pose, curvature, velocity=velocity))
        // We replicate: new Path_Point(..., curvature, velocity)
        // If your Path_Point constructor is (point, curvature, velocity, theta),
        // you can pass 'undefined' or 0 for theta here since Python does not store it in calc_trajectory.
        steps.push(new Path_Point(pose, curvature, velocity));
      }
    }
    return steps;
  }
  
  /**
   * Simple debugging / placeholder for graphing. 
   * In Python we have matplotlib; here we just log to console or you might use a charting library.
   */
  function graphPath(data) {
    console.log("=== Graph: Points ===");
    console.log(data.map(pt => ({ x: pt.x, y: pt.y })));
    console.log("=== Graph: Velocity ===");
    console.log(data.map((pt, i) => ({ i, velocity: pt.velocity })));
    console.log("=== Graph: Curvature ===");
    console.log(data.map((pt, i) => ({ i, curvature: pt.curvature })));
  }
  
  /**
   * Creates a spline path through raw (x,y) waypoints, optionally using an initial/final heading.
   * Matches Python's path_with_points(...).
   * @param {Array} points - Array of points
   * @param {number} initial_heading - Initial heading in degrees
   * @param {number|null|undefined} final_heading - Final heading in degrees, can be null or undefined
   * @param {number} v - Velocity
   * @param {number} a - Acceleration
   * @param {number} tangent_magnitude - Tangent magnitude
   * @returns {Array} Array of Quintic_Bezier curves
   */
  export function pathWithPoints(points, initial_heading, final_heading = null, v = 100, a = 10, tangent_magnitude = 0.5) {
    const path = [];
    
    // Convert degrees => radians
    initial_heading *= (Math.PI / 180);
    if (final_heading !== null) {
      final_heading *= (Math.PI / 180);
    }
    
    for (let curr = 0; curr < points.length - 1; curr++) {
      const curve = calcBezierCurveWithPoints(
        curr,
        points,
        initial_heading,
        final_heading,
        tangent_magnitude
      );
      path.push(curve);
    }
    return path;
  }
  
  /**
   * Creates a spline path from an array of "poses" [ (x,y,heading), ... ] 
   * Matches Python's path_with_poses(...).
   */
  export function pathWithPoses(poses, v = 100, a = 10, tangent_magnitude = 0.5) {
    const path = [];
    for (let curr = 0; curr < poses.length - 1; curr++) {
      const curve = calcBezierCurveWithPoses(curr, poses, tangent_magnitude);
      path.push(curve);
    }
    return path;
  }
  
  /**
   * Matches Python's trapezoidal_motion_profile.
   */
  function trapezoidalMotionProfile(distance, total_dist, v0, v1, v_max, a_accel, a_decel) {
    let cruise_velocity = Math.sqrt(
      (2 * total_dist * a_accel * a_decel + a_decel * v0 * v0 - a_accel * v1 * v1) /
      (a_decel - a_accel)
    );
    cruise_velocity = Math.min(v_max, cruise_velocity);
    
    const v_forward = Math.sqrt(v0 * v0 + 2 * a_accel * distance);
    const v_back    = Math.sqrt(v1 * v1 - 2 * a_decel * (total_dist - distance));
    
    return Math.min(cruise_velocity, v_forward, v_back);
  }
  
  /**
   * Matches Python's calc_velocity (simple forward/backward pass).
   * (Requires distance_formula(...) to measure distance between points.)
   */
  function calcVelocity(path, v, a) {
    if (path.length === 0) return path;
    
    // Python sets the last point's velocity to 0
    path[path.length - 1].velocity = 0;
    
    // Backward pass limiting velocities
    for (let i = path.length - 2; i >= 0; i--) {
      const curv = (path[i].curvature === 0) ? 0.001 : path[i].curvature;
      const desired_velocity = Math.min(v, 3.0 / Math.abs(curv));
      const dist = distance_formula(path[i + 1], path[i]);
      
      const limited_velocity = Math.sqrt(
        path[i + 1].velocity * path[i + 1].velocity + 2 * a * dist
      );
      path[i].velocity = Math.min(desired_velocity, limited_velocity);
    }
    return path;
  }
  
  /**
   * Matches Python's calc_bezier_curve_with_poses.
   * Be sure you have or define Point(...) and its associated vector methods 
   * (add, subtract, multiply, round, etc.) plus getPerpendicularVector as needed.
   */
  function calcBezierCurveWithPoses(curr, path, tangent_magnitude) {
    // v0, v1: unit direction vectors from heading
    let v0 = new Point(Math.sin(path[curr].heading), Math.cos(path[curr].heading));
    let v1 = new Point(Math.sin(path[curr + 1].heading), Math.cos(path[curr + 1].heading));
    
    let acc0, acc1;
    
    // Exactly follow the Python logic
    if (curr === 0) {
      if (path.length === 2) {
        const magnitudev0 = tangent_magnitude * magnitude(path[curr + 1].subtract(path[curr]));
        v0 = v0.multiply(magnitudev0);
        v1 = v1.multiply(magnitudev0);
        
        acc0 = path[curr].multiply(-6)
               .add(v0.multiply(-4))
               .add(v1.multiply(-2))
               .add(path[curr + 1].multiply(6));
        acc1 = path[curr].multiply(6)
               .add(v0.multiply(2))
               .add(v1.multiply(4))
               .add(path[curr + 1].multiply(-6));
      } else {
        // Next "vD"
        let vD;
        if (curr + 3 === path.length) {
          const magnitudevD = tangent_magnitude *
            magnitude(path[curr + 2].subtract(path[curr + 1]));
          vD = new Point(
            Math.sin(path[curr + 2].heading),
            Math.cos(path[curr + 2].heading)
          ).multiply(magnitudevD);
        } else {
          const magnitudevD = tangent_magnitude * Math.min(
            magnitude(path[curr + 2].subtract(path[curr + 1])),
            magnitude(path[curr + 3].subtract(path[curr + 2]))
          );
          vD = new Point(
            Math.sin(path[curr + 2].heading),
            Math.cos(path[curr + 2].heading)
          ).multiply(magnitudevD);
        }
        
        const magnitudev0 = tangent_magnitude *
          magnitude(path[curr + 1].subtract(path[curr]));
        const magnitudev1 = tangent_magnitude * Math.min(
          magnitude(path[curr + 1].subtract(path[curr])),
          magnitude(path[curr + 2].subtract(path[curr + 1]))
        );
        
        v0 = v0.multiply(magnitudev0);
        v1 = v1.multiply(magnitudev1);
        
        const alpha = magnitude(path[curr + 2].subtract(path[curr + 1])) /
          (magnitude(path[curr + 1].subtract(path[curr])) + magnitude(path[curr + 2].subtract(path[curr + 1])));
        const beta = magnitude(path[curr + 1].subtract(path[curr])) /
          (magnitude(path[curr + 1].subtract(path[curr])) + magnitude(path[curr + 2].subtract(path[curr + 1])));
        
        console.log(alpha, beta);
        acc0 = path[curr].multiply(-6)
               .add(v0.multiply(-4))
               .add(v1.multiply(-2))
               .add(path[curr + 1].multiply(6));
        acc1 = (path[curr].multiply(6)
               .add(v0.multiply(2))
               .add(v1.multiply(4))
               .add(path[curr + 1].multiply(-6))).multiply(alpha)
               .add(
                (path[curr + 1].multiply(-6)
                .add(v1.multiply(-4))
                .add(vD.multiply(-2))
                .add(path[curr + 2].multiply(6))).multiply(beta));
        console.log(acc1);
      }
    } else if (curr === path.length - 2) {
      // Second to last curve...
      let vA;
      if (curr - 1 === 0) {
        const magnitudevA = tangent_magnitude * magnitude(path[curr].subtract(path[curr - 1]));
        vA = new Point(
          Math.sin(path[curr - 1].heading),
          Math.cos(path[curr - 1].heading)
        ).multiply(magnitudevA);
      } else {
        const magnitudevA = tangent_magnitude * Math.min(
          magnitude(path[curr - 2].subtract(path[curr - 1])),
          magnitude(path[curr].subtract(path[curr - 1]))
        );
        vA = new Point(
          Math.sin(path[curr - 1].heading),
          Math.cos(path[curr - 1].heading)
        ).multiply(magnitudevA);
      }
      
      const magnitudev0 = tangent_magnitude * Math.min(
        magnitude(path[curr].subtract(path[curr - 1])),
        magnitude(path[curr + 1].subtract(path[curr]))
      );
      const magnitudev1 = tangent_magnitude *
        magnitude(path[curr + 1].subtract(path[curr]));
      
      v0 = v0.multiply(magnitudev0);
      v1 = v1.multiply(magnitudev1);
      
      const alpha = magnitude(path[curr + 1].subtract(path[curr])) /
        (magnitude(path[curr].subtract(path[curr - 1])) + magnitude(path[curr + 1].subtract(path[curr])));
      const beta  = magnitude(path[curr].subtract(path[curr - 1])) /
        (magnitude(path[curr].subtract(path[curr - 1])) + magnitude(path[curr + 1].subtract(path[curr])));
      
      acc0 = (
        path[curr - 1].multiply(6)
          .add(vA.multiply(2))
          .add(v0.multiply(4))
          .add(path[curr].multiply(-6))
      ).multiply(alpha).add((
        path[curr].multiply(-6)
          .add(v0.multiply(-4))
          .add(v1.multiply(-2))
          .add(path[curr + 1].multiply(6))
      ).multiply(beta));
      acc1 = path[curr].multiply(6)
             .add(v0.multiply(2))
             .add(v1.multiply(4))
             .add(path[curr + 1].multiply(-6));
    } else {
      // Middle of the path
      let vA;
      if (curr - 1 === 0) {
        const magnitudevA = tangent_magnitude * magnitude(path[curr].subtract(path[curr - 1]));
        vA = new Point(
          Math.sin(path[curr - 1].heading),
          Math.cos(path[curr - 1].heading)
        ).multiply(magnitudevA);
      } else {
        const magnitudevA = tangent_magnitude * Math.min(
          magnitude(path[curr - 2].subtract(path[curr - 1])),
          magnitude(path[curr].subtract(path[curr - 1]))
        );
        vA = new Point(
          Math.sin(path[curr - 1].heading),
          Math.cos(path[curr - 1].heading)
        ).multiply(magnitudevA);
      }
      
      let vD;
      if (curr + 2 === path.length - 1) {
        const magnitudevD = tangent_magnitude *
          magnitude(path[curr + 2].subtract(path[curr + 1]));
        vD = new Point(
          Math.sin(path[curr + 2].heading),
          Math.cos(path[curr + 2].heading)
        ).multiply(magnitudevD);
      } else {
        const magnitudevD = tangent_magnitude * Math.min(
          magnitude(path[curr + 2].subtract(path[curr + 1])),
          magnitude(path[curr + 3].subtract(path[curr + 2]))
        );
        vD = new Point(
          Math.sin(path[curr + 2].heading),
          Math.cos(path[curr + 2].heading)
        ).multiply(magnitudevD);
      }
      
      const magnitudev0 = tangent_magnitude * Math.min(
        magnitude(path[curr].subtract(path[curr - 1])),
        magnitude(path[curr + 1].subtract(path[curr]))
      );
      const magnitudev1 = tangent_magnitude * Math.min(
        magnitude(path[curr + 1].subtract(path[curr])),
        magnitude(path[curr + 2].subtract(path[curr + 1]))
      );
      
      v0 = v0.multiply(magnitudev0);
      v1 = v1.multiply(magnitudev1);
      
      const alpha0 = magnitude(path[curr + 1].subtract(path[curr])) /
        (magnitude(path[curr].subtract(path[curr - 1])) + magnitude(path[curr + 1].subtract(path[curr])));
      const beta0 = magnitude(path[curr].subtract(path[curr - 1])) /
        (magnitude(path[curr].subtract(path[curr - 1])) + magnitude(path[curr + 1].subtract(path[curr])));
      
      const alpha1 = magnitude(path[curr + 2].subtract(path[curr + 1])) /
        (magnitude(path[curr + 1].subtract(path[curr])) + magnitude(path[curr + 2].subtract(path[curr + 1])));
      const beta1 = magnitude(path[curr + 1].subtract(path[curr])) /
        (magnitude(path[curr + 1].subtract(path[curr])) + magnitude(path[curr + 2].subtract(path[curr + 1])));
      
      acc0 =  (
        path[curr - 1].multiply(6)
          .add(vA.multiply(2))
          .add(v0.multiply(4))
          .add(path[curr].multiply(-6))
      ).multiply(alpha0).add((
        path[curr].multiply(-6)
          .add(v0.multiply(-4))
          .add(v1.multiply(-2))
          .add(path[curr + 1].multiply(6))
      ).multiply(beta0));
      console.log(alpha1, beta1, acc1);
      acc1 = (
        path[curr].multiply(6)
          .add(v0.multiply(2))
          .add(v1.multiply(4))
          .add(path[curr + 1].multiply(-6))
      ).multiply(alpha1).add((
        path[curr + 1].multiply(-6)
          .add(v1.multiply(-4))
          .add(vD.multiply(-2))
          .add(path[curr + 2].multiply(6))
      ).multiply(beta1));
      console.log(alpha1, beta1, acc1);
    }

    
    
    // Round the vectors (like Python's round(..., 10))
    v0   = v0.round(10);
    v1   = v1.round(10);
    acc0 = acc0.round(10);
    acc1 = acc1.round(10);
    
    const point0 = path[curr];
    const point5 = path[curr + 1];
    const point1 = point0.add(v0.multiply(1/5));
    const point2 = point1.multiply(2).subtract(point0).add(acc0.multiply(1/20));
    const point4 = point5.subtract(v1.multiply(1/5));
    const point3 = point4.multiply(2).subtract(point5).add(acc1.multiply(1/20));
    
    return new Quintic_Bezier(point0, point1, point2, point3, point4, point5);
  }
  
  /**
   * Matches Python's calc_bezier_curve_with_points.
   */
  function calcBezierCurveWithPoints(curr, path, initial_heading, final_heading, tangent_magnitude) {
    let v0, v1, acc0, acc1;
    
    if (curr === 0) {
      if (path.length === 2) {
        const magnitudev0 = tangent_magnitude * magnitude(path[1].subtract(path[0]));
        v0 = new Point(Math.sin(initial_heading), Math.cos(initial_heading)).multiply(magnitudev0);
        if (final_heading === null) {
          const diff = path[1].subtract(path[0]);
          v1 = diff.divide(magnitude(diff)).multiply(magnitudev0);
        } else {
          v1 = new Point(Math.sin(final_heading), Math.cos(final_heading)).multiply(magnitudev0);
        }
        acc0 = path[0].multiply(-6)
               .add(v0.multiply(-4))
               .add(v1.multiply(-2))
               .add(path[1].multiply(6));
        acc1 = path[0].multiply(6)
               .add(v0.multiply(2))
               .add(v1.multiply(4))
               .add(path[1].multiply(-6));
      } else {
        // We have more than 2 waypoints
        let vD;
        if (curr + 2 === path.length - 1) {
          const magnitudevD = tangent_magnitude *
            magnitude(path[curr + 2].subtract(path[curr + 1]));
          if (final_heading === null) {
            const diff = path[curr + 2].subtract(path[curr + 1]);
            vD = diff.divide(magnitude(diff)).multiply(magnitudevD);
          } else {
            vD = new Point(Math.sin(final_heading), Math.cos(final_heading)).multiply(magnitudevD);
          }
        } else {
          const magnitudevD = tangent_magnitude * Math.min(
            magnitude(path[curr + 2].subtract(path[curr + 1])),
            magnitude(path[curr + 3].subtract(path[curr + 2]))
          );
          // getPerpendicularVector is used in the Python code in the middle cases 
          // (we do the same logic).
          vD = getPerpendicularVector(path[curr], path[curr + 1], path[curr + 2])
               .multiply(magnitudevD);
        }
        
        const magnitudev0 = tangent_magnitude *
          magnitude(path[curr + 1].subtract(path[curr]));
        const magnitudev1 = tangent_magnitude * Math.min(
          magnitude(path[curr + 1].subtract(path[curr])),
          magnitude(path[curr + 2].subtract(path[curr + 1]))
        );
        v0 = new Point(Math.sin(initial_heading), Math.cos(initial_heading)).multiply(magnitudev0);
        v1 = getPerpendicularVector(path[curr], path[curr + 1], path[curr + 2])
               .multiply(magnitudev1);
        
        const alpha = magnitude(path[curr + 2].subtract(path[curr + 1])) /
          (magnitude(path[curr + 1].subtract(path[curr])) + magnitude(path[curr + 2].subtract(path[curr + 1])));
        const beta = magnitude(path[curr + 1].subtract(path[curr])) /
          (magnitude(path[curr + 1].subtract(path[curr])) + magnitude(path[curr + 2].subtract(path[curr + 1])));
        
        acc0 = path[curr].multiply(-6)
               .add(v0.multiply(-4))
               .add(v1.multiply(-2))
               .add(path[curr + 1].multiply(6));
        acc1 = (
          path[curr].multiply(6)
            .add(v0.multiply(2))
            .add(v1.multiply(4))
            .add(path[curr + 1].multiply(-6))
        ).multiply(alpha).add((
          path[curr + 1].multiply(-6)
            .add(v1.multiply(-4))
            .add(vD.multiply(-2))
            .add(path[curr + 2].multiply(6))
        ).multiply(beta));
      }
    } else if (curr === path.length - 2) {
      let vA;
      if (curr - 1 === 0) {
        const magnitudevA = tangent_magnitude * magnitude(path[curr].subtract(path[curr - 1]));
        vA = new Point(Math.sin(initial_heading), Math.cos(initial_heading)).multiply(magnitudevA);
      } else {
        const magnitudevA = tangent_magnitude * Math.min(
          magnitude(path[curr - 2].subtract(path[curr - 1])),
          magnitude(path[curr].subtract(path[curr - 1]))
        );
        vA = getPerpendicularVector(path[curr - 2], path[curr - 1], path[curr])
             .multiply(magnitudevA);
      }
      
      const magnitudev0 = tangent_magnitude * Math.min(
        magnitude(path[curr].subtract(path[curr - 1])),
        magnitude(path[curr + 1].subtract(path[curr]))
      );
      const magnitudev1 = tangent_magnitude *
        magnitude(path[curr + 1].subtract(path[curr]));
      
      v0 = getPerpendicularVector(path[curr - 1], path[curr], path[curr + 1])
             .multiply(magnitudev0);
      
      if (final_heading === null) {
        const diff = path[curr + 1].subtract(path[curr]);
        v1 = diff.divide(magnitude(diff)).multiply(magnitudev1);
      } else {
        v1 = new Point(Math.sin(final_heading), Math.cos(final_heading)).multiply(magnitudev1);
      }
      
      const alpha = magnitude(path[curr + 1].subtract(path[curr])) /
        (magnitude(path[curr].subtract(path[curr - 1])) + magnitude(path[curr + 1].subtract(path[curr])));
      const beta = magnitude(path[curr].subtract(path[curr - 1])) /
        (magnitude(path[curr].subtract(path[curr - 1])) + magnitude(path[curr + 1].subtract(path[curr])));
      
      acc0 = (
        path[curr - 1].multiply(6)
          .add(vA.multiply(2))
          .add(v0.multiply(4))
          .add(path[curr].multiply(-6))
      ).multiply(alpha).add((
        path[curr].multiply(-6)
          .add(v0.multiply(-4))
          .add(v1.multiply(-2))
          .add(path[curr + 1].multiply(6))
      ).multiply(beta));
      acc1 = path[curr].multiply(6)
             .add(v0.multiply(2))
             .add(v1.multiply(4))
             .add(path[curr + 1].multiply(-6));
    } else {
      // Middle of path
      let vA;
      if (curr - 1 === 0) {
        const magnitudevA = tangent_magnitude * magnitude(path[curr].subtract(path[curr - 1]));
        vA = new Point(Math.sin(initial_heading), Math.cos(initial_heading)).multiply(magnitudevA);
      } else {
        const magnitudevA = tangent_magnitude * Math.min(
          magnitude(path[curr - 2].subtract(path[curr - 1])),
          magnitude(path[curr].subtract(path[curr - 1]))
        );
        vA = getPerpendicularVector(path[curr - 2], path[curr - 1], path[curr])
             .multiply(magnitudevA);
      }
      
      let vD;
      if (curr + 2 === path.length - 1) {
        const magnitudevD = tangent_magnitude *
          magnitude(path[curr + 2].subtract(path[curr + 1]));
        if (final_heading === null) {
          const diff = path[curr + 2].subtract(path[curr + 1]);
          vD = diff.divide(magnitude(diff)).multiply(magnitudevD);
        } else {
          vD = new Point(Math.sin(final_heading), Math.cos(final_heading)).multiply(magnitudevD);
        }
      } else {
        const magnitudevD = tangent_magnitude * Math.min(
          magnitude(path[curr + 2].subtract(path[curr + 1])),
          magnitude(path[curr + 3].subtract(path[curr + 2]))
        );
        vD = getPerpendicularVector(path[curr + 1], path[curr + 2], path[curr + 3])
             .multiply(magnitudevD);
      }
      
      const magnitudev0 = tangent_magnitude * Math.min(
        magnitude(path[curr].subtract(path[curr - 1])),
        magnitude(path[curr + 1].subtract(path[curr]))
      );
      const magnitudev1 = tangent_magnitude * Math.min(
        magnitude(path[curr + 1].subtract(path[curr])),
        magnitude(path[curr + 2].subtract(path[curr + 1]))
      );
      
      v0 = getPerpendicularVector(path[curr - 1], path[curr], path[curr + 1])
           .multiply(magnitudev0);
      v1 = getPerpendicularVector(path[curr], path[curr + 1], path[curr + 2])
           .multiply(magnitudev1);
      
      const alpha0 = magnitude(path[curr + 1].subtract(path[curr])) /
        (magnitude(path[curr].subtract(path[curr - 1])) + magnitude(path[curr + 1].subtract(path[curr])));
      const beta0 = magnitude(path[curr].subtract(path[curr - 1])) /
        (magnitude(path[curr].subtract(path[curr - 1])) + magnitude(path[curr + 1].subtract(path[curr])));
      
      const alpha1 = magnitude(path[curr + 2].subtract(path[curr + 1])) /
        (magnitude(path[curr + 1].subtract(path[curr])) + magnitude(path[curr + 2].subtract(path[curr + 1])));
      const beta1 = magnitude(path[curr + 1].subtract(path[curr])) /
        (magnitude(path[curr + 1].subtract(path[curr])) + magnitude(path[curr + 2].subtract(path[curr + 1])));
      
      acc0 = (
        path[curr - 1].multiply(6)
          .add(vA.multiply(2))
          .add(v0.multiply(4))
          .add(path[curr].multiply(-6))
      ).multiply(alpha0).add((
        path[curr].multiply(-6)
          .add(v0.multiply(-4))
          .add(v1.multiply(-2))
          .add(path[curr + 1].multiply(6))
      ).multiply(beta0));
      acc1 = (
        path[curr].multiply(6)
          .add(v0.multiply(2))
          .add(v1.multiply(4))
          .add(path[curr + 1].multiply(-6))
      ).multiply(alpha1).add((
        path[curr + 1].multiply(-6)
          .add(v1.multiply(-4))
          .add(vD.multiply(-2))
          .add(path[curr + 2].multiply(6))
      ).multiply(beta1));
    }
    
    v0   = v0.round(10);
    v1   = v1.round(10);
    acc0 = acc0.round(10);
    acc1 = acc1.round(10);
    
    const point0 = path[curr];
    const point5 = path[curr + 1];
    const point1 = point0.add(v0.multiply(1/5));
    const point2 = point1.multiply(2).subtract(point0).add(acc0.multiply(1/20));
    const point4 = point5.subtract(v1.multiply(1/5));
    const point3 = point4.multiply(2).subtract(point5).add(acc1.multiply(1/20));
    
    return new Quintic_Bezier(point0, point1, point2, point3, point4, point5);
  }
  
  /** 
   * Solve a polynomial a x^3 + b x^2 + c x + d = 0 (mimicking Python's solve).
   */
  function solve(a, b, c, d) {
    if (a === 0 && b === 0) {
      // Linear
      return [(-d / c)];
    } else if (a === 0) {
      // Quadratic
      let D = c * c - 4 * b * d;
      if (D >= 0) {
        const sqrtD = Math.sqrt(D);
        return [(-c + sqrtD) / (2 * b), (-c - sqrtD) / (2 * b)];
      } else {
        const sqrtD = Math.sqrt(-D);
        // Return complex roots as {re, im}
        return [
          { re: -c / (2 * b), im:  sqrtD / (2 * b) },
          { re: -c / (2 * b), im: -sqrtD / (2 * b) }
        ];
      }
    }
    
    // Cubic
    const f = findF(a, b, c);
    const g = findG(a, b, c, d);
    const h = findH(g, f);
    
    // All roots real and equal
    if (f === 0 && g === 0 && h === 0) {
      let x;
      if (d / a >= 0) {
        x = -Math.pow(d / a, 1/3);
      } else {
        x = Math.pow(-d / a, 1/3);
      }
      return [x, x, x];
    } else if (h <= 0) {
      // All roots are real
      const i = Math.sqrt((g * g) / 4 - h);
      const j = Math.cbrt(i);
      const k = Math.acos(-(g / (2 * i)));
      const L = -j;
      const M = Math.cos(k / 3);
      const N = Math.sqrt(3) * Math.sin(k / 3);
      const P = -b / (3 * a);
      const x1 = 2 * j * Math.cos(k / 3) - (b / (3 * a));
      const x2 = L * (M + N) + P;
      const x3 = L * (M - N) + P;
      return [x1, x2, x3];
    } else {
      // One real root + two complex
      const R = -g / 2 + Math.sqrt(h);
      const S = (R >= 0) ? Math.cbrt(R) : -Math.cbrt(-R);
      const T = -g / 2 - Math.sqrt(h);
      const U = (T >= 0) ? Math.cbrt(T) : -Math.cbrt(-T);
      const x1 = (S + U) - (b / (3 * a));
      return [x1];
    }
  }
  
  function findF(a, b, c) {
    return ((3 * c / a) - ((b * b) / (a * a))) / 3;
  }
  
  function findG(a, b, c, d) {
    return (
      ((2 * Math.pow(b, 3)) / Math.pow(a, 3))
      - ((9 * b * c) / (a * a))
      + ((27 * d) / a)
    ) / 27;
  }
  
  function findH(g, f) {
    return ((g * g) / 4) + ((f * f * f) / 27);
  }
  
  /**
   * Matches Python's 'findV' helper to compute (v, a) for local intervals in s-curve.
   */
  function findV(s0, s1, v0, a0, j) {
    const vals = solve((1/6)*j, (1/2)*a0, v0, (s0 - s1));
    if (vals.length > 1) {
      for (const t of vals) {
        // If t is real, we compute v
        const timeVal = (typeof t === 'object') ? t.re : t; // handle complex
        const v = v0 + a0 * timeVal + 0.5 * j * (timeVal * timeVal);
        if (v > 0) {
          return [v, a0 + j * timeVal];
        }
      }
    }
    // Otherwise just use the first root
    const t = (typeof vals[0] === 'object') ? vals[0].re : vals[0];
    const v = v0 + a0 * t + 0.5 * j * (t * t);
    return [v, a0 + j * t];
  }
  
  /**
   * Matches Python's calculate_s_curve(...) function.
   */
  function calculateSCurve(q0, q1, v0, v1, v_max, a_max, j_max, d) {
    const dv = Math.abs(v1 - v0);
    const dq = Math.abs(q1 - q0);
    
    const time_to_reach_max_a = a_max / j_max;
    const time_to_set_speeds = Math.sqrt(dv / j_max);
    let Tj = Math.min(time_to_reach_max_a, time_to_set_speeds);
    
    if (Tj === time_to_reach_max_a) {
      if (!(dq > 0.5 * (v0 + v1) * (Tj + dv / a_max))) {
        throw new Error("Something went wrong (SCurve check #1).");
      }
    } else if (Tj < time_to_reach_max_a) {
      if (!(dq > Tj * (v0 + v1))) {
        throw new Error("Something went wrong (SCurve check #2).");
      }
    } else {
      throw new Error("Something went wrong (SCurve check #3).");
    }
    
    let Tj1, Ta, Tj2, Td;
    if ((v_max - v0) * j_max < a_max * a_max) {
      Tj1 = Math.sqrt((v_max - v0) / j_max);
      Ta = 2 * Tj1;
    } else {
      Tj1 = a_max / j_max;
      Ta = Tj1 + (v_max - v0) / a_max;
    }
    
    if ((v_max - v1) * j_max < a_max * a_max) {
      Tj2 = Math.sqrt((v_max - v1) / j_max);
      Td = 2 * Tj2;
    } else {
      Tj2 = a_max / j_max;
      Td = Tj2 + (v_max - v1) / a_max;
    }
    
    let Tv = (q1 - q0) / v_max
      - (Ta / 2) * (1 + v0 / v_max)
      - (Td / 2) * (1 + v1 / v_max);
    
    if (Tv < 0) {
      Tj = a_max / j_max;
      Tj1 = Tj2 = Tj;
      Tv = 0;
      
      const v = (a_max * a_max) / j_max;
      const delta = 
        ((a_max ** 4) / (j_max * j_max))
        + 2 * (v0 * v0 + v1 * v1)
        + a_max * (4 * (q1 - q0) - 2 * (a_max / j_max) * (v0 + v1));
      
      Ta = (v - 2 * v0 + Math.sqrt(delta)) / (2 * a_max);
      Td = (v - 2 * v1 + Math.sqrt(delta)) / (2 * a_max);
    }
    
    const a_lim_a = j_max * Tj1;
    const a_lim_d = -j_max * Tj2;
    const v_lim = v0 + (Ta - Tj1) * a_lim_a;
    
    const EPSILON = 1e-5;
    if ((Ta - 2 * Tj) < EPSILON || (Td - 2 * Tj) < EPSILON) {
      throw new Error("Something went wrong (SCurve check #4).");
    }
    
    // "Switch" positions x1..x7
    const x1 = q0 + v0 * Tj1 + (a_lim_a * (Tj1 ** 2)) / 6;
    const x2 = q0 + ((v_lim + v0) * Ta / 2) - v_lim * Tj1 + (j_max * (Tj1 ** 3)) / 6;
    const x3 = q0 + ((v_lim + v0) * Ta / 2);
    const x4 = q1 - ((v_lim + v1) * Td / 2);
    const x5 = q1 - ((v_lim + v1) * Td / 2)
                 + v_lim * Tj2
                 + a_lim_d * (Tj2 ** 2) / 6;
    const x6 = q1 - v1 * Tj2 - j_max * (Tj2 ** 3) / 6;
    const x7 = q1;
    
    let a_val, v_ret;
    
    if (d >= 0 && d < x1) {
      [v_ret, a_val] = findV(0, d, v0, 0, j_max);
    } else if (d >= x1 && d < x2) {
      let newV0 = v0 + a_lim_a * (Tj1 / 2);
      [v_ret, a_val] = findV(x1, d, newV0, a_lim_a, 0);
    } else if (d >= x2 && d < x3) {
      let newV0 = v_lim - j_max * (Tj1 ** 2) / 2;
      [v_ret, a_val] = findV(x2, d, newV0, a_lim_a, -j_max);
    } else if (d >= x3 && d < x4) {
      [v_ret, a_val] = findV(x3, d, v_lim, 0, 0);
    } else if (d >= x4 && d < x5) {
      [v_ret, a_val] = findV(x4, d, v_lim, 0, -j_max);
    } else if (d >= x5 && d < x6) {
      let newV0 = v_lim + a_lim_d * (Tj2 / 2);
      [v_ret, a_val] = findV(x5, d, newV0, a_lim_d, 0);
    } else if (d >= x6 && d < x7) {
      let newV0 = v1 + j_max * (Tj2 ** 2) / 2;
      [v_ret, a_val] = findV(x6, d, newV0, a_lim_d, j_max);
    } else {
      a_val = 0;
      v_ret = v1;
    }
    
    return [v_ret, a_val];
  }
  
  /** 
   * Linear interpolation—mirrors Python's lerp(...)
   */
  function lerp(point0, point1, t) {
    return point0.multiply(1 - t).add(point1.multiply(t));
  }
  