import { BezierCurveModel, PointModel } from '@/services/api';

/**
 * Calculate a point on a quintic Bezier curve at parameter t
 * 
 * Formula:
 * p = (1-t)^5 * p0 + 5(1-t)^4 * t * p1 + 10(1-t)^3 * t^2 * p2 + 
 *     10(1-t)^2 * t^3 * p3 + 5(1-t) * t^4 * p4 + t^5 * p5
 */
export function calculateQuinticBezierPoint(
  controlPoints: PointModel[], 
  t: number
): PointModel {
  if (controlPoints.length !== 6) {
    throw new Error('Quintic Bezier curve requires exactly 6 control points');
  }

  const [p0, p1, p2, p3, p4, p5] = controlPoints;
  
  // Calculate the coefficients
  const t1 = 1 - t;
  const t1_5 = Math.pow(t1, 5);
  const t1_4 = Math.pow(t1, 4);
  const t1_3 = Math.pow(t1, 3);
  const t1_2 = Math.pow(t1, 2);
  const t_2 = Math.pow(t, 2);
  const t_3 = Math.pow(t, 3);
  const t_4 = Math.pow(t, 4);
  const t_5 = Math.pow(t, 5);
  
  // Calculate x coordinate
  const x = t1_5 * p0.x + 
            5 * t1_4 * t * p1.x + 
            10 * t1_3 * t_2 * p2.x + 
            10 * t1_2 * t_3 * p3.x + 
            5 * t1 * t_4 * p4.x + 
            t_5 * p5.x;
  
  // Calculate y coordinate
  const y = t1_5 * p0.y + 
            5 * t1_4 * t * p1.y + 
            10 * t1_3 * t_2 * p2.y + 
            10 * t1_2 * t_3 * p3.y + 
            5 * t1 * t_4 * p4.y + 
            t_5 * p5.y;
  
  return { x, y };
}

/**
 * Generate path points for a Bezier curve
 * @param curve A single Bezier curve with control points
 * @param numPoints Number of points to generate along the curve
 * @returns Array of points along the curve
 */
export function generateBezierCurvePoints(
  curve: BezierCurveModel,
  numPoints: number = 100
): PointModel[] {
  const points: PointModel[] = [];
  
  for (let i = 0; i <= numPoints; i++) {
    const t = i / numPoints;
    const point = calculateQuinticBezierPoint(curve.control_points, t);
    points.push(point);
  }
  
  return points;
}

/**
 * Generate path points for multiple Bezier curves
 * @param curves Array of Bezier curves
 * @param numPointsPerCurve Number of points to generate per curve
 * @returns Array of points along all curves
 */
export function generatePathPoints(
  curves: BezierCurveModel[],
  numPointsPerCurve: number = 100
): PointModel[] {
  if (!curves || curves.length === 0) {
    return [];
  }
  
  let allPoints: PointModel[] = [];
  
  curves.forEach((curve) => {
    const curvePoints = generateBezierCurvePoints(curve, numPointsPerCurve);
    
    // If this isn't the first curve, remove the first point to avoid duplication
    // (assuming curves are connected)
    if (allPoints.length > 0 && curvePoints.length > 0) {
      curvePoints.shift();
    }
    
    allPoints = [...allPoints, ...curvePoints];
  });
  
  return allPoints;
}
