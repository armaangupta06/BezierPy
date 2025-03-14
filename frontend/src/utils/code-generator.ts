/**
 * Utility functions for generating C++ code for motion profiling
 */

import { PointModel, PoseModel, BezierCurveModel } from '@/services/api';

/**
 * Round a number to a specified number of decimal places
 */
const roundTo = (num: number, decimals: number = 1): number => {
  const factor = Math.pow(10, decimals);
  return Math.round(num * factor) / factor;
};

/**
 * Generate C++ code for motion profiling based on path creation method
 */
export const generateMotionProfileCode = (
  pathCreationMethod: 'poses' | 'points' | 'control-points',
  poses: PoseModel[],
  points: PointModel[],
  controlPointsList: BezierCurveModel[],
  initialHeading?: number,
  finalHeading?: number,
  tangentMagnitude: number = 0.8,
  trajectoryParams: {
    initialVelocity: number;
    finalVelocity: number;
    maxVelocity: number;
    acceleration: number;
    deceleration: number;
    maxJerk: number;
    maxAngularVelocity: number;
    useTrapezoidalProfile: boolean;
  } = {
    initialVelocity: 0,
    finalVelocity: 0,
    maxVelocity: 100,
    acceleration: 100,
    deceleration: 100,
    maxJerk: 500,
    maxAngularVelocity: 180,
    useTrapezoidalProfile: true,
  },
  reversed: boolean = false,
  areControlPointsEdited: boolean = false
): string => {
  console.log('generateMotionProfileCode called with:', {
    areControlPointsEdited,
    pathCreationMethod,
    numPoints: points.length,
    numPoses: poses.length,
    numControlPoints: controlPointsList.length
  });

  // Follow the prioritization logic:
  // If control points have been edited, use them
  if (areControlPointsEdited && controlPointsList.length > 0) {
    return generateControlPointsCode(controlPointsList, trajectoryParams, reversed);
  }

  // Otherwise, use points/poses based on the method and available data
  if (pathCreationMethod === 'points' && points.length >= 2 && initialHeading !== undefined) {
    return generatePointsCode(points, initialHeading, finalHeading, tangentMagnitude, trajectoryParams, reversed);
  }
  if (pathCreationMethod === 'poses' && poses.length >= 2) {
    return generatePosesCode(poses, tangentMagnitude, trajectoryParams, reversed);
  }
  if (pathCreationMethod === 'control-points' && controlPointsList.length > 0) {
    return generateControlPointsCode(controlPointsList, trajectoryParams, reversed);
  }
  if (points.length >= 2 && initialHeading !== undefined) {
    return generatePointsCode(points, initialHeading, finalHeading, tangentMagnitude, trajectoryParams, reversed);
  }
  if (poses.length >= 2) {
    return generatePosesCode(poses, tangentMagnitude, trajectoryParams, reversed);
  }
  if (controlPointsList.length > 0) {
    return generateControlPointsCode(controlPointsList, trajectoryParams, reversed);
  }
  return '// No path data available';
};

/**
 * Generate C++ code for motion profiling with points
 */
const generatePointsCode = (
  points: PointModel[],
  initialHeading?: number,
  finalHeading?: number,
  tangentMagnitude: number = 0.8,
  trajectoryParams: any = {},
  reversed: boolean = false
): string => {
  if (points.length < 2) {
    return '// Not enough points to generate a path';
  }

  const pointsVector = points.map(p => `Point{${roundTo(p.x)}, ${roundTo(p.y)}}`).join(', ');
  const finalAngle = finalHeading !== undefined ? roundTo(finalHeading) : 'current_heading()';

  return `chassis.motion_profiling({${pointsVector}}, ${typeof finalAngle === 'number' ? finalAngle : finalAngle}, ${roundTo(tangentMagnitude)}, ${roundTo(trajectoryParams.finalVelocity)}, ${roundTo(trajectoryParams.maxVelocity)}, ${roundTo(trajectoryParams.acceleration)}, ${roundTo(Math.abs(trajectoryParams.deceleration))}, ${roundTo(trajectoryParams.maxAngularVelocity)}, ${reversed});`;
};

/**
 * Generate C++ code for motion profiling with poses
 */
const generatePosesCode = (
  poses: PoseModel[],
  tangentMagnitude: number = 0.8,
  trajectoryParams: any = {},
  reversed: boolean = false
): string => {
  if (poses.length < 2) {
    return '// Not enough poses to generate a path';
  }

  const posesVector = poses.map(p => `Pose{${roundTo(p.x)}, ${roundTo(p.y)}, ${roundTo(p.heading)}}`).join(', ');

  return `chassis.motion_profiling({${posesVector}}, ${roundTo(tangentMagnitude)}, ${roundTo(trajectoryParams.finalVelocity)}, ${roundTo(trajectoryParams.maxVelocity)}, ${roundTo(trajectoryParams.acceleration)}, ${roundTo(Math.abs(trajectoryParams.deceleration))}, ${roundTo(trajectoryParams.maxAngularVelocity)}, ${reversed});`;
};

/**
 * Generate C++ code for motion profiling with control points
 */
const generateControlPointsCode = (
  controlPointsList: BezierCurveModel[],
  trajectoryParams: any = {},
  reversed: boolean = false
): string => {
  if (controlPointsList.length < 1) {
    return '// No Bezier curves to generate a path';
  }

  const curvesCode = controlPointsList.map(curve => {
    const points = curve.control_points;
    if (points.length !== 6) {
      return '// Invalid control points';
    }

    return `Quintic_Bezier{Point{${roundTo(points[0].x)}, ${roundTo(points[0].y)}}, Point{${roundTo(points[1].x)}, ${roundTo(points[1].y)}}, Point{${roundTo(points[2].x)}, ${roundTo(points[2].y)}}, Point{${roundTo(points[3].x)}, ${roundTo(points[3].y)}}, Point{${roundTo(points[4].x)}, ${roundTo(points[4].y)}}, Point{${roundTo(points[5].x)}, ${roundTo(points[5].y)}}}`;
  }).join(', ');

  return `chassis.motion_profiling({${curvesCode}}, ${roundTo(trajectoryParams.finalVelocity)}, ${roundTo(trajectoryParams.maxVelocity)}, ${roundTo(trajectoryParams.acceleration)}, ${roundTo(Math.abs(trajectoryParams.deceleration))}, ${roundTo(trajectoryParams.maxAngularVelocity)}, ${reversed});`;
};
