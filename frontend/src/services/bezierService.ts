import { v4 as uuidv4 } from 'uuid';
import { 
  PointModel, 
  PoseModel, 
  PathParamsModel, 
  TrajectoryParamsModel, 
  BezierCurveModel,
  PathResponse,
  TrajectoryResponse
} from './api';

// Import the ES modules directly
import { Point, distance_formula, magnitude, dot_product, determinant, getPerpendicularVector } from '../Point.js';
import { Pose } from '../Pose.js';
import { Path_Point } from '../Path_Point.js';
import { Quintic_Bezier } from '../Quintic_Bezier.js';
import { pathWithPoses, pathWithPoints, generatePoints, calculateTrajectory } from '../curve.js';

// Type definitions for TypeScript
type PointType = Point;
type PoseType = Pose;
type PathPointType = Path_Point;
type QuinticBezierType = Quintic_Bezier;

// In-memory storage for paths and trajectories
const pathsStorage: Record<string, any> = {};
const trajectoriesStorage: Record<string, any> = {};

// Convert frontend PointModel to JavaScript Point
const pointModelToPoint = (point: PointModel): PointType => {
  return new Point(point.x, point.y);
};

// Convert frontend PoseModel to JavaScript Pose
const poseModelToPose = (pose: PoseModel): PoseType => {
  return new Pose(pose.x, pose.y, pose.heading);
};

// Convert JavaScript Point to frontend PointModel
const pointToPointModel = (point: PointType): PointModel => {
  return { x: point.x, y: point.y };
};

// Convert control points to JavaScript Points
const controlPointsToPoints = (controlPoints: PointModel[]): PointType[] => {
  return controlPoints.map(cp => pointModelToPoint(cp));
};

// Generate a unique ID
const generateUniqueId = (): string => {
  return uuidv4();
};

export const bezierService = {
  // Create path from points
  createPathFromPoints(data: {
    points: PointModel[];
    initial_heading: number;
    final_heading?: number;
    params?: PathParamsModel;
  }): PathResponse {
    // Convert to JavaScript Points
    const jsPoints = data.points.map(pointModelToPoint);
    
    // Generate path using our JavaScript implementation
    const bezierCurves = pathWithPoints(
      jsPoints,
      data.initial_heading,
      data.final_heading ?? null, // Use nullish coalescing to handle undefined
      100, // v (default)
      10,  // a (default)
      data.params?.tangent_magnitude || 0.5
    );
    
    // Generate discretized points
    const discretizedPoints = generatePoints(bezierCurves);
    
    // Generate a unique ID for the path
    const pathId = generateUniqueId();
    
    // Store the path
    pathsStorage[pathId] = {
      bezierCurves,
      discretizedPoints
    };
    
    // Convert curves to the expected format
    const curves = bezierCurves.map((curve: QuinticBezierType) => {
      return {
        control_points: curve.getControlPoints().map(pointToPointModel)
      };
    });
    
    // Return the response in the same format as the API
    return {
      path_id: pathId,
      curves,
      discretized_points: discretizedPoints.map((point: PathPointType) => ({
        x: point.x,
        y: point.y,
        curvature: point.curvature,
        velocity: point.velocity,
        theta: point.theta
      }))
    };
  },
  
  // Create path from poses
  createPathFromPoses(data: {
    poses: PoseModel[];
    params?: PathParamsModel;
  }): PathResponse {
    // Convert to JavaScript Poses
    const jsPoses = data.poses.map(poseModelToPose);
    
    // Generate path using our JavaScript implementation
    const bezierCurves = pathWithPoses(
      jsPoses,
      100, // v (default)
      10,  // a (default)
      data.params?.tangent_magnitude || 0.5
    );
    
    // Generate discretized points
    const discretizedPoints = generatePoints(bezierCurves);
    
    // Generate a unique ID for the path
    const pathId = generateUniqueId();
    
    // Store the path
    pathsStorage[pathId] = {
      bezierCurves,
      discretizedPoints
    };
    
    // Convert curves to the expected format
    const curves = bezierCurves.map((curve: QuinticBezierType) => {
      return {
        control_points: curve.getControlPoints().map(pointToPointModel)
      };
    });
    
    // Return the response in the same format as the API
    return {
      path_id: pathId,
      curves,
      discretized_points: discretizedPoints.map((point: PathPointType) => ({
        x: point.x,
        y: point.y,
        curvature: point.curvature,
        velocity: point.velocity,
        theta: point.theta
      }))
    };
  },
  
  // Create path from control points
  createPathFromControlPoints(data: {
    control_points_list: BezierCurveModel[];
  }): PathResponse {
    // Convert to JavaScript Points and create Quintic_Bezier objects
    const bezierCurves = data.control_points_list.map((curve) => {
      const points = controlPointsToPoints(curve.control_points);
      return new Quintic_Bezier(
        points[0], points[1], points[2], 
        points[3], points[4], points[5]
      );
    });
    
    // Generate discretized points
    // @ts-ignore - using the global function
    const discretizedPoints = generatePoints(bezierCurves);
    
    // Generate a unique ID for the path
    const pathId = generateUniqueId();
    
    // Store the path
    pathsStorage[pathId] = {
      bezierCurves,
      discretizedPoints
    };
    
    // Return the response in the same format as the API
    return {
      path_id: pathId,
      curves: data.control_points_list,
      discretized_points: discretizedPoints.map((point: PathPointType) => ({
        x: point.x,
        y: point.y,
        curvature: point.curvature,
        velocity: point.velocity,
        theta: point.theta
      }))
    };
  },
  
  // Get path
  getPath(pathId: string, includeDiscretized: boolean = false): PathResponse {
    const path = pathsStorage[pathId];
    
    if (!path) {
      throw new Error(`Path with ID ${pathId} not found`);
    }
    
    // Convert curves to the expected format
    const curves = path.bezierCurves.map((curve: any) => {
      return {
        control_points: curve.getControlPoints().map(pointToPointModel)
      };
    });
    
    // Return the response in the same format as the API
    return {
      path_id: pathId,
      curves,
      discretized_points: includeDiscretized ? path.discretizedPoints.map((point: any) => ({
        x: point.x,
        y: point.y,
        curvature: point.curvature,
        velocity: point.velocity,
        theta: point.theta
      })) : undefined
    };
  },
  
  // Delete path
  deletePath(pathId: string): void {
    delete pathsStorage[pathId];
  },
  
  // Generate trajectory
  generateTrajectory(pathId: string, params: TrajectoryParamsModel): TrajectoryResponse {
    const path = pathsStorage[pathId];
    
    if (!path) {
      throw new Error(`Path with ID ${pathId} not found`);
    }
    
    // Generate trajectory using our JavaScript implementation
    // @ts-ignore - using the global function
    const trajectoryPoints = calculateTrajectory(
      path.bezierCurves,
      params.initial_velocity,
      params.final_velocity,
      params.max_velocity,
      params.acceleration,
      params.deceleration,
      params.max_jerk,
      params.max_angular_velocity,
      params.use_trapezoidal
    );
    
    // Generate a unique ID for the trajectory
    const trajectoryId = generateUniqueId();
    
    // Store the trajectory
    trajectoriesStorage[trajectoryId] = {
      pathId,
      points: trajectoryPoints,
      totalTime: trajectoryPoints.length * 0.01 // Assuming dt = 0.01
    };
    
    // Return the response in the same format as the API
    return {
      trajectory_id: trajectoryId,
      path_id: pathId,
      points: trajectoryPoints.map((point: any) => ({
        x: point.x,
        y: point.y,
        curvature: point.curvature,
        velocity: point.velocity
      })),
      total_time: trajectoryPoints.length * 0.01
    };
  },
  
  // Get trajectory
  getTrajectory(trajectoryId: string): TrajectoryResponse {
    const trajectory = trajectoriesStorage[trajectoryId];
    
    if (!trajectory) {
      throw new Error(`Trajectory with ID ${trajectoryId} not found`);
    }
    
    // Return the response in the same format as the API
    return {
      trajectory_id: trajectoryId,
      path_id: trajectory.pathId,
      points: trajectory.points.map((point: any) => ({
        x: point.x,
        y: point.y,
        curvature: point.curvature,
        velocity: point.velocity
      })),
      total_time: trajectory.totalTime
    };
  },
  
  // Delete trajectory
  deleteTrajectory(trajectoryId: string): void {
    delete trajectoriesStorage[trajectoryId];
  }
};

export default bezierService;
