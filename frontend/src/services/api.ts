import axios from 'axios';

// Determine if we're running on the client or server side
const isClient = typeof window !== 'undefined';

// Determine the base URL based on environment
let API_URL = '';

// In production (Vercel deployment)
if (process.env.NODE_ENV === 'production') {
  // Use the standalone backend URL in production
  API_URL = 'https://bezier-py-backend.vercel.app';
} else {
  // In development, use the local API server
  API_URL = process.env.NEXT_PUBLIC_API_URL || 'http://127.0.0.1:8000';
}

console.log('API URL:', API_URL);

// No need for path modification since we're using a separate backend
const getPath = (path: string): string => {
  return path;
};

// Create axios instance with base URL
const api = axios.create({
  baseURL: API_URL,
  headers: {
    'Content-Type': 'application/json',
  },
});

// Types for API requests and responses
export interface PointModel {
  x: number;
  y: number;
}

export interface PoseModel {
  x: number;
  y: number;
  heading: number;
}

export interface PathParamsModel {
  tangent_magnitude: number;
}

export interface TrajectoryParamsModel {
  initial_velocity: number;
  final_velocity: number;
  max_velocity: number;
  acceleration: number;
  deceleration: number;
  max_jerk: number;
  max_angular_velocity: number;
  use_trapezoidal: boolean;
}

export interface CreatePathFromPointsRequest {
  points: PointModel[];
  initial_heading: number;
  final_heading?: number;
  params?: PathParamsModel;
}

export interface CreatePathFromPosesRequest {
  poses: PoseModel[];
  params?: PathParamsModel;
}

export interface ControlPointModel {
  x: number;
  y: number;
}

export interface BezierCurveModel {
  control_points: ControlPointModel[];
}

export interface CreatePathFromControlPointsRequest {
  control_points_list: BezierCurveModel[];
}

export interface PathResponse {
  path_id: string;
  curves: any[];
  discretized_points?: any[];
}

export interface TrajectoryResponse {
  trajectory_id: string;
  path_id: string;
  points: any[];
  total_time: number;
}

// API functions
export const apiService = {
  // Path endpoints
  createPathFromPoints: async (data: CreatePathFromPointsRequest): Promise<PathResponse> => {
    console.log('Creating path from points with URL:', getPath('/paths/from-points'));
    const response = await api.post(getPath('/paths/from-points'), data);
    return response.data;
  },

  createPathFromPoses: async (data: CreatePathFromPosesRequest): Promise<PathResponse> => {
    console.log('Creating path from poses with URL:', getPath('/paths/from-poses'));
    const response = await api.post(getPath('/paths/from-poses'), data);
    return response.data;
  },
  
  createPathFromControlPoints: async (data: CreatePathFromControlPointsRequest): Promise<PathResponse> => {
    console.log('Creating path from control points with URL:', getPath('/paths/from-control-points'));
    const response = await api.post(getPath('/paths/from-control-points'), data);
    return response.data;
  },

  getPath: async (pathId: string, includeDiscretized: boolean = false): Promise<PathResponse> => {
    const url = getPath(`/paths/${pathId}?include_discretized=${includeDiscretized}`);
    console.log('Getting path with URL:', url);
    const response = await api.get(url);
    return response.data;
  },

  deletePath: async (pathId: string): Promise<void> => {
    await api.delete(getPath(`/paths/${pathId}`));
  },

  // Trajectory endpoints
  generateTrajectory: async (pathId: string, params: TrajectoryParamsModel): Promise<TrajectoryResponse> => {
    const url = getPath(`/trajectories/from-path/${pathId}`);
    console.log('Generating trajectory with URL:', url);
    const response = await api.post(url, params);
    return response.data;
  },

  getTrajectory: async (trajectoryId: string): Promise<TrajectoryResponse> => {
    const response = await api.get(getPath(`/trajectories/${trajectoryId}`));
    return response.data;
  },

  deleteTrajectory: async (trajectoryId: string): Promise<void> => {
    await api.delete(getPath(`/trajectories/${trajectoryId}`));
  },
};

export default apiService;
