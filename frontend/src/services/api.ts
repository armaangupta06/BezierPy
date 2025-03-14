import axios from 'axios';

// API base URL - should match the backend URL
const API_URL = 'http://127.0.0.1:8000';

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
    const response = await api.post('/paths/from-points', data);
    return response.data;
  },

  createPathFromPoses: async (data: CreatePathFromPosesRequest): Promise<PathResponse> => {
    const response = await api.post('/paths/from-poses', data);
    return response.data;
  },
  
  createPathFromControlPoints: async (data: CreatePathFromControlPointsRequest): Promise<PathResponse> => {
    const response = await api.post('/paths/from-control-points', data);
    return response.data;
  },

  getPath: async (pathId: string, includeDiscretized: boolean = false): Promise<PathResponse> => {
    const response = await api.get(`/paths/${pathId}?include_discretized=${includeDiscretized}`);
    return response.data;
  },

  deletePath: async (pathId: string): Promise<void> => {
    await api.delete(`/paths/${pathId}`);
  },

  // Trajectory endpoints
  generateTrajectory: async (pathId: string, params: TrajectoryParamsModel): Promise<TrajectoryResponse> => {
    const response = await api.post(`/trajectories/from-path/${pathId}`, params);
    return response.data;
  },

  getTrajectory: async (trajectoryId: string): Promise<TrajectoryResponse> => {
    const response = await api.get(`/trajectories/${trajectoryId}`);
    return response.data;
  },

  deleteTrajectory: async (trajectoryId: string): Promise<void> => {
    await api.delete(`/trajectories/${trajectoryId}`);
  },
};

export default apiService;
