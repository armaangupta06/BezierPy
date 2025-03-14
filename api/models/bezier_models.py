"""
Pydantic models for the BezierPy API.
These models define the data structures for API requests and responses.
"""
from pydantic import BaseModel, Field, validator
from typing import List, Optional, Dict, Any, Union
import math

class PointModel(BaseModel):
    """API model for Point data"""
    x: float
    y: float
    
    class Config:
        schema_extra = {
            "example": {
                "x": 10.5,
                "y": 20.3
            }
        }

class PoseModel(BaseModel):
    """API model for Pose data (point with heading)"""
    x: float
    y: float
    heading: float  # In degrees
    
    @validator('heading')
    def validate_heading(cls, v):
        """Ensure heading is in the range [0, 360)"""
        return v % 360
    
    class Config:
        schema_extra = {
            "example": {
                "x": 10.5,
                "y": 20.3,
                "heading": 45.0
            }
        }

class ControlPointModel(BaseModel):
    """API model for a single control point"""
    x: float
    y: float
    
    class Config:
        schema_extra = {
            "example": {
                "x": 10.5,
                "y": 20.3
            }
        }

class BezierCurveModel(BaseModel):
    """API model for a Quintic Bezier curve segment"""
    control_points: List[ControlPointModel] = Field(
        ..., 
        min_items=6, 
        max_items=6,
        description="The 6 control points defining the curve (P0, P1, P2, P3, P4, P5)"
    )
    
    class Config:
        schema_extra = {
            "example": {
                "control_points": [
                    {"x": 0, "y": 0},
                    {"x": 10, "y": 0},
                    {"x": 20, "y": 0},
                    {"x": 30, "y": 10},
                    {"x": 40, "y": 20},
                    {"x": 50, "y": 30}
                ]
            }
        }

class PathParamsModel(BaseModel):
    """Parameters for path generation"""
    tangent_magnitude: float = Field(
        0.5, 
        gt=0, 
        description="Magnitude of tangent vectors relative to segment length"
    )
    
    class Config:
        schema_extra = {
            "example": {
                "tangent_magnitude": 0.8
            }
        }

class TrajectoryParamsModel(BaseModel):
    """Parameters for trajectory generation with kinematic constraints"""
    initial_velocity: float = Field(
        0.0,
        ge=0,
        description="Initial velocity at the start of the trajectory"
    )
    final_velocity: float = Field(
        0.0,
        ge=0,
        description="Final velocity at the end of the trajectory"
    )
    max_velocity: float = Field(
        100.0, 
        gt=0, 
        description="Maximum linear velocity"
    )
    acceleration: float = Field(
        20.0, 
        gt=0, 
        description="Acceleration value (positive)"
    )
    deceleration: float = Field(
        -20.0, 
        lt=0, 
        description="Deceleration value (negative)"
    )
    max_jerk: float = Field(
        0.0, 
        ge=0, 
        description="Maximum jerk (rate of change of acceleration)"
    )
    max_angular_velocity: float = Field(
        2.0, 
        gt=0, 
        description="Maximum angular velocity"
    )
    use_trapezoidal: bool = Field(
        True, 
        description="Whether to use trapezoidal motion profile (True) or S-curve (False)"
    )
    
    class Config:
        schema_extra = {
            "example": {
                "initial_velocity": 0.0,
                "final_velocity": 0.0,
                "max_velocity": 100.0,
                "acceleration": 20.0,
                "deceleration": -20.0,
                "max_jerk": 0.0,
                "max_angular_velocity": 2.0,
                "use_trapezoidal": True
            }
        }

class PathPointModel(BaseModel):
    """API model for a point along a path with geometric properties"""
    x: float
    y: float
    curvature: float
    theta: float = 0  # Direction angle in degrees
    velocity: float = 0  # Magnitude of velocity vector
    
    class Config:
        schema_extra = {
            "example": {
                "x": 10.5,
                "y": 20.3,
                "curvature": 0.05,
                "theta": 45.0,
                "velocity": 0.0
            }
        }

class TrajectoryPointModel(BaseModel):
    """API model for a point along a trajectory with time and dynamic properties"""
    x: float
    y: float
    curvature: float
    theta: float = 0  # Direction angle in degrees
    velocity: float = 0  # Velocity at this point
    time: float = 0  # Time parameter for trajectory
    
    class Config:
        schema_extra = {
            "example": {
                "x": 10.5,
                "y": 20.3,
                "curvature": 0.05,
                "theta": 45.0,
                "velocity": 80.0,
                "time": 0.5
            }
        }

class PathModel(BaseModel):
    """API model for a complete path consisting of multiple Bezier curve segments"""
    path_id: str
    curves: List[BezierCurveModel]
    discretized_points: Optional[List[PathPointModel]] = None
    
    class Config:
        schema_extra = {
            "example": {
                "path_id": "550e8400-e29b-41d4-a716-446655440000",
                "curves": [
                    {
                        "control_points": [
                            {"x": 0, "y": 0},
                            {"x": 10, "y": 0},
                            {"x": 20, "y": 0},
                            {"x": 30, "y": 10},
                            {"x": 40, "y": 20},
                            {"x": 50, "y": 30}
                        ]
                    }
                ]
            }
        }

class TrajectoryModel(BaseModel):
    """API model for a complete trajectory with time-parameterized points"""
    trajectory_id: str
    path_id: str
    points: List[TrajectoryPointModel]
    total_time: float
    
    class Config:
        schema_extra = {
            "example": {
                "trajectory_id": "550e8400-e29b-41d4-a716-446655440000",
                "path_id": "550e8400-e29b-41d4-a716-446655440000",
                "points": [
                    {
                        "x": 0.0,
                        "y": 0.0,
                        "curvature": 0.0,
                        "theta": 0.0,
                        "velocity": 0.0,
                        "time": 0.0
                    },
                    {
                        "x": 10.5,
                        "y": 20.3,
                        "curvature": 0.05,
                        "theta": 45.0,
                        "velocity": 80.0,
                        "time": 0.5
                    }
                ],
                "total_time": 2.5
            }
        }

# Request models
class CreatePathFromPointsRequest(BaseModel):
    """Request model for creating a path from points"""
    points: List[PointModel] = Field(
        ..., 
        min_items=2,
        description="List of points to create a path through"
    )
    initial_heading: float = Field(
        ...,
        description="Initial heading in degrees"
    )
    final_heading: Optional[float] = Field(
        None,
        description="Final heading in degrees (optional)"
    )
    params: Optional[PathParamsModel] = None

class CreatePathFromPosesRequest(BaseModel):
    """Request model for creating a path from poses"""
    poses: List[PoseModel] = Field(
        ..., 
        min_items=2,
        description="List of poses (points with headings) to create a path through"
    )
    params: Optional[PathParamsModel] = None

class CreatePathFromControlPointsRequest(BaseModel):
    """Request model for creating a path directly from control points"""
    control_points_list: List[BezierCurveModel] = Field(
        ...,
        min_items=1,
        description="List of Bezier curves defined by their control points"
    )
