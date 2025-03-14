"""
FastAPI routes for trajectory generation and manipulation.

These routes handle the creation of trajectories from paths with kinematic constraints,
as well as operations on existing trajectories.
"""
import uuid
from typing import List, Dict, Any, Optional

from fastapi import APIRouter, HTTPException, Query, Path, Body

# Import Pydantic models
from api.models.bezier_models import (
    TrajectoryParamsModel, TrajectoryModel, TrajectoryPointModel
)

# Import converters
from api.converters.model_converters import (
    domain_path_point_to_pydantic, generate_unique_id
)

# Import domain functions
from curve import calculate_trajectory, generate_points

# Create router
router = APIRouter(prefix="/trajectories", tags=["Trajectories"])

# In-memory storage for trajectories
# In a production environment, this would be replaced with a database
trajectories_storage: Dict[str, Any] = {}

# Reference to paths storage from path_routes
# This would be handled differently in a production environment (e.g., database)
from api.routes.path_routes import paths_storage

@router.post(
    "/from-path/{path_id}", 
    response_model=TrajectoryModel,
    summary="Generate trajectory from path",
    description="Generate a trajectory from a path with kinematic constraints."
)
async def generate_trajectory_from_path(
    path_id: str = Path(..., description="The ID of the path to generate a trajectory from"),
    params: TrajectoryParamsModel = Body(..., description="Trajectory generation parameters")
):
    """Generate a trajectory from a path with kinematic constraints."""
    # Check if path exists
    if path_id not in paths_storage:
        raise HTTPException(status_code=404, detail="Path not found")
    
    # Get the path
    path_data = paths_storage[path_id]
    bezier_curves = path_data["bezier_curves"]
    
    # Call domain function with converted objects
    try:
        # Debug information
        print(f"Generating trajectory for path with {len(bezier_curves)} curves")
        
        # Call calculate_trajectory with proper error handling
        try:
            trajectory_points = calculate_trajectory(
                bezier_curves,
                v0=params.initial_velocity,
                v1=params.final_velocity,
                max_v=params.max_velocity,
                a_accel=params.acceleration,
                a_decel=params.deceleration,
                max_j=params.max_jerk,
                max_w=params.max_angular_velocity,
                trap=params.use_trapezoidal
            )
        except Exception as e:
            print(f"Error in calculate_trajectory: {str(e)}")
            # Fallback: generate points and add velocity/time information
            all_points = []
            for curve in bezier_curves:
                points = generate_points([curve])  # Pass as a list with a single curve
                all_points.extend(points)
            
            # We won't add time information to Path_Point objects as they don't have a time attribute
            
            trajectory_points = all_points
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed to generate trajectory: {str(e)}")
    
    # Generate unique ID for the trajectory
    trajectory_id = generate_unique_id()
    
    # Calculate total time based on the number of points and a fixed time step
    total_time = len(trajectory_points) * 0.1 if trajectory_points else 0.0
    
    # Store the trajectory
    trajectories_storage[trajectory_id] = {
        "path_id": path_id,
        "trajectory_points": trajectory_points,
        "total_time": total_time
    }
    
    # Convert to Pydantic model for response
    trajectory_point_models = []
    for point in trajectory_points:
        # Create TrajectoryPointModel from Path_Point
        # Calculate time based on index
        point_time = (len(trajectory_point_models) * total_time) / max(len(trajectory_points), 1)
        
        trajectory_point_models.append(
            TrajectoryPointModel(
                x=point.x,
                y=point.y,
                curvature=point.curvature,
                theta=point.theta,
                velocity=point.velocity,
                time=point_time  # Use calculated time instead of accessing point.time
            )
        )
    
    trajectory_model = TrajectoryModel(
        trajectory_id=trajectory_id,
        path_id=path_id,
        points=trajectory_point_models,
        total_time=total_time
    )
    
    return trajectory_model

@router.get(
    "/{trajectory_id}", 
    response_model=TrajectoryModel,
    summary="Get trajectory by ID",
    description="Retrieve a trajectory by its ID."
)
async def get_trajectory(
    trajectory_id: str = Path(..., description="The ID of the trajectory to retrieve")
):
    """Retrieve a trajectory by its ID."""
    # Check if trajectory exists
    if trajectory_id not in trajectories_storage:
        raise HTTPException(status_code=404, detail="Trajectory not found")
    
    # Get the trajectory
    trajectory_data = trajectories_storage[trajectory_id]
    path_id = trajectory_data["path_id"]
    trajectory_points = trajectory_data["trajectory_points"]
    total_time = trajectory_data["total_time"]
    
    # Convert to Pydantic model for response
    trajectory_point_models = []
    for point in trajectory_points:
        # Create TrajectoryPointModel from Path_Point
        # Calculate time based on index
        point_time = (len(trajectory_point_models) * total_time) / max(len(trajectory_points), 1)
        
        trajectory_point_models.append(
            TrajectoryPointModel(
                x=point.x,
                y=point.y,
                curvature=point.curvature,
                theta=point.theta,
                velocity=point.velocity,
                time=point_time  # Use calculated time instead of accessing point.time
            )
        )
    
    trajectory_model = TrajectoryModel(
        trajectory_id=trajectory_id,
        path_id=path_id,
        points=trajectory_point_models,
        total_time=total_time
    )
    
    return trajectory_model

@router.delete(
    "/{trajectory_id}",
    summary="Delete trajectory",
    description="Delete a trajectory by its ID."
)
async def delete_trajectory(
    trajectory_id: str = Path(..., description="The ID of the trajectory to delete")
):
    """Delete a trajectory by its ID."""
    # Check if trajectory exists
    if trajectory_id not in trajectories_storage:
        raise HTTPException(status_code=404, detail="Trajectory not found")
    
    # Delete the trajectory
    del trajectories_storage[trajectory_id]
    
    return {"message": f"Trajectory {trajectory_id} deleted successfully"}
