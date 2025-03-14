"""
FastAPI routes for path creation and manipulation.

These routes handle the creation of paths from points, poses, and control points,
as well as operations on existing paths.
"""
import uuid
from typing import List, Dict, Any, Optional

from fastapi import APIRouter, HTTPException, Query, Path, Body

# Import Pydantic models
from api.models.bezier_models import (
    PointModel, PoseModel, BezierCurveModel, PathParamsModel,
    PathModel, PathPointModel, CreatePathFromPointsRequest,
    CreatePathFromPosesRequest, CreatePathFromControlPointsRequest
)

# Import converters
from api.converters.model_converters import (
    pydantic_point_to_domain, pydantic_pose_to_domain,
    pydantic_bezier_to_domain, domain_bezier_list_to_path_model,
    domain_path_point_to_pydantic, generate_unique_id
)

# Import domain functions
from curve import path_with_points, path_with_poses, generate_points

# Create router
router = APIRouter(prefix="/paths", tags=["Paths"])

# In-memory storage for paths
# In a production environment, this would be replaced with a database
paths_storage: Dict[str, Any] = {}

@router.post(
    "/from-points", 
    response_model=PathModel,
    summary="Create a path from points",
    description="Generate a path through the specified points with given initial and optional final heading."
)
async def create_path_from_points(request: CreatePathFromPointsRequest):
    """Generate a path from a list of points with initial and final heading."""
    # Extract parameters
    points = request.points
    initial_heading = request.initial_heading
    final_heading = request.final_heading
    params = request.params or PathParamsModel()
    
    # Convert Pydantic models to domain classes
    domain_points = [pydantic_point_to_domain(p) for p in points]
    
    # Call domain function with converted objects
    try:
        bezier_curves = path_with_points(
            *domain_points, 
            initial_heading=initial_heading,
            final_heading=final_heading,
            tangent_magnitude=params.tangent_magnitude
        )
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed to create path: {str(e)}")
    
    # Generate unique ID for the path
    path_id = generate_unique_id()
    
    # Store the path
    paths_storage[path_id] = {
        "bezier_curves": bezier_curves,
        "discretized_points": None  # Will be generated on demand
    }
    
    # Convert to Pydantic model for response
    path_model = domain_bezier_list_to_path_model(bezier_curves, path_id)
    
    return path_model

@router.post(
    "/from-poses", 
    response_model=PathModel,
    summary="Create a path from poses",
    description="Generate a path through the specified poses (points with headings)."
)
async def create_path_from_poses(request: CreatePathFromPosesRequest):
    """Generate a path from a list of poses (points with headings)."""
    # Extract parameters
    poses = request.poses
    params = request.params or PathParamsModel()
    
    # Convert Pydantic models to domain classes
    domain_poses = [pydantic_pose_to_domain(p) for p in poses]
    
    # Call domain function with converted objects
    try:
        bezier_curves = path_with_poses(
            *domain_poses, 
            tangent_magnitude=params.tangent_magnitude
        )
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed to create path: {str(e)}")
    
    # Generate unique ID for the path
    path_id = generate_unique_id()
    
    # Store the path
    paths_storage[path_id] = {
        "bezier_curves": bezier_curves,
        "discretized_points": None  # Will be generated on demand
    }
    
    # Convert to Pydantic model for response
    path_model = domain_bezier_list_to_path_model(bezier_curves, path_id)
    
    return path_model

@router.post(
    "/from-control-points", 
    response_model=PathModel,
    summary="Create a path from control points",
    description="Generate a path directly from Bezier curve control points."
)
async def create_path_from_control_points(request: CreatePathFromControlPointsRequest):
    """Generate a path directly from Bezier curve control points."""
    # Extract parameters
    control_points_list = request.control_points_list
    
    # Convert Pydantic models to domain classes
    bezier_curves = []
    try:
        for cp_model in control_points_list:
            bezier_curve = pydantic_bezier_to_domain(cp_model)
            bezier_curves.append(bezier_curve)
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed to create path: {str(e)}")
    
    # Generate unique ID for the path
    path_id = generate_unique_id()
    
    # Store the path
    paths_storage[path_id] = {
        "bezier_curves": bezier_curves,
        "discretized_points": None  # Will be generated on demand
    }
    
    # Convert to Pydantic model for response
    path_model = domain_bezier_list_to_path_model(bezier_curves, path_id)
    
    return path_model

@router.get(
    "/{path_id}", 
    response_model=PathModel,
    summary="Get path by ID",
    description="Retrieve a path by its ID."
)
async def get_path(
    path_id: str = Path(..., description="The ID of the path to retrieve"),
    include_discretized: bool = Query(False, description="Whether to include discretized points")
):
    """Retrieve a path by its ID."""
    # Check if path exists
    if path_id not in paths_storage:
        raise HTTPException(status_code=404, detail="Path not found")
    
    # Get the path
    path_data = paths_storage[path_id]
    bezier_curves = path_data["bezier_curves"]
    
    # Optionally generate discretized points
    discretized_points = None
    if include_discretized:
        if path_data["discretized_points"] is None:
            # Generate discretized points
            try:
                all_points = []
                for curve in bezier_curves:
                    # Generate points for each curve - we need to pass a list containing the curve
                    # The generate_points function expects a list of Quintic_Bezier objects
                    try:
                        # Pass the curve as a single-element list
                        points = generate_points([curve])
                        all_points.extend(points)
                    except Exception as e:
                        print(f"Error generating points for curve: {e}")
                        
                path_data["discretized_points"] = all_points
            except Exception as e:
                raise HTTPException(
                    status_code=500, 
                    detail=f"Failed to generate discretized points: {str(e)}"
                )
        
        discretized_points = path_data["discretized_points"]
    
    # Convert to Pydantic model for response
    path_model = domain_bezier_list_to_path_model(
        bezier_curves, 
        path_id,
        include_discretized=include_discretized,
        discretized_points=discretized_points
    )
    
    return path_model

@router.delete(
    "/{path_id}",
    summary="Delete path",
    description="Delete a path by its ID."
)
async def delete_path(path_id: str = Path(..., description="The ID of the path to delete")):
    """Delete a path by its ID."""
    # Check if path exists
    if path_id not in paths_storage:
        raise HTTPException(status_code=404, detail="Path not found")
    
    # Delete the path
    del paths_storage[path_id]
    
    return {"message": f"Path {path_id} deleted successfully"}
