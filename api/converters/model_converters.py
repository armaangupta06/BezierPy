"""
Converter functions to translate between Pydantic API models and domain classes.

These functions handle the conversion between the API layer (Pydantic models)
and the domain layer (existing BezierPy classes).
"""
import math
import uuid
from typing import List, Dict, Any, Optional, Tuple

# Import Pydantic models
from api.models.bezier_models import (
    PointModel, PoseModel, BezierCurveModel, ControlPointModel,
    PathPointModel, TrajectoryPointModel, PathModel, TrajectoryModel
)

# Import domain classes
from Point import Point
from Pose import Pose
from Quintic_Bezier import Quintic_Bezier
from Path_Point import Path_Point

# Point conversions
def pydantic_point_to_domain(point_model: PointModel) -> Point:
    """Convert Pydantic PointModel to domain Point class"""
    return Point(point_model.x, point_model.y)

def domain_point_to_pydantic(point: Point) -> PointModel:
    """Convert domain Point to Pydantic PointModel"""
    return PointModel(x=point.x, y=point.y)

# Pose conversions
def pydantic_pose_to_domain(pose_model: PoseModel) -> Pose:
    """Convert Pydantic PoseModel to domain Pose class
    
    Note: The Pose class expects heading in degrees and converts to radians internally.
    """
    return Pose(pose_model.x, pose_model.y, pose_model.heading)

def domain_pose_to_pydantic(pose: Pose) -> PoseModel:
    """Convert domain Pose to Pydantic PoseModel
    
    Note: The Pose class stores heading in radians, so we convert back to degrees.
    """
    return PoseModel(
        x=pose.x, 
        y=pose.y, 
        heading=pose.heading * 180 / math.pi  # Convert radians to degrees
    )

# Bezier curve conversions
def pydantic_bezier_to_domain(bezier_model: BezierCurveModel) -> Quintic_Bezier:
    """Convert Pydantic BezierCurveModel to domain Quintic_Bezier class"""
    # Extract the 6 control points
    if len(bezier_model.control_points) != 6:
        raise ValueError("Quintic Bezier curve requires exactly 6 control points")
    
    # Convert each control point to domain Point
    points = [
        pydantic_point_to_domain(ControlPointModel(**cp.dict())) 
        if not isinstance(cp, ControlPointModel) else pydantic_point_to_domain(cp)
        for cp in bezier_model.control_points
    ]
    
    # Create Quintic_Bezier with the 6 points
    return Quintic_Bezier(points[0], points[1], points[2], points[3], points[4], points[5])

def domain_bezier_to_pydantic(bezier: Quintic_Bezier) -> BezierCurveModel:
    """Convert domain Quintic_Bezier to Pydantic BezierCurveModel"""
    # Get the 6 control points
    p0, p1, p2, p3, p4, p5 = bezier.get_control_points()
    
    # Convert each Point to ControlPointModel
    control_points = [
        ControlPointModel(x=p.x, y=p.y) 
        for p in [p0, p1, p2, p3, p4, p5]
    ]
    
    return BezierCurveModel(control_points=control_points)

# Path point conversions
def domain_path_point_to_pydantic(path_point: Path_Point) -> PathPointModel:
    """Convert domain Path_Point to Pydantic PathPointModel"""
    return PathPointModel(
        x=path_point.x,
        y=path_point.y,
        curvature=path_point.curvature,
        theta=path_point.theta,
        velocity=path_point.velocity
    )

def pydantic_path_point_to_domain(path_point_model: PathPointModel) -> Path_Point:
    """Convert Pydantic PathPointModel to domain Path_Point
    
    Note: This creates a Point object to pass to the Path_Point constructor.
    """
    point = Point(path_point_model.x, path_point_model.y)
    return Path_Point(
        point, 
        curvature=path_point_model.curvature,
        theta=path_point_model.theta,
        velocity=path_point_model.velocity
    )

# Path conversions
def domain_bezier_list_to_path_model(
    bezier_curves: List[Quintic_Bezier], 
    path_id: Optional[str] = None,
    include_discretized: bool = False,
    discretized_points: Optional[List[Path_Point]] = None
) -> PathModel:
    """Convert a list of domain Quintic_Bezier objects to a Pydantic PathModel"""
    # Generate path_id if not provided
    if path_id is None:
        path_id = str(uuid.uuid4())
    
    # Convert each Bezier curve
    curves = [domain_bezier_to_pydantic(curve) for curve in bezier_curves]
    
    # Optionally include discretized points
    path_points = None
    if include_discretized:
        if discretized_points is None:
            # If discretized points weren't provided, we would generate them here
            # This would typically call the generate_points function from curve.py
            # For now, we'll leave this as None
            pass
        else:
            path_points = [
                domain_path_point_to_pydantic(point) 
                for point in discretized_points
            ]
    
    return PathModel(
        path_id=path_id,
        curves=curves,
        discretized_points=path_points
    )

# Helper function to generate a unique ID
def generate_unique_id() -> str:
    """Generate a unique ID for paths and trajectories."""
    return str(uuid.uuid4())
