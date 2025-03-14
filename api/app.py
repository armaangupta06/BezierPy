"""
Main FastAPI application for the BezierPy API.

This module sets up the FastAPI application with all routes and middleware.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.openapi.utils import get_openapi

# Import routes
from api.routes.path_routes import router as path_router
from api.routes.trajectory_routes import router as trajectory_router

# Create FastAPI application
app = FastAPI(
    title="BezierPy API",
    description="API for Bezier curve path generation and trajectory calculation",
    version="1.0.0"
)

# Add CORS middleware to allow cross-origin requests
# This is important for the web-based frontend to communicate with the API
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify the allowed origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(path_router)
app.include_router(trajectory_router)

# Root endpoint
@app.get("/", tags=["Root"])
async def root():
    """Root endpoint that returns API information."""
    return {
        "name": "BezierPy API",
        "version": "1.0.0",
        "description": "API for Bezier curve path generation and trajectory calculation",
        "documentation": "/docs"
    }

# Custom OpenAPI schema to improve documentation
def custom_openapi():
    if app.openapi_schema:
        return app.openapi_schema
    
    openapi_schema = get_openapi(
        title="BezierPy API",
        version="1.0.0",
        description=(
            "API for Bezier curve path generation and trajectory calculation.\n\n"
            "## Features\n\n"
            "- Create paths from points, poses, or control points\n"
            "- Generate trajectories with kinematic constraints\n"
            "- Visualize paths and trajectories\n\n"
            "## Concepts\n\n"
            "- **Path**: A geometric representation defined by Quintic Bezier curves\n"
            "- **Trajectory**: A time-parameterized path with kinematic constraints"
        ),
        routes=app.routes,
    )
    
    app.openapi_schema = openapi_schema
    return app.openapi_schema

app.openapi = custom_openapi
