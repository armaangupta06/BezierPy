"""
Script to run the BezierPy API server using Uvicorn.

This script serves as the entry point for starting the FastAPI application.
"""
import uvicorn
import argparse

def main():
    """Run the BezierPy API server."""
    parser = argparse.ArgumentParser(description="Run the BezierPy API server")
    parser.add_argument(
        "--host", 
        type=str, 
        default="127.0.0.1", 
        help="Host to run the server on"
    )
    parser.add_argument(
        "--port", 
        type=int, 
        default=8000, 
        help="Port to run the server on"
    )
    parser.add_argument(
        "--reload", 
        action="store_true", 
        help="Enable auto-reload for development"
    )
    args = parser.parse_args()
    
    print(f"Starting BezierPy API server at http://{args.host}:{args.port}")
    print("API documentation available at http://{args.host}:{args.port}/docs")
    
    uvicorn.run(
        "api.app:app",
        host=args.host,
        port=args.port,
        reload=args.reload
    )

if __name__ == "__main__":
    main()
