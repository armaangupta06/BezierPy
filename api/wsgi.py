"""
WSGI entry point for the BezierPy API.
This file is used by Render to start the application.
"""
from app import app

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app:app", host="0.0.0.0", port=8000)
