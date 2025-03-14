from mangum import Mangum
import sys
import os
import logging
import json

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Add the project root to the Python path so we can import our modules
root_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(root_path)
logger.info(f"Added to path: {root_path}")
logger.info(f"Python path: {sys.path}")

try:
    # Import the FastAPI app
    from app import app
    logger.info("Successfully imported FastAPI app")
    
    # Create a handler for AWS Lambda / Vercel
    def log_event(event):
        # Log the incoming event for debugging
        logger.info(f"Incoming event: {json.dumps(event)}")
        return event
    
    handler = Mangum(app, lifespan="off", event_hooks={"before": [log_event]})
    logger.info("Mangum handler created successfully")
    
except Exception as e:
    logger.error(f"Error setting up API handler: {str(e)}")
    # Create a simple error handler in case of import failure
    from fastapi import FastAPI, Request
    from fastapi.responses import JSONResponse
    from fastapi.middleware.cors import CORSMiddleware
    
    error_app = FastAPI()
    
    # Add CORS middleware to error app
    error_app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )
    
    @error_app.get("/{path:path}")
    @error_app.post("/{path:path}")
    async def error_handler(request: Request, path: str):
        # Log the request for debugging
        body = await request.body()
        logger.error(f"Error app received request: {request.method} {request.url.path}")
        logger.error(f"Request headers: {request.headers}")
        logger.error(f"Request body: {body}")
        
        return JSONResponse(
            status_code=500,
            content={
                "error": f"API initialization error: {str(e)}", 
                "path": path,
                "method": request.method,
                "url": str(request.url),
                "python_path": sys.path
            }
        )
    
    handler = Mangum(error_app, lifespan="off")
