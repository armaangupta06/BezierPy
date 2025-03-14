from mangum import Mangum
import sys
import os
import logging
import json

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Add the project root to the Python path so we can import our modules
root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(root_path)
logger.info(f"Added to path: {root_path}")
logger.info(f"Python path: {sys.path}")

try:
    # Import the FastAPI app
    from api.app import app
    from api.routes import path_routes, trajectory_routes
    logger.info("Successfully imported FastAPI app and routes")
    
    # Create a handler for AWS Lambda / Vercel
    # Configure Mangum to strip /api prefix from paths
    def strip_api_prefix(event):
        # Log the incoming event for debugging
        logger.info(f"Incoming event: {json.dumps(event)}")
        
        # Check if this is an API Gateway event
        if event.get('requestContext', {}).get('http'):
            path = event.get('rawPath', '')
            logger.info(f"Original path: {path}")
            
            # Strip /api prefix if present
            if path.startswith('/api'):
                path = path[4:] or '/'
                event['rawPath'] = path
                logger.info(f"Modified path: {path}")
                
                # Also update the path parameters if present
                if 'pathParameters' in event and event['pathParameters']:
                    # Update path parameters as needed
                    logger.info(f"Path parameters before: {event['pathParameters']}")
                    # Additional path parameter handling if needed
        
        return event
    
    handler = Mangum(app, lifespan="off", api_gateway_base_path="/api", event_hooks={"before": [strip_api_prefix]})
    logger.info("Mangum handler created successfully with API prefix stripping")
    
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
                "url": str(request.url)
            }
        )
    
    handler = Mangum(error_app, lifespan="off")
