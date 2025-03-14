from mangum import Mangum
import sys
import os
import logging

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
    logger.info("Successfully imported FastAPI app")
    
    # Create a handler for AWS Lambda / Vercel
    handler = Mangum(app, lifespan="off")
    logger.info("Mangum handler created successfully")
    
except Exception as e:
    logger.error(f"Error setting up API handler: {str(e)}")
    # Create a simple error handler in case of import failure
    from fastapi import FastAPI, Request
    from fastapi.responses import JSONResponse
    
    error_app = FastAPI()
    
    @error_app.get("/api/{path:path}")
    @error_app.post("/api/{path:path}")
    async def error_handler(request: Request, path: str):
        return JSONResponse(
            status_code=500,
            content={"error": f"API initialization error: {str(e)}", "path": path}
        )
    
    handler = Mangum(error_app)
