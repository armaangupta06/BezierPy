from mangum import Mangum
import sys
import os

# Add the project root to the Python path so we can import our modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

# Import the FastAPI app
from api.app import app

# Create a handler for AWS Lambda / Vercel
handler = Mangum(app)
