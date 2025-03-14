import os
import sys

# Add the current directory to the path so Python can find the local modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Now import the app
from app import app

# This file can be used directly with uvicorn
if __name__ == "__main__":
    import uvicorn
    uvicorn.run("server:app", host="0.0.0.0", port=int(os.environ.get("PORT", 8000)))
