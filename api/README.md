# BezierPy Backend API

This is the backend API for the BezierPy application, which provides path generation and trajectory calculation services.

## Deployment to Render

### Prerequisites

- A [Render](https://render.com) account
- Your code pushed to a GitHub repository

### Deployment Steps

1. Log in to your Render account
2. Click on "New" and select "Web Service"
3. Connect your GitHub repository
4. Configure the service:
   - **Name**: bezierpy-api (or your preferred name)
   - **Root Directory**: /api
   - **Runtime**: Python 3
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn api.app:app --host 0.0.0.0 --port $PORT`
   
5. Add environment variables:
   - `PYTHON_VERSION`: 3.9.0
   - `ALLOW_ORIGINS`: https://bezierpy.vercel.app,http://localhost:3000 (update with your actual Vercel frontend URL)

6. Click "Create Web Service"

### Testing Your Deployment

Once deployed, you can test your API by visiting:
- `https://your-render-service.onrender.com/` - Should show API information
- `https://your-render-service.onrender.com/docs` - Interactive API documentation

## Connecting Frontend to Backend

After deploying your backend, update your frontend environment variables:

1. In your Vercel deployment settings, add:
   - `NEXT_PUBLIC_API_URL`: https://your-render-service.onrender.com

2. For local development, create a `.env.local` file in your frontend directory with:
   ```
   NEXT_PUBLIC_API_URL=https://your-render-service.onrender.com
   ```

## Local Development

To run the API locally:

```bash
uvicorn api.app:app --reload
```

The API will be available at http://localhost:8000
