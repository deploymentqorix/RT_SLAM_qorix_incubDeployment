# File: backend/dev_scripts/mock_api.py
# Description: A standalone FastAPI server to provide mock data for frontend development.
# To run: python3 mock_api.py

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

# Create the FastAPI application instance
app = FastAPI()

# Add CORS Middleware. This is crucial to allow your React app (running on localhost:3000)
# to make requests to this server (running on localhost:8000).
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allows all origins
    allow_credentials=True,
    allow_methods=["*"],  # Allows all methods
    allow_headers=["*"],  # Allows all headers
)

@app.get("/api/v1/system/health")
def get_system_health():
    """
    This endpoint simulates the real /system/health endpoint.
    It returns a hardcoded, fake system health status.
    """
    return {
      "cpu_usage": 35.4,
      "memory": {"used": 2.1, "total": 8.0},
      "gpu_usage": 15.0,
      "slam_status": "INITIALIZING"
    }

@app.post("/api/v1/slam/reset")
def reset_slam():
    """
    This endpoint simulates receiving a reset command from the UI.
    """
    print("Received SLAM reset command!")
    return {"status": "ok", "message": "SLAM reset command received"}

# This block allows the script to be run directly from the command line
if __name__ == "__main__":
    print("Starting MOCK API Server on http://localhost:8000")
    # Run the server using uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
