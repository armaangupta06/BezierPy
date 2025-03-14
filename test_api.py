"""
Simple test script for the BezierPy API.

This script tests the main API endpoints for path and trajectory generation.
"""
import requests
import json
import time
import matplotlib.pyplot as plt
import numpy as np

# API base URL
BASE_URL = "http://127.0.0.1:8000"

def test_create_path_from_points():
    """Test creating a path from points."""
    print("\n1. Testing path creation from points...")
    
    # Request data
    data = {
        "points": [
            {"x": 0, "y": 0},
            {"x": 50, "y": 50},
            {"x": 100, "y": 0}
        ],
        "initial_heading": 0,
        "final_heading": 0,
        "params": {
            "tangent_magnitude": 0.5
        }
    }
    
    # Make request
    response = requests.post(f"{BASE_URL}/paths/from-points", json=data)
    
    # Print results
    print(f"Status code: {response.status_code}")
    if response.status_code == 200:
        result = response.json()
        path_id = result.get("path_id")
        print(f"Path created with ID: {path_id}")
        print(f"Number of curves: {len(result.get('curves', []))}")
        
        # Return path_id for further testing
        return path_id
    else:
        print(f"Error: {response.text}")
        return None

def test_create_path_from_poses():
    """Test creating a path from poses."""
    print("\n2. Testing path creation from poses...")
    
    # Request data
    data = {
        "poses": [
            {"x": 0, "y": 0, "heading": 0},
            {"x": 50, "y": 50, "heading": 45},
            {"x": 100, "y": 0, "heading": 0}
        ],
        "params": {
            "tangent_magnitude": 0.5
        }
    }
    
    # Make request
    response = requests.post(f"{BASE_URL}/paths/from-poses", json=data)
    
    # Print results
    print(f"Status code: {response.status_code}")
    if response.status_code == 200:
        result = response.json()
        path_id = result.get("path_id")
        print(f"Path created with ID: {path_id}")
        print(f"Number of curves: {len(result.get('curves', []))}")
        
        # Return path_id for further testing
        return path_id
    else:
        print(f"Error: {response.text}")
        return None

def test_get_path(path_id, include_discretized=False):
    """Test retrieving a path by ID."""
    print(f"\n3. Testing path retrieval for ID: {path_id}...")
    
    # Make request
    response = requests.get(
        f"{BASE_URL}/paths/{path_id}",
        params={"include_discretized": include_discretized}
    )
    
    # Print results
    print(f"Status code: {response.status_code}")
    if response.status_code == 200:
        result = response.json()
        print(f"Path ID: {result.get('path_id')}")
        print(f"Number of curves: {len(result.get('curves', []))}")
        if include_discretized:
            points = result.get('discretized_points', [])
            print(f"Number of discretized points: {len(points)}")
    else:
        print(f"Error: {response.text}")

def test_generate_trajectory(path_id):
    """Test generating a trajectory from a path."""
    print(f"\n4. Testing trajectory generation for path ID: {path_id}...")
    
    # Request data
    data = {
        "initial_velocity": 0.0,
        "final_velocity": 0.0,
        "max_velocity": 40.0,
        "acceleration": 20.0,
        "deceleration": -20.0,
        "max_jerk": 0.0,
        "max_angular_velocity": 10.0,
        "use_trapezoidal": True
    }
    
    # Make request
    response = requests.post(
        f"{BASE_URL}/trajectories/from-path/{path_id}",
        json=data
    )
    
    # Print results
    print(f"Status code: {response.status_code}")
    if response.status_code == 200:
        result = response.json()
        trajectory_id = result.get("trajectory_id")
        print(f"Trajectory created with ID: {trajectory_id}")
        print(f"Number of points: {len(result.get('points', []))}")
        print(f"Total time: {result.get('total_time')} seconds")
        
        # Return trajectory_id for further testing
        return trajectory_id
    else:
        print(f"Error: {response.text}")
        return None

def test_get_trajectory(trajectory_id):
    """Test retrieving a trajectory by ID."""
    print(f"\n5. Testing trajectory retrieval for ID: {trajectory_id}...")
    
    # Make request
    response = requests.get(f"{BASE_URL}/trajectories/{trajectory_id}")
    
    # Print results
    print(f"Status code: {response.status_code}")
    if response.status_code == 200:
        result = response.json()
        print(f"Trajectory ID: {result.get('trajectory_id')}")
        print(f"Path ID: {result.get('path_id')}")
        print(f"Number of points: {len(result.get('points', []))}")
        print(f"Total time: {result.get('total_time')} seconds")
    else:
        print(f"Error: {response.text}")

def visualize_trajectory(trajectory_id):
    """Visualize a trajectory."""
    print(f"\n6. Visualizing trajectory for ID: {trajectory_id}...")
    
    # Make request
    response = requests.get(f"{BASE_URL}/trajectories/{trajectory_id}")
    
    # Check if request was successful
    if response.status_code == 200:
        result = response.json()
        points = result.get('points', [])
        
        # Extract x, y coordinates and velocities
        x = [p['x'] for p in points]
        y = [p['y'] for p in points]
        velocities = [p['velocity'] for p in points]
        times = [p['time'] for p in points]
        
        # Create figure with two subplots
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # Plot path with velocity color mapping
        scatter = ax1.scatter(x, y, c=velocities, cmap='viridis', s=5)
        ax1.set_title('Trajectory with Velocity Coloring')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.grid(True)
        fig.colorbar(scatter, ax=ax1, label='Velocity')
        
        # Plot velocity vs time
        ax2.plot(times, velocities)
        ax2.set_title('Velocity vs Time')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity')
        ax2.grid(True)
        
        # Save the figure
        plt.tight_layout()
        plt.savefig('trajectory_visualization.png')
        print("Visualization saved as 'trajectory_visualization.png'")
    else:
        print(f"Error: {response.text}")

def run_all_tests():
    """Run all API tests."""
    print("Starting BezierPy API tests...")
    
    # Test path creation from points
    path_id = test_create_path_from_points()
    if path_id:
        # Test path retrieval
        test_get_path(path_id, include_discretized=True)
        
        # Test trajectory generation
        trajectory_id = test_generate_trajectory(path_id)
        if trajectory_id:
            # Test trajectory retrieval
            test_get_trajectory(trajectory_id)
            
            # Visualize trajectory
            visualize_trajectory(trajectory_id)
    
    # Test path creation from poses
    test_create_path_from_poses()
    
    print("\nAPI tests completed!")

if __name__ == "__main__":
    run_all_tests()
