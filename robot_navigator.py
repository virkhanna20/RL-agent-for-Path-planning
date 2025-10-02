import requests
import time
import json
import numpy as np
from collections import deque
import heapq
import math

class RobotNavigator:
    def __init__(self, server_url="http://localhost:5001"):
        self.server_url = server_url
        self.grid_size = 10  # Size of each grid cell in pixels
        self.grid_width = 65  # 650 pixels / 10
        self.grid_height = 60  # 600 pixels / 10
        self.obstacles = set()
        self.start = (32, 30)  # (320, 300) in grid coordinates
        self.goal = (55, 8)   # (550, 80) in grid coordinates
        self.robot_size = 18   # Robot size in pixels
        self.obstacle_size = 25  # Obstacle size in pixels
        
    def get_obstacles(self):
        """Get all obstacles from the server and convert to grid coordinates"""
        try:
            # First try to get obstacles from the server
            response = requests.get(f"{self.server_url}/obstacles")
            if response.status_code == 200:
                obstacles_data = response.json()
                self.obstacles = set()
                
                # The server doesn't track obstacles, so we'll use the initial obstacles
                initial_obstacles = [
                    {"x": 150, "y": 120, "size": 25},
                    {"x": 450, "y": 180, "size": 25},
                    {"x": 220, "y": 300, "size": 25},
                    {"x": 380, "y": 380, "size": 25},
                    {"x": 100, "y": 450, "size": 25},
                    {"x": 500, "y": 100, "size": 25},
                    {"x": 280, "y": 220, "size": 25},
                    {"x": 420, "y": 320, "size": 25}
                ]
                
                for obs in initial_obstacles:
                    # Convert pixel coordinates to grid coordinates
                    x_grid = obs["x"] // self.grid_size
                    y_grid = obs["y"] // self.grid_size
                    
                    # Add obstacle and surrounding cells for safety margin
                    margin = max(1, (obs["size"] + self.robot_size) // (2 * self.grid_size))
                    for dx in range(-margin, margin + 1):
                        for dy in range(-margin, margin + 1):
                            nx, ny = x_grid + dx, y_grid + dy
                            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                                self.obstacles.add((nx, ny))
                
                print(f"Retrieved {len(self.obstacles)} obstacle cells")
                return True
            else:
                print(f"Failed to get obstacles: {response.status_code}")
                return False
        except Exception as e:
            print(f"Error getting obstacles: {e}")
            return False
    
    def get_robot_position(self):
        """Get current robot position from server"""
        try:
            response = requests.get(f"{self.server_url}/status")
            if response.status_code == 200:
                status_data = response.json()
                # The status endpoint doesn't return robot position directly
                # We'll use the default starting position
                return self.start
            else:
                print(f"Failed to get robot position: {response.status_code}")
                return self.start
        except Exception as e:
            print(f"Error getting robot position: {e}")
            return self.start
    
    def move_robot(self, x, y):
        """Move robot to specified pixel coordinates"""
        try:
            data = {"x": x, "y": y}
            response = requests.post(f"{self.server_url}/move", json=data)
            if response.status_code == 200:
                return True
            else:
                print(f"Failed to move robot: {response.status_code}")
                return False
        except Exception as e:
            print(f"Error moving robot: {e}")
            return False
    
    def check_goal_reached(self):
        """Check if the goal has been reached"""
        try:
            response = requests.get(f"{self.server_url}/goal/status")
            if response.status_code == 200:
                goal_data = response.json()
                return goal_data.get("goal_reached", False)
            else:
                print(f"Failed to check goal status: {response.status_code}")
                return False
        except Exception as e:
            print(f"Error checking goal status: {e}")
            return False
    
    def check_collisions(self):
        """Check if robot has collided with obstacles"""
        try:
            response = requests.get(f"{self.server_url}/collisions")
            if response.status_code == 200:
                collision_data = response.json()
                return collision_data.get("count", 0) > 0
            else:
                print(f"Failed to check collisions: {response.status_code}")
                return False
        except Exception as e:
            print(f"Error checking collisions: {e}")
            return False
    
    def heuristic(self, a, b):
        """Calculate heuristic distance between two points"""
        # Using Euclidean distance for more accurate pathfinding
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def get_neighbors(self, pos):
        """Get valid neighboring positions"""
        x, y = pos
        neighbors = []
        
        # Eight-directional movement for more flexibility
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
            nx, ny = x + dx, y + dy
            
            # Check if within bounds
            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                # Check if not an obstacle
                if (nx, ny) not in self.obstacles:
                    neighbors.append((nx, ny))
        
        return neighbors
    
    def is_line_clear(self, start, end):
        """Check if a straight line between two points is clear of obstacles"""
        # Bresenham's line algorithm to check all cells between start and end
        x0, y0 = start
        x1, y1 = end
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            # Check current cell
            if (x0, y0) in self.obstacles:
                return False
            
            # Check if we've reached the end
            if x0 == x1 and y0 == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
                
        return True
    
    def find_path(self, start, goal):
        """Find path from start to goal using A* algorithm"""
        # Priority queue: (f_score, position)
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            
            for neighbor in self.get_neighbors(current):
                # Calculate cost with diagonal movement penalty
                move_cost = 1.414 if abs(neighbor[0] - current[0]) + abs(neighbor[1] - current[1]) == 2 else 1
                tentative_g_score = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # No path found
        return None
    
    def optimize_path(self, path):
        """Optimize path by removing unnecessary waypoints"""
        if not path or len(path) <= 2:
            return path
        
        optimized = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self.is_line_clear(path[i], path[j]):
                    i = j
                    optimized.append(path[i])
                    break
                j -= 1
            if j == i + 1:
                optimized.append(path[i+1])
                i += 1
                
        return optimized
    
    def navigate_to_goal(self):
        """Navigate the robot to the goal position"""
        print("Starting optimized robot navigation...")
        
        # Get current obstacles
        if not self.get_obstacles():
            print("Failed to get obstacles. Exiting.")
            return False
        
        # Get current robot position
        current_pos = self.get_robot_position()
        print(f"Robot starting at grid position: {current_pos}")
        
        # Find path to goal
        path = self.find_path(current_pos, self.goal)
        
        if not path:
            print("No path found to goal!")
            return False
        
        print(f"Path found with {len(path)} steps")
        
        # Optimize path by removing unnecessary waypoints
        optimized_path = self.optimize_path(path)
        print(f"Optimized path has {len(optimized_path)} steps")
        
        # Follow the optimized path
        for i, step in enumerate(optimized_path):
            # Convert grid coordinates back to pixel coordinates
            x_pixel = step[0] * self.grid_size
            y_pixel = step[1] * self.grid_size
            
            print(f"Step {i+1}/{len(optimized_path)}: Moving to pixel position ({x_pixel}, {y_pixel})")
            
            # Move robot
            if not self.move_robot(x_pixel, y_pixel):
                print(f"Failed to move to step {i+1}")
                return False
            
            # Reduced waiting time for faster movement
            time.sleep(0.05)  # Reduced from 0.5 to 0.05 seconds
            
            # Check for collisions
            if self.check_collisions():
                print("Collision detected! Recalculating path...")
                current_pos = self.get_robot_position()
                path = self.find_path(current_pos, self.goal)
                
                if not path:
                    print("No alternative path found!")
                    return False
                
                # Optimize the new path
                optimized_path = self.optimize_path(path)
                print(f"New optimized path has {len(optimized_path)} steps")
                # Reset loop to follow new path
                i = 0
                continue
            
            # Check if goal reached
            if self.check_goal_reached():
                print("Goal reached successfully!")
                return True
        
        # Final check if goal reached
        if self.check_goal_reached():
            print("Goal reached successfully!")
            return True
        else:
            print("Failed to reach goal")
            return False

if __name__ == "__main__":
    navigator = RobotNavigator()
    navigator.navigate_to_goal()