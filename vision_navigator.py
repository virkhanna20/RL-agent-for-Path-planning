import requests
import time
import cv2
import numpy as np
import base64
import io
from PIL import Image
from collections import deque
import heapq
import math

class VisionBasedNavigator:
    def __init__(self, server_url="http://localhost:5001"):
        self.server_url = server_url
        self.canvas_width = 650
        self.canvas_height = 600
        self.robot_radius = 18
        self.goal_size = 15
        self.obstacle_size = 25
        self.safety_margin = 35  # Reduced safety margin
        self.step_size = 8  # Larger step size for faster movement
        self.sleep_time = 0.08  # Faster movement
        
    def capture_canvas(self):
        """Capture the current state of the canvas"""
        try:
            response = requests.get(f"{self.server_url}/capture")
            if response.status_code == 200:
                data = response.json()
                if data['status'] == 'success':
                    image_data_base64 = data['image_data']
                    if image_data_base64.startswith('data:image/png;base64,'):
                        image_data_base64 = image_data_base64.split(',')[1]
                    image_data = base64.b64decode(image_data_base64)
                    image = Image.open(io.BytesIO(image_data))
                    return cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
                else:
                    print(f"Capture failed: {data['message']}")
                    return None
            else:
                print(f"Failed to capture canvas: {response.status_code}")
                return None
        except Exception as e:
            print(f"Error capturing canvas: {e}")
            return None
    
    def detect_objects(self, image):
        """Detect robot, goal, and obstacles from the image"""
        if image is None:
            return None, None, []
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define color ranges
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])
        
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])
        
        # Create masks
        robot_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        robot_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        robot_mask = cv2.bitwise_or(robot_mask1, robot_mask2)
        
        goal_mask = cv2.inRange(hsv, lower_green, upper_green)
        obstacle_mask = cv2.inRange(hsv, lower_black, upper_black)
        
        # Apply morphological operations
        kernel = np.ones((3,3), np.uint8)
        robot_mask = cv2.morphologyEx(robot_mask, cv2.MORPH_CLOSE, kernel)
        goal_mask = cv2.morphologyEx(goal_mask, cv2.MORPH_CLOSE, kernel)
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        robot_contours, _ = cv2.findContours(robot_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        goal_contours, _ = cv2.findContours(goal_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        obstacle_contours, _ = cv2.findContours(obstacle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Detect robot
        robot_pos = None
        if robot_contours:
            largest_contour = max(robot_contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                robot_pos = (cx, cy)
        
        # Detect goal
        goal_pos = None
        if goal_contours:
            largest_contour = max(goal_contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                goal_pos = (cx, cy)
        
        # Detect obstacles
        obstacles = []
        for contour in obstacle_contours:
            area = cv2.contourArea(contour)
            if area > 100:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    obstacles.append((cx, cy))
        
        return robot_pos, goal_pos, obstacles
    
    def heuristic(self, a, b):
        """Calculate heuristic distance between two points"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def is_position_safe(self, pos, obstacles):
        """Check if a position is safe from obstacles"""
        for obs_x, obs_y in obstacles:
            dist = math.sqrt((pos[0] - obs_x)**2 + (pos[1] - obs_y)**2)
            if dist < self.safety_margin:
                return False
        return True
    
    def find_path_with_adaptive_margin(self, start, goal, obstacles):
        """Find path with adaptive safety margin"""
        # Try with current safety margin
        path = self.find_path(start, goal, obstacles, self.safety_margin)
        if path:
            return path
        
        # If no path found, try with reduced safety margin
        print(f"No path found with margin {self.safety_margin}, trying with {self.safety_margin - 10}")
        path = self.find_path(start, goal, obstacles, self.safety_margin - 10)
        if path:
            return path
        
        # Try with even smaller margin
        print(f"No path found with margin {self.safety_margin - 10}, trying with {self.safety_margin - 20}")
        path = self.find_path(start, goal, obstacles, self.safety_margin - 20)
        if path:
            return path
        
        # Last resort: direct path with minimal margin
        print(f"No path found with margin {self.safety_margin - 20}, trying direct approach")
        return self.find_direct_path(start, goal, obstacles)
    
    def find_path(self, start, goal, obstacles, margin):
        """Find path from start to goal using A* algorithm with specified margin"""
        # Priority queue: (f_score, position)
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            # Check if we've reached the goal
            if self.heuristic(current, goal) < self.robot_radius * 2:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            
            # Get neighbors with specified margin
            neighbors = self.get_neighbors_with_margin(current, obstacles, margin)
            
            for neighbor in neighbors:
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # No path found
        return None
    
    def get_neighbors_with_margin(self, pos, obstacles, margin):
        """Get valid neighboring positions with specified margin"""
        x, y = pos
        neighbors = []
        
        # Four-directional movement with larger step size
        directions = [
            (0, self.step_size),   # Down
            (self.step_size, 0),   # Right
            (0, -self.step_size),  # Up
            (-self.step_size, 0)   # Left
        ]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            
            # Check if within bounds
            if 0 <= nx < self.canvas_width and 0 <= ny < self.canvas_height:
                # Check if not colliding with obstacles
                safe = True
                for obs_x, obs_y in obstacles:
                    dist = math.sqrt((nx - obs_x)**2 + (ny - obs_y)**2)
                    if dist < margin:
                        safe = False
                        break
                
                if safe:
                    neighbors.append((nx, ny))
        
        return neighbors
    
    def find_direct_path(self, start, goal, obstacles):
        """Find a direct path by moving straight toward goal"""
        path = [start]
        current = start
        
        while self.heuristic(current, goal) > self.robot_radius:
            # Calculate direction to goal
            dx = goal[0] - current[0]
            dy = goal[1] - current[1]
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance == 0:
                break
            
            # Normalize and scale to step size
            nx = current[0] + int((dx / distance) * self.step_size)
            ny = current[1] + int((dy / distance) * self.step_size)
            
            # Check bounds
            nx = max(0, min(self.canvas_width - 1, nx))
            ny = max(0, min(self.canvas_height - 1, ny))
            
            # Check if position is safe with minimal margin
            safe = True
            for obs_x, obs_y in obstacles:
                dist = math.sqrt((nx - obs_x)**2 + (ny - obs_y)**2)
                if dist < 20:  # Minimal safety margin
                    safe = False
                    break
            
            if not safe:
                # Try to go around the obstacle
                # Try moving horizontally first
                if abs(dx) > abs(dy):
                    if dx > 0:
                        nx = current[0] + self.step_size
                    else:
                        nx = current[0] - self.step_size
                else:
                    if dy > 0:
                        ny = current[1] + self.step_size
                    else:
                        ny = current[1] - self.step_size
                
                # Check bounds again
                nx = max(0, min(self.canvas_width - 1, nx))
                ny = max(0, min(self.canvas_height - 1, ny))
            
            current = (nx, ny)
            path.append(current)
            
            # Prevent infinite loops
            if len(path) > 100:  # Reduced from 200
                break
        
        return path
    
    def move_robot(self, x, y):
        """Move robot to specified coordinates"""
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
    
    def navigate_to_goal(self):
        """Main navigation function using vision-based path planning"""
        print("Starting fast adaptive vision-based navigation...")
        
        # Capture the canvas and detect objects
        image = self.capture_canvas()
        if image is None:
            print("Failed to capture canvas. Exiting.")
            return False
        
        robot_pos, goal_pos, obstacles = self.detect_objects(image)
        
        if robot_pos is None:
            print("Could not detect robot position. Exiting.")
            return False
        
        if goal_pos is None:
            print("Could not detect goal position. Exiting.")
            return False
        
        print(f"Robot detected at: {robot_pos}")
        print(f"Goal detected at: {goal_pos}")
        print(f"Detected {len(obstacles)} obstacles")
        
        # Find path with adaptive margin
        path = self.find_path_with_adaptive_margin(robot_pos, goal_pos, obstacles)
        
        if not path:
            print("No path found to goal!")
            return False
        
        print(f"Path found with {len(path)} steps")
        
        # Follow the path
        for i, step in enumerate(path):
            print(f"Step {i+1}/{len(path)}: Moving to {step}")
            
            # Move robot
            if not self.move_robot(step[0], step[1]):
                print(f"Failed to move to step {i+1}")
                return False
            
            # Wait for movement to complete (faster)
            time.sleep(self.sleep_time)
            
            # Check for collisions every 5 steps to reduce API calls
            if i % 5 == 0 and self.check_collisions():
                print("Collision detected! Recalculating path...")
                
                # Recapture canvas and recalculate path
                image = self.capture_canvas()
                if image is None:
                    print("Failed to recapture canvas. Exiting.")
                    return False
                
                robot_pos, goal_pos, obstacles = self.detect_objects(image)
                
                if robot_pos is None or goal_pos is None:
                    print("Could not detect robot or goal after collision. Exiting.")
                    return False
                
                path = self.find_path_with_adaptive_margin(robot_pos, goal_pos, obstacles)
                
                if not path:
                    print("No alternative path found!")
                    return False
                
                print(f"New path has {len(path)} steps")
                # Reset loop to follow new path
                i = -1  # Will be incremented to 0
                continue
            
            # Check if goal reached every 5 steps to reduce API calls
            if i % 5 == 0 and self.check_goal_reached():
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
    navigator = VisionBasedNavigator()
    navigator.navigate_to_goal()
