import asyncio
import json
import websockets
from flask import Flask, request, jsonify
import threading
import random
import time

app = Flask(__name__)

# --- CORS: allow simple cross-origin calls from control page ---
@app.after_request
def add_cors_headers(resp):
    resp.headers['Access-Control-Allow-Origin'] = '*'
    resp.headers['Access-Control-Allow-Methods'] = 'GET,POST,OPTIONS'
    resp.headers['Access-Control-Allow-Headers'] = 'Content-Type'
    return resp

# ---------------------------
# Globals
# ---------------------------
connected = set()
async_loop = None
collision_count = 0
goal_reached = False
current_obstacles = []  # Track obstacles server-side
latest_canvas_capture = None  # Store latest canvas capture
CANVAS_WIDTH = 650   # Updated canvas width
CANVAS_HEIGHT = 600  # Updated canvas height

def corner_to_coords(corner: str, margin=20):
    """Convert corner names to 2D coordinates"""
    c = corner.upper()
    x = CANVAS_WIDTH - margin if "E" in c else margin
    y = CANVAS_HEIGHT - margin if ("S" in c or "B" in c) else margin
    
    if c in ("NE", "EN", "TR"): x, y = (CANVAS_WIDTH - margin, margin)
    if c in ("NW", "WN", "TL"): x, y = (margin, margin)
    if c in ("SE", "ES", "BR"): x, y = (CANVAS_WIDTH - margin, CANVAS_HEIGHT - margin)
    if c in ("SW", "WS", "BL"): x, y = (margin, CANVAS_HEIGHT - margin)
    return {"x": x, "y": y}

def generate_random_obstacles(count=8):
    """Generate random obstacle positions"""
    obstacles = []
    for _ in range(count):
        x = random.randint(50, CANVAS_WIDTH - 50)
        y = random.randint(50, CANVAS_HEIGHT - 50)
        obstacles.append({"x": x, "y": y, "size": 25})  # Updated size to 25
    return obstacles

# ---------------------------
# WebSocket Handler
# ---------------------------
async def ws_handler(websocket, path=None):
    global collision_count, goal_reached, latest_canvas_capture
    print("Client connected via WebSocket")
    connected.add(websocket)
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                if isinstance(data, dict):
                    if data.get("type") == "collision" and data.get("collision"):
                        collision_count += 1
                        print(f"Collision detected! Total collisions: {collision_count}")
                    elif data.get("type") == "goal_reached":
                        goal_reached = True
                        print(f"Goal reached! Robot position: {data.get('robot_position')}")
                    elif data.get("type") == "canvas_captured":
                        latest_canvas_capture = {
                            'image_data': data.get('image_data'),
                            'timestamp': data.get('timestamp'),
                            'canvas_size': data.get('canvas_size'),
                            'robot_position': data.get('robot_position'),
                            'goal_position': data.get('goal_position'),
                            'obstacles_count': data.get('obstacles_count')
                        }
                        print(f"Canvas captured at {data.get('timestamp')}")
                    elif data.get("type") == "canvas_capture_error":
                        print(f"Canvas capture error: {data.get('error')}")
            except Exception as e:
                print(f"Error processing message: {e}")
            print("Received from simulator:", message[:100] + "..." if len(message) > 100 else message)
    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")
    finally:
        connected.remove(websocket)

def broadcast(msg: dict):
    if not connected:
        return False
    for ws in list(connected):
        try:
            asyncio.run_coroutine_threadsafe(ws.send(json.dumps(msg)), async_loop)
        except Exception as e:
            print(f"Error broadcasting message: {e}")
    return True

# ---------------------------
# Movement Endpoints
# ---------------------------
@app.route('/move', methods=['POST'])
def move():
    global goal_reached
    data = request.get_json()
    if not data or 'x' not in data or 'y' not in data:
        return jsonify({'error': 'Missing parameters. Please provide "x" and "y".'}), 400
    
    x, y = data['x'], data['y']
    goal_reached = False  # Reset goal status when robot starts moving
    msg = {"command": "move", "target": {"x": x, "y": y}}
    if not broadcast(msg):
        return jsonify({'error': 'No connected simulators.'}), 400
    return jsonify({'status': 'move command sent', 'command': msg})

@app.route('/move_rel', methods=['POST'])
def move_rel():
    global goal_reached
    data = request.get_json()
    if not data or 'angle' not in data or 'distance' not in data:
        return jsonify({'error': 'Missing parameters. Please provide "angle" and "distance".'}), 400
    
    goal_reached = False  # Reset goal status when robot starts moving
    msg = {"command": "move_relative", "angle": data['angle'], "distance": data['distance']}
    if not broadcast(msg):
        return jsonify({'error': 'No connected simulators.'}), 400
    return jsonify({'status': 'move relative command sent', 'command': msg})

@app.route('/stop', methods=['POST'])
def stop():
    msg = {"command": "stop"}
    if not broadcast(msg):
        return jsonify({'error': 'No connected simulators.'}), 400
    return jsonify({'status': 'stop command sent', 'command': msg})

# ---------------------------
# Goal Management
# ---------------------------
@app.route('/goal', methods=['POST'])
def set_goal():
    global goal_reached
    data = request.get_json() or {}
    if 'corner' in data:
        pos = corner_to_coords(str(data['corner']))
    elif 'x' in data and 'y' in data:
        pos = {"x": float(data['x']), "y": float(data['y'])}
    else:
        return jsonify({'error': 'Provide {"corner":"NE|NW|SE|SW"} OR {"x":..,"y":..}'}), 400

    goal_reached = False  # Reset goal status when setting new goal
    msg = {"command": "set_goal", "position": pos}
    if not broadcast(msg):
        return jsonify({'error': 'No connected simulators.'}), 400
    return jsonify({'status': 'goal set', 'goal': pos})

@app.route('/goal/status', methods=['GET'])
def get_goal_status():
    """GET endpoint to check if goal is reached"""
    return jsonify({
        'goal_reached': goal_reached,
        'status': 'reached' if goal_reached else 'not_reached'
    })

# ---------------------------
# Capture Endpoint (modified)
# ---------------------------
@app.route('/capture', methods=['GET'])
def capture_canvas():
    global latest_canvas_capture

    # Step 1: Trigger capture request
    if not broadcast({"command": "capture_canvas"}):
        return jsonify({'status': 'error', 'message': 'No connected simulators.'}), 400

    # Step 2: Wait (up to 3 seconds) for simulator to reply
    timeout = time.time() + 3
    while time.time() < timeout:
        if latest_canvas_capture is not None:
            return jsonify({
                'status': 'success',
                'image_data': latest_canvas_capture['image_data'],
                'timestamp': latest_canvas_capture['timestamp'],
                'canvas_size': latest_canvas_capture['canvas_size'],
                'robot_position': latest_canvas_capture['robot_position'],
                'goal_position': latest_canvas_capture['goal_position'],
                'obstacles_count': latest_canvas_capture['obstacles_count'],
                'message': 'Canvas image captured successfully'
            })
        time.sleep(0.1)  # avoid CPU busy loop

    return jsonify({'status': 'error', 'message': 'Timed out waiting for canvas capture'}), 504

# ---------------------------
# Obstacles Management
# ---------------------------
@app.route('/obstacles/random', methods=['POST'])
def generate_obstacles():
    data = request.get_json() or {}
    count = data.get('count', 8)
    obstacles = generate_random_obstacles(count)
    
    msg = {"command": "set_obstacles", "obstacles": obstacles}
    if not broadcast(msg):
        return jsonify({'error': 'No connected simulators.'}), 400
    return jsonify({'status': 'random obstacles generated', 'obstacles': obstacles})

@app.route('/obstacles/positions', methods=['POST'])
def set_obstacle_positions():
    data = request.get_json() or {}
    obstacles = data.get('obstacles')
    if not isinstance(obstacles, list) or not obstacles:
        return jsonify({'error': 'Provide "obstacles" as a non-empty list.'}), 400

    # Normalize obstacles
    norm = []
    for obs in obstacles:
        if not isinstance(obs, dict) or 'x' not in obs or 'y' not in obs:
            return jsonify({'error': 'Each obstacle needs "x" and "y".'}), 400
        norm.append({
            "x": float(obs['x']), 
            "y": float(obs['y']), 
            "size": float(obs.get('size', 20))
        })

    msg = {"command": "set_obstacles", "obstacles": norm}
    if not broadcast(msg):
        return jsonify({'error': 'No connected simulators.'}), 400
    return jsonify({'status': 'obstacles updated', 'count': len(norm)})

# ---------------------------
# System Management
# ---------------------------
@app.route('/collisions', methods=['GET'])
def get_collisions():
    return jsonify({'count': collision_count})

@app.route('/reset', methods=['POST'])
def reset():
    global collision_count, goal_reached, current_obstacles
    collision_count = 0
    goal_reached = False
    current_obstacles = []  # Clear obstacles on reset
    if not broadcast({"command": "reset"}):
        return jsonify({'status': 'reset done (no simulators connected)', 'collisions': collision_count, 'goal_reached': goal_reached})
    return jsonify({'status': 'reset broadcast', 'collisions': collision_count, 'goal_reached': goal_reached})

@app.route('/status', methods=['GET'])
def get_status():
    return jsonify({
        'connected_simulators': len(connected),
        'collision_count': collision_count,
        'goal_reached': goal_reached,
        'canvas_width': CANVAS_WIDTH,
        'canvas_height': CANVAS_HEIGHT
    })

# ---------------------------
# Flask Thread
# ---------------------------
def start_flask():
    print("Starting Flask server on http://localhost:5001")
    app.run(host='0.0.0.0', port=5001, debug=False, use_reloader=False)

# ---------------------------
# Main Async for WebSocket
# ---------------------------
async def main():
    global async_loop
    async_loop = asyncio.get_running_loop()
    ws_server = await websockets.serve(ws_handler, "localhost", 8080)
    print("WebSocket server started on ws://localhost:8080")
    print("Flask server starting on http://localhost:5000")
    await ws_server.wait_closed()

# ---------------------------
# Entry Point
# ---------------------------
if __name__ == "__main__":
    print("🚀 Starting 2D Robot Server...")
    print("Flask server will run on: http://localhost:5001")
    print("WebSocket server will run on: ws://localhost:8080")
    print("Press Ctrl+C to stop")
    
    flask_thread = threading.Thread(target=start_flask, daemon=True)
    flask_thread.start()
    
    # Give Flask a moment to start
    time.sleep(1)
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n🛑 Server stopped by user")
