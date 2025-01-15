from flask import Flask, render_template, jsonify
import threading
from threading import Thread
import time

app = Flask(__name__)

# Placeholder for robot's dynamic location
robot_location = {"x": 0, "y": 0}  # New position
 # Starting position
robot_path = [{"x": 0, "y": 0}]  # Path as a list of coordinates

def simulate_ros_updates():
    """Simulate ROS updates for robot's location and path."""
    global robot_location, robot_path
    x, y = 0, 0
    while True:
        x += 10
        y += 5
        robot_location = {"x": x, "y": y}
        robot_path.append({"x": x, "y": y})
        time.sleep(1)  # Update every second

threading.Thread(target=simulate_ros_updates, daemon=True).start()

def simulate_robot_movement():
    global robot_location
    while True:
        robot_location["x"] += 5
        robot_location["y"] += 3
        time.sleep(1)  # Update every second

Thread(target=simulate_robot_movement, daemon=True).start()

@app.route('/')
def home():
    return render_template('index.html')

@app.route('/map')
def map_view():
    return render_template('map.html')

@app.route('/location')
def get_location():
    """API to provide real-time robot location."""
    return jsonify(robot_location)

if __name__ == '__main__':
    app.run(debug=True)
