import rospy
from geometry_msgs.msg import PoseStamped
from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO, emit
import threading
from threading import Thread
import time

app = Flask(__name__)
socketio = SocketIO(app)

current_goal = None


def goal_callback(msg):
    global current_goal
    current_goal = msg
    # Extract x, y, z coordinates from PoseStamped message
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    print(f"New Goal Received: x={x}, y={y}, z={z}")
    
    # Forward the goal to the WebSocket
    socketio.emit('goal_update', {'x': x, 'y': y, 'z': z})

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

# Flask WebSocket event to get the latest goal
@socketio.on('get_current_goal')
def send_current_goal():
    global current_goal
    if current_goal:
        emit('goal_update', {
            'x': current_goal.pose.position.x,
            'y': current_goal.pose.position.y,
            'z': current_goal.pose.position.z
        })
    else:
        emit('goal_update', {'error': 'No goal received yet'})

def ros_thread():
    rospy.init_node('flask_goal_listener', anonymous=True)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
    rospy.spin()

@app.route('/map')
def map_view():
    return render_template('map.html')

@app.route('/location')
def get_location():
    """API to provide real-time robot location."""
    return jsonify(robot_location)

if __name__ == '__main__':
    app.run(debug=True)
