import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from utils.speechtotext import socketio,record_and_transcribe
from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO, emit
import threading
import time

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="http://127.0.0.1:5000")


# Global variables
current_goal = None
robot_location = {"x": 0, "y": 0}  # Starting position
robot_path = [{"x": 0, "y": 0}]    # Path as a list of coordinates

# ROS Node and Subscriber
class GoalListener(Node):
    def __init__(self):
        super().__init__('flask_goal_listener')

                # Publisher to publish goal messages
        self.publisher = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)

        self.subscription = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_callback,
            10
        )

    def goal_callback(self, msg):
        global current_goal
        current_goal = msg
        # Extract x, y, z coordinates from PoseStamped message
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        # Log the received coordinates to confirm   
        self.get_logger().info(f"New Goal Received: x={x}, y={y}, z={z}")
        
        # Forward the goal to the WebSocket
        socketio.emit('goal_update', {'x': x, 'y': y, 'z': z})

    def send_goal(self, x, y):
        # Create a PoseStamped message with the destination
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'  # Set the reference frame
        goal_msg.pose.position.x = float(x)  # Ensure x and y are floats
        goal_msg.pose.position.y = float(y)
        goal_msg.pose.position.z = 0.0  # Assuming 2D goal, z is 0
        self.publisher.publish(goal_msg)
        print(f"Sending goal to robot: x={x}, y={y}")
    

# WebSocket event handler to receive the destination coordinates
# WebSocket event handler to receive the destination coordinates
@socketio.on('send_destination')
def handle_destination(data):
    x, y = data['x'], data['y']
    
    # Log the coordinates received from the frontend
    print(f"Destination received: x={x}, y={y}")
    
    # Send coordinates to ROS
    node.send_goal(x, y)  # This line sends the goal to ROS via GoalListener

    # Acknowledge receipt and inform frontend
    emit('destination_received', {'status': 'success', 'message': f"Coordinates received: x={x}, y={y}"})


# Simulate robot's dynamic location and path updates
def simulate_robot_updates():
    global robot_location, robot_path
    x, y = 0, 0
    while True:
        x += 10
        y += 5
        robot_location = {"x": x, "y": y}
        robot_path.append({"x": x, "y": y})
        time.sleep(1)

# Flask routes and WebSocket events
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

#ROS THREAD 
def ros_thread():
    rclpy.init()
    global node
    node = GoalListener()  # Initialize the ROS node here
    node.send_goal(10.0, 20.0)  # Initial goal (example)
    rclpy.spin(node)  # Keep spinning to listen to topics
    node.destroy_node()  # Cleanup ROS node after it stops
    rclpy.shutdown()  # Shut down the ROS client


@socketio.on('start_recording')
def start_recording():
    print("Recording started...")
    # Call speech recognition function here and send result
    # Start recording in a separate thread (calls the function from speechtotext.py)
    threading.Thread(target=record_and_transcribe).start()
    socketio.emit("transcription_result", {"transcription": transcription})


# Start the application
if __name__ == '__main__':
    # Start ROS thread
    threading.Thread(target=ros_thread, daemon=True).start()
    
    # Start robot simulation thread
    threading.Thread(target=simulate_robot_updates, daemon=True).start()
    
    # Run Flask with WebSocket support
    socketio.run(app, debug=True)
