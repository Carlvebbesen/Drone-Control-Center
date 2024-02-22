from flask import Flask, request, jsonify
from flask_socketio import SocketIO, emit
import socket

app = Flask(__name__)
socketio = SocketIO(app)

# For ROS2 Nodes Communication
ros2_ports = {
    'manual_control': 5685,
}

ros2_sockets = {}

def connect_to_ros2_node(port):
    try:
        ros2_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ros2_socket.connect(('localhost', port))
        return ros2_socket
    except:
        print("could not connect")
        return None

@socketio.on('message_to_ros2')
def handle_message_to_ros2(socket_connection: socket.socket, message: str):
    socket_connection.sendall(message.encode())
    response = socket_connection.recv(1024)
    emit('from_ros2', {'data': response.decode()})

@socketio.on('connect')
def test_connect():
    emit('connect', {'data': 'Connected'})

@socketio.event
def message_node(data: dict):
    node = data["node"]
    message = data["message"]
    port = ros2_ports[node]
    try:
        socket_connection = connect_to_ros2_node(port)
        ros2_sockets[node] = socket_connection
        print(socket_connection)
        handle_message_to_ros2(ros2_sockets[node], message)
    except Exception as e:
        print(e)
        emit("error", {'data': f'no node named {data["node"]}'})

if __name__ == '__main__':
    socketio.run(app, debug=True)