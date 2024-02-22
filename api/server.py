from flask import Flask, request
from flask_socketio import SocketIO
import socket
import threading
import asyncio

app = Flask(__name__)
sio = SocketIO(app,cors_allowed_origins="*")

# For ROS2 Nodes Communication
ros2_ports = {
    'manual_control': 5685,
    'video_feed': 5690
}

# This dictionary will store the socket connections
ros2_sockets = {}

def get_or_create_socket(port):
    if port in ros2_sockets:
        # Reuse the existing socket
        return ros2_sockets[port]
    else:
        # Create a new socket connection and store it
        new_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        new_socket.connect(('localhost', port))
        ros2_sockets[port] = new_socket
        return new_socket

def start_asyncio_task(target, *args):
    def run():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        coro = target(*args)
        loop.run_until_complete(coro)
        loop.close()
    t = threading.Thread(target=run)
    t.start()

async def send_message(port: int, message: str):
    try:
        # Reuse an existing socket or create a new one
        ros2_socket = get_or_create_socket(port)
        ros2_socket.sendall(message.encode())
        response = ros2_socket.recv(1024).decode()
        print(f"Reponse: {response}")
        # Use the saved `sid` to emit the response back to the correct client
        sio.emit("from_ros2", {'data': response})
    except Exception as e:
        print(f'Error sending message to node at port {port}: {e}')
        # Consider removing or resetting the socket in ros2_sockets if it's no longer valid

# Not sure if this works yet
async def recieve_message(port: int):
    try:
        # Reuse an existing socket or create a new one
        ros2_socket = get_or_create_socket(port)
        ros2_socket.bind("localhost", port)
        ros2_socket.listen(1)
        print(f"Listening for messages on port {port}...")
        response = ros2_socket.recv(1024)
        sio.emit("from_ros2", {'data': response})
    except Exception as e:
        print(f'Error sending message to node at port {port}: {e}')
        # Consider removing or resetting the socket in ros2_sockets if it's no longer valid

@sio.on('test')
def test(message):
    print(message)
    sio.emit('test', 'Test Emit')
    return message


@sio.on_error()
def error_handler(e):
    print("Error",e)
    pass

@sio.event
def connect():
    print('WebApp connected:', request.sid)

@sio.event
def disconnect():
    print('WebApp disconnected:', request.sid)

@sio.on('manual_control')
def handle_manual_control(data):
    print('Manual control event received:', data)
    port = ros2_ports['manual_control']
    start_asyncio_task(send_message, port, data)
    return data + " Done"

# Not sure if this works yet
@sio.on('video_feed')
def handle_video_feed(data):
    print('Video feed event received:', data)
    port = ros2_ports['video_feed']
    start_asyncio_task(recieve_message, port, data["message"])

if __name__ == '__main__':
    sio.run(app, debug=True)


# from flask import Flask, request, jsonify
# from flask_socketio import SocketIO, emit
# import socket
# import threading
# import asyncio
# import socketio

# app = Flask(__name__)
# sio = SocketIO(app, cors_allowed_origins="*")

# async_mode = None
# node_sio = socketio.AsyncClient()


# # For ROS2 Nodes Communication
# ros2_ports = {
#     'manual_control': 5685,
#     'video_feed': 5690
# }

# ros2_sockets = {}


# def start_asyncio_task(target, *args):
#     def run():
#         # Create a new event loop for the thread
#         loop = asyncio.new_event_loop()
#         asyncio.set_event_loop(loop)
        
#         # Schedule the coroutine to be run on the new event loop
#         coro = target(*args)
#         loop.run_until_complete(coro)
        
#         # Close the loop at the end of the task
#         loop.close()

#     # Create and start a new thread for running the asyncio task
#     t = threading.Thread(target=run)
#     t.start()

# async def connect_to_node(port: int, message: str):
#     try:
#         ros2_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         ros2_socket.connect(('localhost', port))
#         ros2_socket.sendall(message.encode())
#         response = ros2_socket.recv(1024)
#         emit("from_ros2", response)
#         ros2_socket.close()
#     except Exception as e:
#         print(f'Connection to node at {port} failed: {e}')

# @sio.event
# def connect():
#     print('WebApp connected:', request.sid)

# @sio.event
# def disconnect():
#     print('WebApp disconnected:', request.sid)

# @sio.on('manual_control')
# def handle_manual_control(data):
#     print('Manual control event received:', data)
#     # Here, connect to the drone control node
#     port = ros2_ports['manual_control']
#     start_asyncio_task(connect_to_node, port, data["message"])

# @sio.on('video_feed')
# def handle_video_feed(data):
#     print('Video feed event received:', data)
#     # Here, connect to the video feed node
#     port = ros2_ports['video_feed']
#     start_asyncio_task(connect_to_node, port, data["message"])

# if __name__ == '__main__':
#     sio.run(app, debug=True)