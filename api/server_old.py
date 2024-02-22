from flask import Flask, request
from flask_socketio import SocketIO
import socket
import threading
import asyncio
import struct
import pickle

app = Flask(__name__)
sio = SocketIO(app,cors_allowed_origins="*")

# For ROS2 Nodes Communication
ros2_ports = {
    'manual_control': 5685,
    'video_feed': 5533
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

def listen_for_video_feed(port):

    def handle_client_connection(client_socket):
        try:
            while True:
                # First, receive the size of the pickled image
                raw_msglen = recvall(client_socket, 8)  # Assuming you're using struct.pack("L", ...)
                if not raw_msglen:
                    break  # Connection closed
                msglen = struct.unpack("L", raw_msglen)[0]

                # Then, receive the actual image data
                data = recvall(client_socket, msglen)
                if not data:
                    break

                # Unpickle the image data to get the cv_image back
                cv_image = pickle.loads(data)

                # Process the cv_image as needed
                print("Received image")

        finally:
            client_socket.close()

    def recvall(sock, n):
        """Helper function to receive n bytes or return None if EOF is hit"""
        data = b''
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data

    def server_listen():
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            print("server_listen")
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind(('localhost', port))
            server_socket.listen()
            print(f"Server listening on port {port}...")
            while True:
                client_socket, addr = server_socket.accept()
                print(f"Accepted connection from {addr}")
                client_thread = threading.Thread(target=handle_client_connection, args=(client_socket,))
                client_thread.start()

    # Start the server listening function in a new thread so it doesn't block Flask
    threading.Thread(target=server_listen).start()

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
# @sio.on('video_feed')
# def handle_video_feed(data):
#     print('Video feed event received:', data)
#     # Ensure that this doesn't start multiple listeners on the same port
#     if ros2_ports['video_feed'] not in ros2_sockets.keys():
#         start_listening_thread(ros2_ports['video_feed'])

if __name__ == '__main__':
    sio.run(app, debug=True)