from flask import Flask, request
from flask_socketio import SocketIO
import socket
import threading
import asyncio
import struct
import pickle
import cv2
import base64
import numpy as np

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*")

# For ROS2 Nodes Communication
ros2_ports = {
    'manual_control': 5685,  # Port for sending control commands
    'video_feed': 5533       # Port for receiving video feed
}

def send_message(port: int, message: str):
    """Function to send messages to a specific port."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as ros2_socket:
            ros2_socket.connect(('localhost', port))
            ros2_socket.sendall(message.encode())
            response = ros2_socket.recv(1024).decode()
            print(f"Response: {response}")
            # Emit response back to the client if necessary
    except Exception as e:
        print(f'Error sending message to node at port {port}: {e}')

def listen_for_video_feed(port):
    """Function to listen for incoming video feed on a specific port."""
    def handle_client_connection(client_socket):
        try:
            while True:
                # Receive the size of the pickled image
                raw_msglen = recvall(client_socket, 8)
                if not raw_msglen:
                    break
                msglen = struct.unpack("L", raw_msglen)[0]

                # Receive the actual image data
                data = recvall(client_socket, msglen)
                if not data:
                    break

                # Unpickle the image data
                cv_image = pickle.loads(data)
                
                # Encode the image as JPEG
                _, buffer = cv2.imencode('.jpg', cv_image)
                
                # Convert to base64 encoding and decode to string
                jpg_as_text = base64.b64encode(buffer).decode('utf-8')
                sio.emit('image_data', jpg_as_text)
        finally:
            client_socket.close()

    def recvall(sock, n):
        """Helper function to receive n bytes or return None if EOF is hit."""
        data = b''
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data

    def server_listen():
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind(('localhost', port))
            server_socket.listen()
            print(f"Server listening on port {port}...")
            while True:
                client_socket, addr = server_socket.accept()
                print(f"Accepted connection from {addr}")
                client_thread = threading.Thread(target=handle_client_connection, args=(client_socket,))
                client_thread.start()

    # Start the server listening in a new thread
    threading.Thread(target=server_listen).start()

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
    threading.Thread(target=send_message, args=(port, data)).start()
    return data + " Done"

if __name__ == '__main__':
    listen_for_video_feed(ros2_ports['video_feed'])  # Start listening for video feed
    sio.run(app, debug=True, use_reloader=False)