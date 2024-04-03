import rclpy
from rclpy.node import Node
import socket
import threading
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from pydantic import BaseModel

# Assuming Response is not used since it's not utilized in the original script.
# class Response(BaseModel):
#     msg: str

class ManualControl(Node):

    def __init__(self):
        super().__init__('api_manual_control_node')
        self.publisher_land = self.create_publisher(Empty, 'land', 1)
        self.publisher_flip = self.create_publisher(String, 'flip', 1)
        self.publisher_takeoff = self.create_publisher(Empty, 'takeoff', 10)
        self.publisher_velocity = self.create_publisher(Twist, 'control', 1)
        self.publisher_emergency = self.create_publisher(Empty, 'emergency', 1)

        self.manual_speed = 50.0  # Speed of the drone in manual control mode

        self.timer = self.create_timer(0.001, self.timer_callback)
        self.run()

    def socket_handler(self, client_socket, addr):
        with client_socket:
            print(f"Connected by {addr}")
            while True:
                data = client_socket.recv(1024)
                if not data:
                    break
                command = data.decode()
                self.get_logger().info(f"Received command: {command}")
                self.timer_callback(command)
                response_message = f"{command} command sent\n".encode()
                client_socket.sendall(response_message)

    def manual_control(self, command):
        msg = Twist()

        if command == "rotate_left":
            msg.angular.z = -self.manual_speed
        elif command == "rotate_right":
            msg.angular.z = self.manual_speed
        elif command == "up":
            msg.linear.z = self.manual_speed
        elif command == "down":
            msg.linear.z = -self.manual_speed
        elif command == "left":
            msg.linear.x = -self.manual_speed
        elif command == "right":
            msg.linear.x = self.manual_speed
        elif command == "forward":
            msg.linear.y = self.manual_speed
        elif command == "backward":
            msg.linear.y = -self.manual_speed

        self.publisher_velocity.publish(msg)

    def timer_callback(self, command):
        if command == "takeoff":
            # Perform the action corresponding to "start"
            self.publisher_takeoff.publish(Empty())
        elif command == "land":
            # Perform the action corresponding to "start"
            self.publisher_land.publish(Empty())
        elif command == "flip":
            # Perform the action corresponding to "start"
            self.publisher_flip.publish(String(data="f"))
        elif command == "e":
            self.publisher_emergency.publish(Empty())
        else:
            self.manual_control(command)
        
    def run(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('localhost', 5685))
        self.server_socket.listen()
        
        while True:
            client_socket, addr = self.server_socket.accept()
            client_thread = threading.Thread(target=self.socket_handler, args=(client_socket, addr))
            client_thread.start()

    def shutdown(self):
        # Shutdown logic for your node and sockets if necessary
        self.server_socket.close()
        pass

def main(args=None):
    rclpy.init()
    node = ManualControl()
    try:
        rclpy.spin(node)
    except:
        pass
    finally:
        print("Shutting down node")
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()