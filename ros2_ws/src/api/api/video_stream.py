import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import socket
import pickle
import struct

class VideoStream(Node):
    def __init__(self):
        super().__init__('video_stream')
        self.subscription_image = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.subscription_info = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.info_callback,
            10)
        self.bridge = CvBridge()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect(('localhost', 5533))
        self.get_logger().info("VideoStream Node Started, connected to socket server")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        data = pickle.dumps(cv_image)
        message_size = struct.pack("L", len(data))  # Unsigned long, Little Endian
        self.socket.sendall(message_size + data)

    def info_callback(self, msg):
        data = pickle.dumps(msg)
        message_size = struct.pack("L", len(data))  # Unsigned long, Little Endian
        self.socket.sendall(message_size + data)

    def __del__(self):
        self.socket.close()

def main(args=None):
    rclpy.init(args=args)
    video_stream_node = VideoStream()
    rclpy.spin(video_stream_node)
    video_stream_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
