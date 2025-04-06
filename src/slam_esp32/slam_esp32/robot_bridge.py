 
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import socket
import json

class RobotBridge(Node):
    def __init__(self):
        super().__init__('robot_bridge')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # UDP сервер для данных с ESP32
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 1234))
        self.sock.settimeout(0.01)

        self.timer = self.create_timer(0.1, self.read_udp)

    def read_udp(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            try:
                msg = json.loads(data.decode())
                odom = Odometry()
                odom.header.stamp = self.get_clock().now().to_msg()
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_link'
                odom.pose.pose.position.x = msg['x']
                odom.pose.pose.position.y = msg['y']
                odom.pose.pose.orientation.z = sin(msg['theta'] / 2)
                odom.pose.pose.orientation.w = cos(msg['theta'] / 2)
                self.odom_pub.publish(odom)
            except Exception as e:
                self.get_logger().error(f"Parse error: {e}")
        except:
            pass
