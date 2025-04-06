 
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import json
import socket
from geometry_msgs.msg import TwistStamped

class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_vel_sub = self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_vel_cb, 10)

        self.udp_port = 1234
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.udp_port))
        self.sock.setblocking(False)

        self.target_vel = (0.0, 0.0)

        self.timer = self.create_timer(0.1, self.process_odom)
        self.send_timer = self.create_timer(0.1, self.send_velocity)

    def process_odom(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            msg = json.loads(data.decode())

            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'

            odom_msg.pose.pose.position.x = msg['x']
            odom_msg.pose.pose.position.y = msg['y']
            odom_msg.pose.pose.orientation.z = sin(msg['theta']/2)
            odom_msg.pose.pose.orientation.w = cos(msg['theta']/2)

            odom_msg.twist.twist.linear.x = msg['vx']
            odom_msg.twist.twist.angular.z = msg['vyaw']

            self.odom_pub.publish(odom_msg)

        except:
            pass

    def cmd_vel_cb(self, msg):
        self.target_vel = (msg.twist.linear.x, msg.twist.angular.z)

    def send_velocity(self):
        cmd = f"SET_ROBOT_VELOCITY {self.target_vel[0]} {self.target_vel[1]}"
        self.sock.sendto(cmd.encode(), ('192.168.106.235', 1234))  # IP ESP32

def main():
    rclpy.init()
    node = OdomBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
