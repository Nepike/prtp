import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket

class CmdVelSender(Node):
    def __init__(self):
        super().__init__('cmd_vel_sender')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.robot_ip = "192.168.106.235"

    def cb(self, msg):
        cmd = f"SET_ROBOT_VELOCITY {msg.linear.x} {msg.angular.z}"
        self.sock.sendto(cmd.encode(), (self.robot_ip, 1234))

def main():
    rclpy.init()
    node = CmdVelSender()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
