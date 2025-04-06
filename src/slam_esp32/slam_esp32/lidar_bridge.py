 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket

class LidarBridge(Node):
    def __init__(self):
        super().__init__('lidar_bridge')
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        self.declare_parameters(namespace='', parameters=[
            ('udp_port', 1235),
            ('frame_id', 'laser')
        ])

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.get_parameter('udp_port').value))

        self.timer = self.create_timer(0.1, self.process_data)

        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = self.get_parameter('frame_id').value
        self.scan_msg.angle_min = -3.1415
        self.scan_msg.angle_max = 3.1415
        self.scan_msg.angle_increment = 0.01745  # 1 градус
        self.scan_msg.range_min = 0.15
        self.scan_msg.range_max = 12.0

    def process_data(self):
        try:
            data, _ = self.sock.recvfrom(4096)
            packet = data.decode().split()

            if packet[0] == 'LIDAR':
                ranges = [float('inf')] * 360
                for point in packet[1].split(';'):
                    if not point: continue
                    angle, dist = point.split(',')
                    idx = int(float(angle)*180/3.1415)
                    if 0 <= idx < 360:
                        ranges[idx] = float(dist)/1000.0  # мм в метры

                self.scan_msg.ranges = ranges
                self.scan_msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher.publish(self.scan_msg)

        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main():
    rclpy.init()
    node = LidarBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
