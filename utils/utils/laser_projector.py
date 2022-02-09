import rclpy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry as lg
import math
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2 as pc2



class LaserScanToPointCloud(Node):
    def __init__(self):
        super().__init__('laserscan_to_pointcloud')
        self.publisher = self.create_publisher(PointCloud2, "/point_cloud", 10)
        self.lp = lg.LaserProjection()
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            10)

    def scan_cb(self,msg):
        pc2_msg = self.lp.projectLaser(msg)
        self.publisher.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    laserscan_to_pointcloud = LaserScanToPointCloud()
    rclpy.spin(laserscan_to_pointcloud)
    laserscan_to_pointcloud.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
