import math
import time

import laser_geometry.laser_geometry as lg
import rclpy
from rclpy.node import Node
from rclpy import qos # import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2


class ScanToPointCloud2(Node):
    """Node that controlls pace of scaning and puts individual scans together."""
    def __init__(self):
        super().__init__('start_scan_action_server')
        self.lp = lg.LaserProjection()
        self.pc2_publisher_ = self.create_publisher(PointCloud2, 'cloud_in', 10)
        # qos = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
        #                  reliability=QoSReliabilityPolicy.RELIABLE,
        #                  history=QoSHistoryPolicy.KEEP_ALL)
        self.ls_subscription_ = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos.qos_profile_sensor_data)

    def scan_callback(self, msg):
        pc2_msg = self.lp.projectLaser(msg)
        self.pc2_publisher_.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    scan_to_point_cloud_node = ScanToPointCloud2()
    try:
        rclpy.spin(scan_to_point_cloud_node)
    except KeyboardInterrupt:
        pass
    scan_to_point_cloud_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
