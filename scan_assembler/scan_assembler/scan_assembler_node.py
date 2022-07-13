import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO 
import time 

from lc_interfaces.action import StartScan

class ScanAssembler(Node):
    """Node that controlls pace of scaning and puts individual scans together."""
    def __init__(self):
        super().__init__('start_scan_action_server')
        self._start_scan_action_server = ActionServer(
            self,
            StartScan,
            'start_scan',
            self.start_scan_callback)

    def start_scan_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = StartScan.Feedback()
        feedback_msg.percentage_done = 0

        for i in range(0, 240):
            feedback_msg.percentage_done = int(i/240)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = StartScan.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

def main(args=None):
    rclpy.init(args=args)

    scan_assembler_node = ScanAssembler()

    rclpy.spin(scan_assembler_node)

    scan_assembler_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()