import os
import time

import rclpy
try:
    import RPi.GPIO as GPIO
    PROD_MODE=True
except RuntimeError:
    PROD_MODE=False


from rclpy.node import Node
from bluedot import BlueDot

from rclpy.action import ActionClient

from lc_interfaces.action import StartScan



class BluetoothTalker(Node):
    """Node that controlls pace of scaning and puts individual scans together."""
    def __init__(self):
        super().__init__('bluetooth_talker')
        self._action_start_recording_client = ActionClient(self, StartScan, 'start_scan_action_server')
    
    def start_recording(self):
        goal_msg = StartScan.Goal()
        goal_msg.request = 1
        self._action_start_recording_client.wait_for_server()
        return self._action_start_recording_client.send_goal_async(goal_msg)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

def main(args=None):
    rclpy.init(args=args)
    bluetooth_talker = BluetoothTalker()
    bd = BlueDot()
    try:
        if bd.wait_for_press():
            bluetooth_talker.start_recording()
        rclpy.spin(bluetooth_talker)
    except KeyboardInterrupt:
        pass
    bluetooth_talker.destroy_node()
    server.stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
