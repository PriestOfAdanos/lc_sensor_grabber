import random 
import signal
import subprocess
import time

import rclpy
from lc_interfaces.action import StartScan
from lc_interfaces.srv import MakeStep
from rclpy.action import ActionServer
from rclpy.node import Node


class ScanAssembler(Node):
    """Node that controlls pace of scaning and puts individual scans together."""
    def __init__(self):
        self._is_recording = False
        super().__init__("start_scan_action_server")
        self._start_scan_action_server = ActionServer(
            self,
            StartScan,
            "start_scan",
            self.start_scan_callback)
        self.make_step_client = self.create_client(MakeStep, "make_step")

        while not self.make_step_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = MakeStep.Request()

    def send_request(self):
        self.future = self.make_step_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def is_recording(self):
        return self._is_recording

    def set_recording_on(self):
        if self.is_recording():
            self._is_recording = True
            
    def set_recording_off(self):
        if self.is_recording():
            self._is_recording = False

    # TODO(PriestOfAdanos): add error raising to above functions

    def start_scan_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")
        recording_process = self.start_recording()
        feedback_msg = StartScan.Feedback()
        feedback_msg.percentage_done = 0

        for i in range(242):
            self.send_request()
            feedback_msg.percentage_done = 100*i//240
            self.get_logger().info("Feedback: {0}".format(feedback_msg.percentage_done))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = StartScan.Result()
        result.message = "Done!"
        self.stop_recording(recording_process)
        return result
    
    def start_recording(self):
        # TODO: random need to be replaced with configurable name
        self.set_recording_on()
        return subprocess.Popen(["ros2", "bag", "record", "-a", "-o", f"/bags/{random.randint(0,1000000)}.bag"])
    
    def stop_recording(self, process):
        self.set_recording_off()
        process.send_signal(signal.SIGINT)
        # for whatever reason You need this
    
    def make_step(self):
        return subprocess.Popen(["ros2", "service", "call", "/make_step", "lc_interfaces/srv/MakeStep", "'{make_clockwise_step: True}'"])
        


def main(args=None):
    rclpy.init(args=args)

    scan_assembler_node = ScanAssembler()

    try:
        rclpy.spin(scan_assembler_node)
    except KeyboardInterrupt:
        pass

    scan_assembler_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
