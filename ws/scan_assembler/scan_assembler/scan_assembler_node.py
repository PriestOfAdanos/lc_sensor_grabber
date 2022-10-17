import random 
import signal
import string
import subprocess
import time
from functools import partial

import rclpy
from lc_interfaces.srv import MakeStep, MakeScan
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
from rclpy.serialization import serialize_message


import rosbag2_py

### TODo(PriestOfAdanos): Split into separate classes (single responsibility)
class ScanAssembler(Node):
    """Node that controlls pace of scaning and puts individual scans together."""
    def __init__(self):
        self._is_recording = False
        super().__init__("scan_assembler_node")
        #TODO(PriestOfAdanos): move to config
        #{
        self.topic_infos = {}
        self.topic_subscriptions = {}
        self.topic_name_type_dict = {}
        self.make_clockwise_step=True
        self.topic_name_type_dict['scan']= ['sensor_msgs/msg/LaserScan', LaserScan]
        self.topic_name_type_dict['tf']= ['tf2_msgs/msg/TFMessage', TFMessage]
        self.topic_name_type_dict['tf_static']= ['tf2_msgs/msg/TFMessage', TFMessage]
        # }
        self.make_step_client = self.create_client(MakeStep, "make_step")
        self.srv = self.create_service(MakeScan, 'make_scan', self.make_scan_callback)
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri='my_bag', #TODO(PriestOfAdanos): move to config
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)
       
    def start_recording(self):
        self.set_recording_on()
        for topic, topic_type in self.topic_name_type_dict.items():
            self.topic_infos[topic] = rosbag2_py._storage.TopicMetadata(
                name=topic,
                type=topic_type[0],
                serialization_format='cdr')
            
            self.topic_subscriptions[topic] = self.create_subscription(
                topic_type[1],
                topic,
                partial(self.topic_callback, topic_name=topic),
                10) 
            self.writer.create_topic(self.topic_infos[topic])
        
    def topic_callback(self, msg, topic_name):
        self.writer.write(
            topic_name,
            serialize_message(msg),
            self.get_clock().now().nanoseconds)


    def send_request(self):
        self.future = self.make_step_client.call_async(MakeStep.Request())
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("make_step requested")
        return self.future.result()

    def is_recording(self):
        return self._is_recording

    def set_recording_on(self):
        if self.is_recording():
            self._is_recording = True
            
    def set_recording_off(self):
        if self.is_recording():
            self._is_recording = False
            
    def stop_recording(self):
        del self.writer
        self.set_recording_off()
    # TODO(PriestOfAdanos): add error raising to above functions

    def make_scan_callback(self, req, res):
        self.start_recording()
        self.get_logger().info("started to make steps")

        for _ in range(242): # TODO(PriestOfAdanos): move to config
            self.send_request()
        self.stop_recording()
        res.message = "Skan zakończony"
        return res
    
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
