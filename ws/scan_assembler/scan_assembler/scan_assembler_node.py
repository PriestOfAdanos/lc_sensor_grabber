from datetime import datetime
from functools import partial

import rclpy
import rosbag2_py
from lc_interfaces.srv import MakeScan, MakeStep
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.parameter import Parameter


# TODo(PriestOfAdanos): Split into separate classes (single responsibility)
class ScanAssembler(Node):
    """Node that controlls pace of scaning and puts individual scans together."""

    def __init__(self):
        super().__init__("scan_assembler_node")
        self.topic_subscriptions = {}
        self.topic_storage_metadata_infos = {}
        self._is_recording = False
        datetime_now = datetime.now().strftime("%d-%m-%Y|%H:%M:%S")
        
        self.declare_parameter('steps_to_full_circle', Parameter.Type.INTEGER)
        self.declare_parameter('make_clockwise_steps', Parameter.Type.BOOL)
        self.declare_parameter('bags_path', Parameter.Type.STRING)
        self.declare_parameter('topics_to_subscribe', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('bag_name', value = 'scan')
            
        self.steps_to_full_circle = self.get_parameter('steps_to_full_circle').value
        self.make_clockwise_steps = self.get_parameter('make_clockwise_steps').value
        self.bags_path = self.get_parameter('bags_path').value
        self.topics_to_subscribe = self.get_parameter('topics_to_subscribe').value
        self.bag_name = self.get_parameter('bag_name').value

        self.make_step_client = self.create_client(MakeStep, "make_step")
        self.srv = self.create_service(
            MakeScan, 'make_scan', self.make_scan_callback)
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri=f"{self.bags_path}/{self.bag_name}-{datetime_now}",
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

    def start_recording(self):
        self.set_recording_on()
        self.topic_name_type_dict = dict(self.get_topic_names_and_types())
        for topic in self.topics_to_subscribe:
            topic_type = self.topic_name_type_dict[topic]
            self.topic_storage_metadata_infos[topic] = rosbag2_py._storage.TopicMetadata(
                name=topic,
                type=topic_type[0],
                serialization_format='cdr')

            self.topic_subscriptions[topic] = self.create_subscription(
                topic_type[1],
                topic,
                partial(self.topic_callback, topic_name=topic),
                10)
            self.writer.create_topic(self.topic_storage_metadata_infos[topic])

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

        for _ in range(self.steps_to_full_circle):
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
        pass # TODO(PriestOfAdanos): add such safety to all nodes

    scan_assembler_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
