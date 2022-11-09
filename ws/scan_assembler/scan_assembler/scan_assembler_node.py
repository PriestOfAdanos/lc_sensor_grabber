import time
from datetime import datetime
from functools import partial
from threading import Event

import laser_geometry.laser_geometry as lg
import lc_interfaces
import rclpy
import rosbag2_py
import sensor_msgs.msg
import tf2_msgs.msg
from lc_interfaces.srv import MakeScan, MakeStep
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message

# from builtin_interfaces.msg import Time
# TODo(PriestOfAdanos): Split into separate classes (single responsibility)
class ScanAssembler(Node):
    """Node that controlls pace of scaning and puts individual scans together."""

    def __init__(self):
        super().__init__("scan_assembler_node")

        self.topic_subscriptions = {}
        self.topic_storage_metadata_infos = {}
        self._is_recording = False
        datetime_now = datetime.now().strftime("%d-%m-%Y|%H:%M:%S")
        self.reentrant_callback_group = ReentrantCallbackGroup()

        self.declare_parameter("target_frame", Parameter.Type.STRING)
        self.declare_parameter("steps_to_full_circle", Parameter.Type.INTEGER)
        self.declare_parameter("make_clockwise_steps", Parameter.Type.BOOL)
        self.declare_parameter("bags_path", Parameter.Type.STRING)
        self.declare_parameter("topics_to_subscribe", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("pause_beetwen_steps", Parameter.Type.INTEGER)
        self.declare_parameter("bag_name", value="scan")

        self.target_frame = self.get_parameter("target_frame").value
        self.steps_to_full_circle = self.get_parameter("steps_to_full_circle").value
        self.make_clockwise_steps = self.get_parameter("make_clockwise_steps").value
        self.bags_path = self.get_parameter("bags_path").value
        self.topics_to_subscribe = self.get_parameter("topics_to_subscribe").value
        self.pause_beetwen_steps = self.get_parameter("pause_beetwen_steps").value
        self.bag_name = self.get_parameter("bag_name").value

        self.make_step_client = self.create_client(
            MakeStep, "make_step", callback_group=self.reentrant_callback_group
        )

        self.make_scan_service = self.create_service(
            MakeScan,
            "make_scan",
            self.make_scan_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri=f"{self.bags_path}/{self.bag_name}-{datetime_now}", storage_id="sqlite3"
        )
        converter_options = rosbag2_py._storage.ConverterOptions("", "")
        self.writer.open(storage_options, converter_options)

    def start_recording(self):
        self.set_recording_on()
        self.topic_name_type_dict = dict(self.get_topic_names_and_types())
        # call service to make draft pointcloud2

        for topic in self.topics_to_subscribe:
            topic_type_list = self.topic_name_type_dict[topic]
            self.topic_storage_metadata_infos[
                topic
            ] = rosbag2_py._storage.TopicMetadata(
                name=topic, type=topic_type_list[0], serialization_format="cdr"
            )

            self.topic_subscriptions[topic] = self.create_subscription(
                eval(topic_type_list[0].replace("/", ".")),
                topic,
                partial(self.topic_callback, topic_name=topic),
                10,
            )
            self.writer.create_topic(self.topic_storage_metadata_infos[topic])

    def topic_callback(self, msg, topic_name):
        self.writer.write(
            topic_name, serialize_message(msg), self.get_clock().now().nanoseconds
        )

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
        for _ in range(self.steps_to_full_circle):
            time.sleep(self.pause_beetwen_steps)
            self.get_logger().info("started to make steps")
            event = Event()

            def done_callback(future):
                nonlocal event
                event.set()

            future = self.make_step_client.call_async(MakeStep.Request())
            future.add_done_callback(done_callback)
            event.wait()
            self.get_logger().info("make_step done")
        self.stop_recording()
        res.message = "Skan zakończony"
        return res


def main(args=None):
    rclpy.init(args=args)
    scan_assembler_node = ScanAssembler()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(scan_assembler_node, executor)
    except KeyboardInterrupt:
        scan_assembler_node.stop_recording()
    scan_assembler_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
