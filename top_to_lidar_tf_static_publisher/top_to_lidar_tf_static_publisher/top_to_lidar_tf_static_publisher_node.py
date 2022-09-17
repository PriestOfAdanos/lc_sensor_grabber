import sys

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import tf_transformations
import math

class TopToLidarTfStaticPublisher(Node):
   """
   Broadcast transforms that never change.

   This example publishes transforms from `world` to a static turtle frame.
   The transforms are only published once at startup, and are constant for all
   time.
   """

   def __init__(self):
      super().__init__('static_turtle_tf2_broadcaster')

      self._tf_publisher = StaticTransformBroadcaster(self)

      # Publish static transforms once at startup
      self.make_transforms()

   def make_transforms(self):
      static_transformStamped = TransformStamped()
      static_transformStamped.header.stamp = self.get_clock().now().to_msg()
      static_transformStamped.header.frame_id = 'top'
      static_transformStamped.child_frame_id = "laser_frame"

      static_transformStamped.transform.translation.x = 0.142 # 
      static_transformStamped.transform.translation.y = 0.0
      static_transformStamped.transform.translation.z = 0.1
      quat = tf_transformations.quaternion_from_euler(0, math.pi/2, 0,axes='rxyz')

      static_transformStamped.transform.rotation.w = quat[0]
      static_transformStamped.transform.rotation.x = quat[1]
      static_transformStamped.transform.rotation.y = quat[2]
      static_transformStamped.transform.rotation.z = quat[3]

      self._tf_publisher.sendTransform(static_transformStamped)


def main():

   rclpy.init()
   node = TopToLidarTfStaticPublisher()
   try:
      rclpy.spin(node)
   except KeyboardInterrupt:
      pass

   rclpy.shutdown()