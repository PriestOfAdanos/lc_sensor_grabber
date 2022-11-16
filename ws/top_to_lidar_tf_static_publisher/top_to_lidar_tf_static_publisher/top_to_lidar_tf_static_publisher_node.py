import math
import sys

import rclpy
import tf_transformations
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rclpy.parameter import Parameter


class TopToLidarTfStaticPublisher(Node): 
   def __init__(self):
      super().__init__('top_to_lidar_tf_static_publisher_node')
      self._tf_publisher = StaticTransformBroadcaster(self)
      # Publish static transforms once at startup
      self.declare_parameter('parent_frame_id', Parameter.Type.STRING)
      self.declare_parameter('child_frame_id', Parameter.Type.STRING)
      self.declare_parameter('translationXYZ', Parameter.Type.DOUBLE_ARRAY)

      self.parent_frame_id = self.get_parameter('parent_frame_id').value
      self.child_frame_id = self.get_parameter('child_frame_id').value
      self.translationXYZ = self.get_parameter('translationXYZ').value

      self.make_transforms()

   def make_transforms(self):
      static_transformStamped = TransformStamped()
      static_transformStamped.header.stamp = self.get_clock().now().to_msg()
      static_transformStamped.header.frame_id = self.parent_frame_id # TODO(PriestOfAdanos): move to config
      static_transformStamped.child_frame_id = self.child_frame_id # TODO(PriestOfAdanos): move to config

      (static_transformStamped.transform.translation.x, 
       static_transformStamped.transform.translation.y, 
       static_transformStamped.transform.translation.z) = self.translationXYZ
      
      quat = tf_transformations.quaternion_from_euler(0, -math.pi/2, 0,axes='rxyz')
      
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
   
if __name__ == '__main__':
    main()
