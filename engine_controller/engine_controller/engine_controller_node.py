import time 
import math
from xml.dom import NotFoundErr

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

try:
    import RPi.GPIO as GPIO 
    
    PROD_MODE=True
except RuntimeError:
    PROD_MODE=False



from lc_interfaces.srv import MakeStep, SetStepAngle

class EngineControllerNode(Node):
    """Node to make steps on request by set angle."""
    def __init__(self):
        super().__init__('engine_controller_node')
        self.angle = 0.0 

        # 360/242 which is full circle divided by steps required 
        # to make one, thus this is, in degrees, change of one step
        self.step_angle = 1.4835 # TODO(PriestOfAdanos): to config.yaml

        self.make_step = self.create_service(MakeStep, 'make_step', self.make_step_callback)
        self.set_step_angle = self.create_service(SetStepAngle, 'set_step_angle', self.set_step_angle_callback)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.base_link_to_top_tf_publisher()

        self.direction_pin = 20  # TODO(PriestOfAdanos): to config.yaml
        self.step_pin = 21   # TODO(PriestOfAdanos): to config.yaml
        if PROD_MODE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.direction_pin, GPIO.OUT)
            GPIO.setup(self.step_pin, GPIO.OUT)

    def make_step_callback(self, req, res):
        delay = .0208 # TODO(PriestOfAdanos): to config.yaml
        if PROD_MODE:
            GPIO.output(self.direction_pin, req.make_clockwise_step) # counterclockwise otherwise 
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)
        
        self.angle += self.step_angle
        self.base_link_to_top_tf_publisher()
        res.done = True
        return res
    
    def base_link_to_top_tf_publisher(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link' # TODO(PriestOfAdanos): to config.yaml
        t.child_frame_id = 'top' # TODO(PriestOfAdanos): to config.yaml

        t.transform.translation.x = 0.0 # TODO(PriestOfAdanos): to config.yaml
        t.transform.translation.y = 0.0 # TODO(PriestOfAdanos): to config.yaml
        t.transform.translation.z = 0.0 # TODO(PriestOfAdanos): to config.yaml

        q = tf_transformations.quaternion_from_euler(0, 0, math.radians(self.angle),axes='rxyz') # TODO(PriestOfAdanos): to config.yaml
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]

        self.tf_broadcaster.sendTransform(t)
        
    def set_step_angle_callback(self, req, res):
        self.step_angle = req.step_angle
        res.step_angle_set = True
        self.get_logger().info('Step angle set to %d ' % (req.step_angle))
        return res


def main(args=None):
    rclpy.init(args=args)

    engine_controller_node = EngineControllerNode()

    rclpy.spin(engine_controller_node)

    engine_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()