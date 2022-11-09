import math
import time

import rclpy
import tf_transformations
from geometry_msgs.msg import TransformStamped
from lc_interfaces.srv import MakeStep, SetStepAngle
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import TransformBroadcaster
try:
    import RPi.GPIO as GPIO 
except:
    pass

class EngineControllerNode(Node):
    """Node to make steps on request by set angle."""
    def __init__(self):
        super().__init__('engine_controller_node')
        self.angle = 0.0 
        
        self.declare_parameter('steps_to_full_circle', Parameter.Type.INTEGER)
        self.declare_parameter('direction_pin', Parameter.Type.INTEGER)
        self.declare_parameter('step_pin', Parameter.Type.INTEGER)
        self.declare_parameter('prod_mode', Parameter.Type.BOOL)
        self.declare_parameter('delay', Parameter.Type.DOUBLE)
        self.declare_parameter('parent_frame_id', Parameter.Type.STRING)
        self.declare_parameter('child_frame_id', Parameter.Type.STRING)
        self.declare_parameter('translationXYZ', Parameter.Type.DOUBLE_ARRAY)

        self.steps_to_full_circle = self.get_parameter('steps_to_full_circle').value
        self.direction_pin = self.get_parameter('direction_pin').value
        self.step_pin = self.get_parameter('step_pin').value
        self.prod_mode = self.get_parameter('prod_mode').value
        self.delay = self.get_parameter('delay').value
        self.parent_frame_id = self.get_parameter('parent_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.translationXYZ = self.get_parameter('translationXYZ').value

        # 360/242 which is full circle divided by steps required 
        # to make one, thus this is, in degrees, change of one step
        self.step_angle = 360/self.steps_to_full_circle 

        self.make_step = self.create_service(MakeStep, 'make_step', self.make_step_callback)
        self.set_step_angle = self.create_service(SetStepAngle, 'set_step_angle', self.set_step_angle_callback)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.base_link_to_top_tf_publisher()

        if self.prod_mode:
            
            self.RPI_GPIO = GPIO
            self.RPI_GPIO.setmode(self.RPI_GPIO.BCM)
            self.RPI_GPIO.setup(self.direction_pin, self.RPI_GPIO.OUT)
            self.RPI_GPIO.setup(self.step_pin, self.RPI_GPIO.OUT)

    def make_step_callback(self, req, res):
        self.get_logger().info("step made")
        if self.prod_mode:
            self.RPI_GPIO.output(self.direction_pin, req.make_clockwise_step) # counterclockwise otherwise 
            self.RPI_GPIO.output(self.step_pin, self.RPI_GPIO.HIGH)
            time.sleep(self.delay)
            self.RPI_GPIO.output(self.step_pin, self.RPI_GPIO.LOW)
            time.sleep(self.delay)
        
        self.angle += self.step_angle
        self.base_link_to_top_tf_publisher()
        res.done = True
        return res
    
    def base_link_to_top_tf_publisher(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame_id 
        t.child_frame_id = self.child_frame_id 
        
        (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z) = self.translationXYZ
        q = tf_transformations.quaternion_from_euler(0, 0, math.radians(self.angle),axes='rxyz')
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
    try:
        rclpy.spin(engine_controller_node)
    except KeyboardInterrupt:
        pass
    engine_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
