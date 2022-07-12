from logging import raiseExceptions
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO 
import time 

from lc_interfaces.srv import MakeStep, SetStepAngle


class EngineControllerNode(Node):
    """Node to make steps on request by set angle."""
    def __init__(self):
        super().__init__('engine_controller_node')
        self.angle = 0
        self.step_angle = 0.0
        self.make_step = self.create_service(MakeStep, 'make_step', self.make_step_callback)
        self.set_step_angle = self.create_service(SetStepAngle, 'set_step_angle', self.set_step_angle_callback)
        #######################
        ##### RP related ######
        #######################
        self.direction_pin = 20 
        self.step_pin = 21  
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)

    def make_step_callback(self, req, res):
        delay = .0208
        GPIO.output(self.DIR, req.make_clockwise_step) # counterclockwise otherwise 
        GPIO.output(self.step_pin, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(self.step_pin, GPIO.LOW)
        time.sleep(delay)
        res.angle = self.angle
        return res
        
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