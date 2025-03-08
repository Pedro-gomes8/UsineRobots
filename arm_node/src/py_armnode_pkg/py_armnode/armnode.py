#!/usr/bin/env python3

from example_interfaces.srv import AddTwoInts

from py_armnode_pkg.srv import TurtleBotArrived

import rclpy
from rclpy.node import Node

from pyniryo import NiryoRobot, JointsPosition

from turtleBotClass import TurtleBot

class ArmNed2Node(Node):

    def __init__(self):
        # init Node
        super().__init__('ArmNed2Node_move')

        # init parameters
        self.declare_parameter('x_pos', 0.1)
        self.declare_parameter('y_pos', 0.1)
        self.declare_parameter('x_direction', 1)
        
        # init variables
        # turtleBots info
        self.trutleBotRight = TurtleBot()
        self.trutleBotLeft = TurtleBot()
        
        # setup Niryo Arm (ip should be given)
        self.robot = NiryoRobot('192.168.242.126')
        self.robot.calibrate_auto()
        self.robot.update_tool()
        self.robot.move(JointsPosition(0.2, -0.3, 0.1, 0.0, 0.5, -0.8))
        
        # setup services
        self.srv1 = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.srv2 = self.create_service(TurtleBotArrived, 'srv/TurtleBotArrived', self.turtleBotArrived)
        
        self.get_logger().info("Service is ready")

    def add_two_ints_callback(self, request, response):
        print("callback")
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        # a is second value (J2-shoulder) with min = -1.83 and max = 0.61
        # b is third value (J3-Elbow) with min = -1.34 and max = 1.57
        a = float(request.a)
        b = float(request.b)

        a = min(0.6, max(-1.85, a))
        b = min(1.56, max(-1.35, b))
        
        self.robot.move(JointsPosition(0.2, a, b, 0.0, 0.5, -0.8))
        self.get_logger().info('Sending response: %d' % response.sum)

        return response
        
    # Callback function for the service -> Update the turtleBot info when it arrives
    def turtleBotArrived(self, request, response):
        self.get_logger().info('Incoming request\nid: %d x_pos: %f y_pos: %f color: %s ' % (request.data.id, request.data.x_pos, request.data.y_pos, request.data.color))

        isUp = request.data.y_pos > self.get_parameter('y_pos').get_parameter_value().double_value
        isRight = isUp and self.get_parameter('x_direction').get_parameter_value().integer_value < 0 or not isUp and self.get_parameter('x_direction').get_parameter_value().integer_value > 0

        self.get_logger().info('Incoming request\ny_pos_Arm: %f x_direction: %d x_pos_Arm: %f color: %s ' % (self.get_parameter('y_pos').get_parameter_value().double_value, self.get_parameter('x_direction').get_parameter_value().integer_value, self.get_parameter('x_pos').get_parameter_value().double_value, request.data.color))
        
        if isRight:
            self.trutleBotRight.turtleBotArrived(request.data.x_pos, request.data.y_pos, request.data.color, request.data.id)
            self.get_logger().info('Setting Right turtleBot')
        else:
            self.trutleBotLeft.turtleBotArrived(request.data.x_pos, request.data.y_pos, request.data.color, request.data.id)
            self.get_logger().info('Setting Left turtleBot')
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = ArmNed2Node()

    while rclpy.ok():
    	rclpy.spin_once(minimal_service)

    print("Shutting Down")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
