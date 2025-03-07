from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node

from pyniryo import NiryoRobot, JointsPosition

class ArmNed2Node(Node):

    def __init__(self):
        super().__init__('ArmNed2Node_move')
        self.robot = NiryoRobot('192.168.242.126')
        self.robot.calibrate_auto()
        self.robot.update_tool()
        self.robot.move(JointsPosition(0.2, -0.3, 0.1, 0.0, 0.5, -0.8))
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info("Service is ready")
        #print("done init")

    def add_two_ints_callback(self, request, response):
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


def main(args=None):
    rclpy.init(args=args)

    minimal_service = ArmNed2Node()

    while rclpy.ok():
    	rclpy.spin_once(minimal_service)

    print("Shutting Down")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
