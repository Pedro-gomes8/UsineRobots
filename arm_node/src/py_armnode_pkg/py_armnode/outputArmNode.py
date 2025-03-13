#!/usr/bin/env python3

from example_interfaces.srv import AddTwoInts

from arm_interface.srv import TurtleBotArrived
from coordinator_interface.srv import NotifyObjectMovement

import rclpy
from rclpy.node import Node

import os
import sys

import time
from pyniryo import JointsPosition, PoseObject, ObjectColor

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', 'camera', 'cameraNiryo', 'src')))

from RobotTriage import RobotTriage

from turtleBotClass import TurtleBot

class ObjectOutputDeposit:
    def __init__(self, pose, color):
        self.jointsDroppingPose = pose
        self.color = color


class OutputArmNed2Node(Node):

    def __init__(self):
        # init Node
        super().__init__('OutputArmNed2Node')

        # init parameters
        self.declare_parameter('x_pos', 3.6)
        self.declare_parameter('y_pos', 0.0)
        self.declare_parameter('x_direction', 1)
        self.declare_parameter('objectsOffset', 0.004)
        
        # init variables
        # turtleBots info
        self.turtleBotRight = TurtleBot()
        self.turtleBotLeft = TurtleBot()
        self.firstTurtleArrived = -1 # 0: left, 1: right -1: none

        self.objectsOutputDeposits = [
            ObjectOutputDeposit(JointsPosition(-0.253, -0.473, -0.093, 0.042, -0.177, -0.121), "red"),
            ObjectOutputDeposit(JointsPosition(-0.014, -0.344, -0.255, 0.045, -0.216, -0.120), "green"),
            ObjectOutputDeposit(JointsPosition(0.203, -0.250, -0.328, 0.223, -0.216, -0.118), "blue")]
        
        # setup Niryo Arm (ip should be given)

        self.intermediateDroppingJoints = JointsPosition(0.054, 0.525, -0.869, 0.058, -0.520, -0.032)

        # TODO: change this
        self.robot = RobotTriage('172.20.10.2', 'SalleTpSetup', observationPose, 0)
        
        # setup services
        self.srv1 = self.create_service(TurtleBotArrived, 'OutputArm/TurtleBotArrived', self.turtleBotArrived)
        self.get_logger().info("Service is ready")
        
        # setup clients
        self.cli = self.create_client(NotifyObjectMovement, 'notify_object_movement')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info("Client is ready")
        self.req = NotifyObjectMovement.Request()
        
        self.client_futures = []
        self.turtleId_futures = []

        self.robot.observe()
        
        timer_period = 7  # seconds
        self.timer = self.create_timer(timer_period, self.checkForObjects)
        
    # Callback function for the service -> Update the turtleBot info when it arrives
    def turtleBotArrived(self, request, response):
        self.get_logger().info('Incoming request\nid: %d x_pos: %f y_pos: %f color: %s ' % (request.id, request.x_pos, request.y_pos, request.color))

        isUp = request.y_pos > self.get_parameter('y_pos').get_parameter_value().double_value
        isRight = isUp and self.get_parameter('x_direction').get_parameter_value().integer_value < 0 or not isUp and self.get_parameter('x_direction').get_parameter_value().integer_value > 0

        self.get_logger().info('Incoming request\ny_pos_Arm: %f x_direction: %d x_pos_Arm: %f color: %s ' % (self.get_parameter('y_pos').get_parameter_value().double_value, self.get_parameter('x_direction').get_parameter_value().integer_value, self.get_parameter('x_pos').get_parameter_value().double_value, request.color))
        
        if isRight:
            self.turtleBotRight.turtleBotArrived(request.x_pos, request.y_pos, request.color, request.id)
            self.get_logger().info('Setting Right turtleBot')
        else:
            self.turtleBotLeft.turtleBotArrived(request.x_pos, request.y_pos, request.color, request.id)
            self.get_logger().info('Setting Left turtleBot')
        return response
        
    def notifyObjectMovement(self, turtle_id, obj_diff, object_color):
        self.get_logger().info('Incoming request\nturtle_id: %d obj_diff: %d object_color: %s' % (turtle_id, obj_diff, object_color))

        self.req.turtle_id = turtle_id
        self.req.obj_diff = obj_diff
        self.req.object_color = object_color
        
        self.client_futures.append(self.cli.call_async(self.req))
        self.turtleId_futures.append(turtle_id)
        return
        
    def turtleLeft(self, turtleId):
        if (self.turtleBotRight.id == turtleId):
            self.turtleBotRight.turtleBotLeft()
            self.get_logger().info('Right turtleBot left')
        else:
            self.turtleBotLeft.turtleBotLeft()
            self.get_logger().info('Left turtleBot left')
        
    def checkForObjects(self):
        if (len(self.client_futures) > 0):
            return
        
        self.get_logger().info('Checking for objects')
        if (not self.turtleBotLeft.isThere and not self.turtleBotRight.isThere):
            self.get_logger().info('No objects to move')
            return
        
        if (self.firstTurtleArrived == -1):
            if (self.turtleBotLeft.isThere):
                self.firstTurtleArrived = 0
            else:
                self.firstTurtleArrived = 1
        else:
            if (self.firstTurtleArrived == 0 and self.turtleBotLeft.isThere):
                print("Pick up left")
                self.pickUpObject(0)
            elif (self.firstTurtleArrived == 1 and self.turtleBotRight.isThere):
                print("Pick up right")
                self.pickUpObject(1)
            else:
                if (not self.turtleBotLeft.isThere and self.turtleBotRight.isThere):
                    self.firstTurtleArrived = 1
                    self.pickUpObject(1)
                    print("Pick up right")
                elif (self.turtleBotLeft.isThere and not self.turtleBotRight.isThere):
                    self.firstTurtleArrived = 0
                    self.pickUpObject(0)
                    print("Pick up left")
                else:
                    self.firstTurtleArrived = -1

        

    def pickUpObject(self, side):
        self.get_logger().info('Picking up object')
        if (side == 0):
            turtleId = self.turtleBotLeft.id
            color = self.turtleBotLeft.color
        else:
            turtleId = self.turtleBotRight.id
            color = self.turtleBotRight.color
        # pick up object from side


        self.dropObjectOnDesposits(color)

        # after picking up object, notify the object movement
        self.notifyObjectMovement(turtleId, -1, "red")


    def dropObjectOnDesposits(self, color):
        self.robot.move_to(self.intermediateDroppingJoints)
        if (color == "red"):
            self.robot.move_to(self.objectsOutputDeposits[0].jointsDroppingPose)
        elif (color == "green"):
            self.robot.move_to(self.objectsOutputDeposits[1].jointsDroppingPose)
        else: # blue
            self.robot.move_to(self.objectsOutputDeposits[2].jointsDroppingPose)

        self.robot.open_gripper()
        self.robot.wait(1)

        self.robot.move_to(self.intermediateDroppingJoints)



        
        
        

def main(args=None):
    rclpy.init(args=args)

    armNode = ArmNed2Node()

    #lastCheck = 0
    #deltaTime = 10
    
    while rclpy.ok():
        rclpy.spin_once(armNode)
        incomplete_futures = []
        incomplete_ids = []
        index = 0
        for f in armNode.client_futures:
            if f.done():
                res = f.result()
                if (res.ack == 1):
                    armNode.turtleLeft(armNode.turtleId_futures[index])
            else:
                incomplete_futures.append(f)
                incomplete_ids.append(armNode.turtleId_futures[index])
            index += 1
        armNode.client_futures = incomplete_futures
        armNode.turtleId_futures = incomplete_ids
    	    
    print("Shutting Down")

    rclpy.shutdown()


if __name__ == '__main__':
    main()