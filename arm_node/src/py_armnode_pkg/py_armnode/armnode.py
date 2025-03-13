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

class ArmNed2Node(Node):

    def __init__(self):
        # init Node
        super().__init__('ArmNed2Node_move')

        # init parameters
        self.declare_parameter('x_pos', 0.1)
        self.declare_parameter('y_pos', 0.1)
        self.declare_parameter('x_direction', -1)
        self.declare_parameter('objectsOffset', 0.004)
        
        # init variables
        # turtleBots info
        self.turtleBotRight = TurtleBot()
        self.turtleBotLeft = TurtleBot()
        
        # setup Niryo Arm (ip should be given)
        observationPose = PoseObject(0.219, -0.008, 0.2886, 3.025, 1.346, 2.976)
        self.robot = RobotTriage('172.20.10.2', 'SalleTpSetup', observationPose, 0)
        
        # setup services
        self.srv1 = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.srv2 = self.create_service(TurtleBotArrived, 'InputArm/TurtleBotArrived', self.turtleBotArrived)
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
        self.get_logger().info('Checkin for objects')
        availablesColors = []
        turtleWithoutColor = False
        if self.turtleBotRight.isThere:
            if self.turtleBotRight.objectsColors == "":
                turtleWithoutColor = True
            else:
                availablesColors.append(self.turtleBotRight.objectsColors)
        if self.turtleBotLeft.isThere:
            if self.turtleBotLeft.objectsColors == "":
                turtleWithoutColor = True
            else:
                availablesColors.append(self.turtleBotLeft.objectsColors)

        objectColor = ObjectColor

        armPickedObject = False

        if turtleWithoutColor:
            objectColor = ObjectColor.ANY
            armPickedObject, objectColor = self.robot.pickObject(self.get_parameter('objectsOffset').get_parameter_value().double_value, color = objectColor)

        else: 
            for color in availablesColors:
                if (color == "red"):
                    objectColor = ObjectColor.RED
                elif (color == "green"):
                    objectColor = ObjectColor.GREEN
                elif (color == "blue"):
                    objectColor = ObjectColor.BLUE

                armPickedObject, objectColor = self.robot.pickObject(self.get_parameter('objectsOffset').get_parameter_value().double_value, color = objectColor)
                if armPickedObject:
                    break
            
        if armPickedObject:
            #self.robot.observe()
            color = ""
            if objectColor == ObjectColor.RED:
                color = "red"
            elif objectColor == ObjectColor.GREEN:
                color = "green"	
            elif objectColor == ObjectColor.BLUE:
                color = "blue"
                
            if color in availablesColors:
                #print("the color was in the list")
                if self.turtleBotRight.isThere and self.turtleBotRight.objectsColors == color:
                    pose = self.getPoseOverTurtle(self.turtleBotRight.id)
                    self.robot.placeObject(pose)
                    self.notifyObjectMovement(self.turtleBotRight.id, 1, color)
                else:
                    pose = self.getPoseOverTurtle(self.turtleBotLeft.id)
                    self.robot.placeObject(pose)
                    self.notifyObjectMovement(self.turtleBotLeft.id, 1, color)
                    
            else: 
                #print("turtles with no colors")
                if self.turtleBotRight.isThere and self.turtleBotRight.objectsColors == "":
                    pose = self.getPoseOverTurtle(self.turtleBotRight.id)
                    self.robot.placeObject(pose)
                    self.notifyObjectMovement(self.turtleBotRight.id, 1, color)
                    self.turtleBotRight.setObjectColor(color)
                else:
                    pose = self.getPoseOverTurtle(self.turtleBotLeft.id)
                    self.robot.placeObject(pose)
                    self.notifyObjectMovement(self.turtleBotLeft.id, 1, color)
                    self.turtleBotLeft.setObjectColor(color)

        self.robot.observe()
        
    # calculate a pose to place the object over the turtleBot coordinates based on the x_pos and y_pos as 0 0
    def getPoseOverTurtle(self, turtle_id):
        # position of the arm and direction in wich it is facing (+x or -x)
        x_pos = self.get_parameter('x_pos').get_parameter_value().double_value
        y_pos = self.get_parameter('y_pos').get_parameter_value().double_value
        x_direction = self.get_parameter('x_direction').get_parameter_value().integer_value

        if turtle_id == self.turtleBotRight.id:
            turtleBotNextObjectCoord = self.turtleBotRight.getCoordonatesNextObject()
            pose_x = turtleBotNextObjectCoord[0] - x_pos
            pose_y = turtleBotNextObjectCoord[1] - y_pos
            if (x_direction < 0):
                pose_y = -pose_y
                pose_x = -pose_x
            return PoseObject(round(pose_x, 4), round(pose_y,4), 0.1933, 0.102, 1.465, -1.282)

        else:
            turtleBotNextObjectCoord = self.turtleBotLeft.getCoordonatesNextObject()
            pose_x = turtleBotNextObjectCoord[0] - x_pos
            pose_y = turtleBotNextObjectCoord[1] - y_pos
            if (x_direction < 0):
                pose_y = -pose_y
                pose_x = -pose_x
        return PoseObject(round(pose_x, 4), round(pose_y,4), 0.1933, 1.993, 1.489, -2.579)


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
