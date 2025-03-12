from pyniryo import NiryoRobot, PoseObject, ObjectShape, ObjectColor

import time

class RobotTriage(NiryoRobot):
    '''
    Robot representation with additional methods for picking and placing objects.
    Inherits from NiryoRobot class.

    :param ip: str: IP address of the robot
    :param workspace_name: str: Name of the workspace
    :param verbose: int: Verbosity level (0 = no output, 1 = debug output)

    Attributes
    ----------
    workspace_name : str
        Name of the workspace
    verbose : int
        Verbosity level
    '''
    def __init__(self, ip,workspace_name,observePosition,verbose=1):
        '''
        Constructs all the necessary attributes for the robot object.

        :param ip: str: IP address of the robot
        :param workspace_name: str: Name of the workspace
        :param verbose: int: Verbosity level (0 = no output, 1 = debug output)
        '''
        super().__init__(ip)
        self.calibrate_auto()
        self.update_tool()
        self.clear_collision_detected()
        self.workspace_name = workspace_name
        self.verbose = verbose
        self.observePosition = observePosition
    
    def __log(self,message):
        '''
        Logs a message if verbosity is enabled.

        :param message: str: The message to log
        '''
        if self.verbose:
            print(message)
    
    def observe(self):
        '''
        Moves the robot to the given pose and returns the current pose.

        :param pose: PoseObject: The target pose for the robot
        :return: PoseObject: The current pose of the robot
        '''
        self.__log(f"Observing this pose: {self.observePosition}")
        joints = self.inverse_kinematics(self.observePosition)
        self.move(joints)
        time.sleep(0.5)
        return self.get_pose()

    def pickObject(self, heightOffset, shape = ObjectShape.ANY,color = ObjectColor.ANY):
        '''
        Picks an object with the specified height offset, shape, and color.

        :param heightOffset: float: The height offset for picking the object
        :param shape: ObjectShape, optional: The shape of the object to pick (default is ObjectShape.ANY)
        :param color: ObjectColor, optional: The color of the object to pick (default is ObjectColor.ANY)
        :return: bool: True if the object was found, False otherwise
        '''
        self.release_with_tool()
        obj_found, shape_obj, color_ret = self.vision_pick(self.workspace_name,height_offset=heightOffset,shape=shape,color=color)
        self.__log(f"Object found: {obj_found}")
        self.observe()
        return obj_found, color_ret
    
    def placeObject(self,pose : PoseObject):
        '''
        Places an object at the specified pose.

        :param pose: PoseObject: The target pose for placing the object
        '''
        self.__log(f"Placing object at pose: {pose}")
        poseError = False
        try:
            jointsUp = self.inverse_kinematics(pose)
        except:
            poseError = True
            while poseError:
            	self.__log(f"Pose not reachable, recalculating pose {pose}")
            	isPoseXPositive = pose.x >= 0
            	isPoseYPositive = pose.y >= 0
            	if pose.x != 0:
                    if isPoseXPositive:
                        pose.x -= 0.01
                    else:
                        pose.x += 0.01
            	if pose.y != 0:
                    if isPoseYPositive:
                        pose.y -= 0.01
                    else:
                        pose.y += 0.01
            	try:
                    jointsUp = self.inverse_kinematics(pose)
                    poseError = False
            	except:
                    poseError = True
            
        # self.move(pose.x/2.0, pose.y/2.0, pose.z, pose.roll,pose.pitch,pose.yaw)
        # time.sleep(0.5)
        self.move(jointsUp)
        self.__log(f"Arrived at: {self.get_pose()}")
    
        time.sleep(0.5)

        poseDown = PoseObject(pose.x,pose.y,pose.z - 0.10,pose.roll,pose.pitch,pose.yaw - 0.2)
        jointsDown = self.inverse_kinematics(poseDown)
        self.move(jointsDown)
        time.sleep(0.5)
        self.__log(f"Place object at: {self.get_pose()}")

        self.release_with_tool()
        self.move(jointsUp)
        time.sleep(0.5)
        return





