from pyniryo import NiryoRobot, PoseObject, ObjectShape, ObjectColor
# from ..include import cameraFunc
import time

# - Constants
visionWorkspace = "newTest"


robot = NiryoRobot("10.10.10.10")

# Calibrate robot if it needs calibration
robot.calibrate_auto()

robot.update_tool()

object_offset = 0.3


observation_pose = PoseObject(200,-4.8,227,2.9,0.98,3)
destination_pos = PoseObject(-68,257,340,2,1.5,-2.5)


def placeObject(x,y,z,roll,pitch,yaw):

    destinationUpPosition = PoseObject(x,y,z,roll,pitch,yaw)
    robot.move(destinationUpPosition)

    # Delay to ensure position
    time.sleep(1)
    destinationDownPosition = PoseObject(x,y,z - 50,roll,pitch,yaw)

    robot.move(destinationDownPosition)
    time.sleep(1)

    robot.release_with_tool()
    return 

def main():

    maxCatchCount = 2
    catchCount = 0

    while catchCount < maxCatchCount:
        robot.release_with_tool()
        robot.move(observation_pose)

        obj_found, shape_obj, color_ret = robot.vision_pick(visionWorkspace,height_offset=object_offset,shape=shape,color=color)

        if not obj_found:
            robot.wait(0.1)
            continue
            
        else:
            placeObject(destination_pos)
        catchCount +=1
        print(f"object Count: {catchCount}")

    robot.close_connection()
