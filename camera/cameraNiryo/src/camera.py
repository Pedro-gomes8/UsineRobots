from pyniryo import NiryoRobot, PoseObject, ObjectShape, ObjectColor
# from ..include import cameraFunc
import time

# - Constants
visionWorkspace = "t"


robot = NiryoRobot("172.20.10.12")

# Calibrate robot if it needs calibration
robot.calibrate_auto()

robot.update_tool()

object_offset = 0.3


# observation_pose = PoseObject(0,0,0.2,-2.8,1.3,2.9)
observation_pose = PoseObject(0.2524,0.0019,0.2172,3.034,1.306,3.137)

destination_pos = PoseObject(0.0088,0.2193,0.3233,2.429,1.532,-2.248)


def placeObject(x,y,z,roll,pitch,yaw):

    destinationUpPosition = PoseObject(x,y,z,roll,pitch,yaw)
    jointsUp = robot.inverse_kinematics(destinationUpPosition)
    robot.move(jointsUp)

    # Delay to ensure position
    time.sleep(1)
    destinationDownPosition = PoseObject(x,y,z - 0.10,roll,pitch,yaw - 0.2)
    jointsDown = robot.inverse_kinematics(destinationDownPosition)
    robot.move(jointsDown)


    # robot.move(destinationDownPosition)
    time.sleep(1)

    robot.release_with_tool()
    robot.move(jointsUp)
    time.sleep(1)
    return 

def main():

    maxCatchCount = 2
    catchCount = 0
    pose_read = robot.get_pose()
    joints = robot.inverse_kinematics(observation_pose)
    print(pose_read)
    while catchCount < maxCatchCount:
        robot.clear_collision_detected()
        robot.release_with_tool()
        print("move")

        robot.move(joints)

        time.sleep(1)
        pose_read = robot.get_pose()
        print(pose_read)

        obj_found, shape_obj, color_ret = robot.vision_pick(visionWorkspace,height_offset=0.00,shape=ObjectShape.ANY,color=ObjectColor.ANY)
        print(obj_found)
        if not obj_found:
            robot.wait(0.1)
            continue
            
        else:
            placeObject(0.0088,0.2193,0.3233,2.429,1.532,-2.248)
        catchCount +=1
        print(f"object Count: {catchCount}")
    robot.move(joints)

    robot.close_connection()


if __name__ == "__main__":
    main()