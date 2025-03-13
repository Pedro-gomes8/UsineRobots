from pyniryo import PoseObject, ObjectColor

import sys
import os

# Add the parent directory of cameraNiryo to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from RobotTriage import RobotTriage

robot_ip = "172.20.10.12"
visionWorkspace = "t"
observation_pose = PoseObject(0.2016,0.021,0.25,2.824,1.280,3.078)


robot = RobotTriage(robot_ip,visionWorkspace,observation_pose,verbose=1)


# Controls object height
object_offset = 0.00

# Destination position to place the object
destination_pos = PoseObject(0.0088,0.2193,0.3233,2.429,1.532,-2.248)

def main():

    maxCatchCount = 2
    catchCount = 0

    # catches maxCatchCount objects
    while catchCount < maxCatchCount:

        # Observes the workspace
        robot.observe()

        # Picks an object with the specified color
        obj_found = robot.pickObject(object_offset,color=ObjectColor.RED)
        
        # If no object is found, wait for 0.1 seconds and continue
        if not obj_found:
            robot.wait(0.1)
            continue
        
        catchCount +=1

        # Places the object at the destination position
        robot.placeObject(destination_pos)
        print(f"object Count: {catchCount}")
    
    
    # Returns to the observation position
    robot.observe()
    robot.close_connection()


if __name__ == "__main__":
    main()