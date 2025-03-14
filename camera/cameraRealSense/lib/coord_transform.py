import cv2
import numpy as np
import realsense
from pyniryo import NiryoRobot, PoseObject, ObjectShape, ObjectColor

# rob = np.array([[0.023154,0.386312],
#     [-0.073051, 0.380406],
#     [0.050374, 0.413582],
#     [0.136866, 0.277010],
#     [0.161289, 0.415949],
#     [0.007427, 0.413384],
#     [0.078531, 0.354994]],dtype=np.float32)

# cam = np.array([[203,200],
#        [222.8,124.3],
#        [220.317, 226.053],
#        [74,262],
#        [187.17, 330],
#        [228.7, 187],
#        [160.05, 236.5]],dtype=np.float32)



# H, status = cv2.findHomography(cam, rob, cv2.RANSAC)
# R = 3.140 P = 1.189 Y = -1.612
# R2 = 0.405 P = 1.540 Y = 2.026
# J4 = 0.012 J5 = -0.761 J6 = 0.118

rob = np.array([[-0.0707, 0.2820, 0.0485],
                [-0.0859, 0.3992, 0.0491],
                [0.0989, 0.4365, 0.0481],
                [0.1279, 0.2945, 0.0527],
                [-0.0289, 0.3381, 0.053],
                [-0.0396, 0.4077, 0.049],
                [0.0848, 0.4016, 0.050],
                [0.0774, 0.3215, 0.051]])

cam = np.array([[-0.0334, -0.1280, 0.49],
                [-0.0902, -0.1135, 0.49],
                [0.0955, 0.0707, 0.45],
                [0.0560, 0.0625, 0.45],
                [0.0153, -0.0738, 0.48],
                [0.0923, 0.0739, 0.47],
                [0.0615, 0.047, 0.45],
                [-0.0164, 0.0184, 0.45]])
# rob_points = [[-0.077, 0.285014, 0.049112],
#                 [-0.72287, 0.401, 0.051442],
#                 [0.128723, 0.4202, 0.051419],
#                 [-0.11476, 0.360312, 0.048222],
#                 [-0.117039,0.427903, 0.050737],
#                 [-0.009058, 0.443317, 0.049443],
#                 [0.100728, 0.431817, 0.049334],
#                 [0.173935,0.428018,0.046943],
#                 [0.137863, 0.383979, 0.052977]]

# cam_points = [[-0.06300, -0.121, 0.48],
#               [0.068, -0.103, 0.482553],
#               [-0.162132, 0.008966, 0.468199],
#               [-0.117524, -0.177476, 0.500301],
#               [-0.043343, -0.162854, 0.496254],
#               [-0.058092, -0.061763, 0.481369],
#               [-0.128266, 0.106635, 0.449682],
#               [-0.117601,0.109094,0.449310],
#               [-0.162588, 0.064136, 0.452070]]

# camera_points = [[-0.200869, 0.00, 0.47529],
#                  [-0.199102, -0.1923, 0.50815],
#                  [-0.06104, -0.1643,0.49572],
#                  [-0.08746,0.02625, 0.46283],
#                  [-0.13988, 0.0989, 0.45964],
#                  [-0.26078, 0.0454, 0.46375],
#                  [-0.16140, 0.00, 0.46680]]

# robot_points = [[-0.0917,0.3258, 0.04558 ],
#                 [0.09982, 0.2856, 0.0503],
#                 [-0.10477,0.4112, 0.04948],
#                 [0.07874,0.433018, 0.04841],
#                 [0.16384, 0.41107, 0.04751],
#                 [0.16096, 0.28559, 0.05704],
#                 [0.08548,0.36348, 0.04808]]


# cameraPoints = np.array(cam,dtype=np.float32)
# robotPoints = np.array(rob,dtype=np.float32)

ret , affine, inl  = cv2.estimateAffine3D(cam,rob)

if not ret:
    print("fail calibration")
    # The rotation/affine part (3x3)
    # R_affine = affine[:, :3]  
    # # The translation vector (3x1)
    # T_affine = affine[:, 3]

# T_camera2robot = np.eye(4,dtype=np.float32)

# T_camera2robot[:3,:4] = affine 

# def pixel_to_robot(u, v, H):
#     pts = np.array([[[u, v]]], dtype=np.float32)  # shape (1,1,2)
#     dst = cv2.perspectiveTransform(pts, H)
#     Xr, Yr = dst[0,0,0], dst[0,0,1]
#     return Xr, Yr

def transform_point_3d(pt_cam, affine):

    pt_cam_hom = np.append(pt_cam, 1)  # [x_c, y_c, z_c, 1]
    pt_robot = affine.dot(pt_cam_hom)  # shape (3,)
    return pt_robot



ipAddress = "10.10.10.10"
robot = NiryoRobot(ipAddress)
robot.calibrate_auto()
robot.update_tool()
robot.clear_collision_detected()

# test = [-0.0917,0.3258, 0.04558,2.758,1.493,-2.382 ]
# newPOse = PoseObject(*test)
# robot.move(robot.inverse_kinematics(newPOse))

observation_pose = PoseObject(0.2282,-0.0128,0.2281,2.809,1.417,-2.232)
obs = robot.inverse_kinematics(observation_pose)
cam = realsense.RealSense()
cam.start()
number = cam.scan()
print(number)
objects = cam.getObjects()

# obs = PoseObject(0.203,-0.250,-0.328,0.223, -0.216, -0.118)
robot.move(obs)
robot.wait(0.5)
for object in objects:
    robot.move(obs)
    robot.release_with_tool()
    camx,camy = (object.cx, object.cy)
    rx, ry, rz = transform_point_3d(np.array([object.Xc,object.Yc,object.Zc]), affine)
    # rX,rY = pixel_to_robot(camx,camy,H)
    # print(rX,rY)
    pose = PoseObject(rx,ry,rz,2.809,1.417,-2.232)
    poseJ = robot.inverse_kinematics(pose)
    print("moving")
    print(poseJ)
    robot.move(poseJ)
    robot.grasp_with_tool()
    pose2 = PoseObject(rx,ry,rz + 0.01,2.809,1.417,-2.232)
    posez = robot.inverse_kinematics(pose2)
    robot.move(posez)
    robot.release_with_tool()

    robot.move(obs)

