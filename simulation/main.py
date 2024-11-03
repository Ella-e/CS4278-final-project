import time
import numpy as np
import pickle
import sys
import os
import pybullet as p
from stretch import *
from navigation.Pose import *
from navigation.slam import *
from navigation.A_star import AStarPlanner
from navigation.RRT import RRTPlanner
from navigation.PRM import PRMPlanner
from navigation.SmoothAstar import pathplanning


from navigation.RobotController import RobotController

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.setGravity(0, 0, -1.81)

def get_joint_index_by_name(robot, joint_name):
    num_joints = p.getNumJoints(robot)
    
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot, i)
        if joint_info[1].decode("utf-8") == joint_name:
            print(f"joint name: {joint_name} found at index {i}")
            return i
    return None  # Return None if the joint name is not found

def get_link_index_by_name(robot, link_name):
    num_joints = p.getNumJoints(robot)  # This gives the number of joints, which is one less than the number of links
    
    for i in range(num_joints):
        link_info = p.getJointInfo(robot, i)
        link_name_in_urdf = link_info[12].decode("utf-8")  # Link name is stored at index 12
        if link_name_in_urdf == link_name:
            print(f"link name: {link_name_in_urdf} found at index {i}")
            return i
    return None  # Return None if the link name is not found


mobot = init_scene(p)
    
forward=0
turn=0
speed=10
up=0
stretch=0
gripper_open=0
roll=0
yaw=0

mobot.get_observation()

# numBody = p.getNumBodies()
# print("Num Bodies: ", numBody)
# Loop through all the bodies and get their info
# for body_id in range(numBody):
#     body_info = p.getBodyInfo(body_id)
#     print("Body info: ", body_info)
#     body_name = body_info[1].decode('utf-8')  # Decode byte string to normal string
#     print(f"Body ID: {body_id}, Name: {body_name}")

nav_obstacles_bounds = simple_slam(mobot, False)
# print("nav_obstacles_bounds: ", nav_obstacles_bounds)

# Test smoothAStar
# goal = (2.8,0.1)
# start = (1.6,-2.8)
# goal = (2,2)
# start = (0,0)
# pathplanning(start=start, end=goal, image_path="slam.jpg", verbose=True)

base_position, base_orientation = p.getBasePositionAndOrientation(mobot.robotId)
start_base_pose = Pose(base_position, base_orientation)
waypoints = [start_base_pose, Pose([1.7,-2.8,0.05], [0, 0, math.pi/2]), Pose([1.7,0.1,0.05], [0, 0, math.pi/2]), Pose([2.85,0.1,0.05], [0, 0, math.pi/2])]

# First phase
robot_size = 0.5 #sim_get_robot_size(mobot) 0.5
nav_planner = AStarPlanner(
    robot_size=robot_size,
    obstacles_bounds=nav_obstacles_bounds,
    resolution=0.05,
    enable_plot=False,
)

path = nav_planner.plan(
    start_pose=waypoints[0], goal_pose=waypoints[1]
)

mobotController = RobotController(mobot, waypoints[0], False)
mobotController.sim_navigate_base(waypoints[1], path)

# Second phase
robot_size_2 = 0.2
nav_planner_2 = AStarPlanner(
    robot_size=robot_size_2,
    obstacles_bounds=nav_obstacles_bounds,
    resolution=0.05,
    enable_plot=False,
)

path_2 = nav_planner_2.plan(
    start_pose=waypoints[1], goal_pose=waypoints[2]
)

# Raise arm to a certain height to avoid collision with bed
timestep = 0.01
p.setJointMotorControl2(mobot.robotId,8,p.POSITION_CONTROL,targetPosition=0.5,force=100)
for _ in range(1):
    p.stepSimulation()
    time.sleep(timestep)

mobotController = RobotController(mobot, waypoints[1], True)
mobotController.sim_navigate_base(waypoints[2], path_2)


# Third phase
path_3 = nav_planner.plan(
    start_pose=waypoints[2], goal_pose=waypoints[3]
)

mobotController = RobotController(mobot, waypoints[2], False)
mobotController.sim_navigate_base(waypoints[3], path_3)

# nav_planner_2 = RRTPlanner(
#     robot_size = robot_size_2,
#     obstacles_bounds = nav_obstacles_bounds,
#     enable_plot = True
# )

# nav_planner_2 = PRMPlanner(
#     robot_size = robot_size_2,
#     obstacles_bounds = nav_obstacles_bounds,
#     enable_plot = True
# )

# Initialise Start Base pose
# base_position, base_orientation = p.getBasePositionAndOrientation(mobot.robotId)
# start_base_pose = Pose(base_position, base_orientation)
# path = nav_planner_2.plan(
#     start_pose=start_base_pose, goal_pose=goal_base_pose_2
# )

# Raise arm to a certain height to avoid collision with bed
# timestep = 0.01
# p.setJointMotorControl2(mobot.robotId,8,p.POSITION_CONTROL,targetPosition=0.6,force=100)
# for _ in range(1):
#     p.stepSimulation()
#     time.sleep(timestep)

# # Navigate to the goal along the path
# mobotController = RobotController(mobot, start_base_pose)
# mobotController.sim_navigate_base(goal_base_pose_2, path)

time.sleep(5)

# print("path:", path)


while (1):
    time.sleep(1./240.)
    keys = p.getKeyboardEvents()
    
    pos, ori = p.getBasePositionAndOrientation(mobot.robotId)
    print("pos_before_transform: ", pos)
    print("ori_before_transform: ", ori)
    
    current_pose = Pose(pos, ori)
    print("pos_after_transform: ", current_pose.get_position())
    print("ori_after_transform_quaternion: ", current_pose.get_orientation("quaternion"))
    print("euler: ", current_pose.get_orientation("euler"))

    for k,v in keys.items():
        # moving
        if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
            turn = -1
        if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
            turn = 0
        if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
            turn = 1
        if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
            turn = 0
        if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_TRIGGERED)):
            forward=1
        if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
            forward=0
        if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_TRIGGERED)):
            forward=-1
        if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
            forward=0

        # lifting
        if (k == ord('z') and (v & p.KEY_WAS_TRIGGERED)):
            up = 1
        if (k == ord('z') and (v & p.KEY_WAS_RELEASED)):
            up = 0
        if (k == ord('x') and (v & p.KEY_WAS_TRIGGERED)):
            up = -1
        if (k == ord('x') and (v & p.KEY_WAS_RELEASED)):
            up = 0

        # stretching
        if (k == ord('a') and (v & p.KEY_WAS_TRIGGERED)):
            stretch = -1
        if (k == ord('a') and (v & p.KEY_WAS_RELEASED)):
            stretch = 0
        if (k == ord('d') and (v & p.KEY_WAS_TRIGGERED)):
            stretch = 1
        if (k == ord('d') and (v & p.KEY_WAS_RELEASED)):
            stretch = 0

        # roll
        if (k == ord('r') and (v & p.KEY_WAS_TRIGGERED)):
            roll = 1
        if (k == ord('r') and (v & p.KEY_WAS_RELEASED)):
            roll = 0
        if (k == ord('f') and (v & p.KEY_WAS_TRIGGERED)):
            roll = -1
        if (k == ord('f') and (v & p.KEY_WAS_RELEASED)):
            roll = 0

        # yaw
        if (k == ord('y') and (v & p.KEY_WAS_TRIGGERED)):
            yaw = 1
        if (k == ord('y') and (v & p.KEY_WAS_RELEASED)):
            yaw = 0
        if (k == ord('h') and (v & p.KEY_WAS_TRIGGERED)):
            yaw = -1
        if (k == ord('h') and (v & p.KEY_WAS_RELEASED)):
            yaw = 0


        # gripper
        if (k == ord('q') and (v & p.KEY_WAS_TRIGGERED)):
            gripper_open = -1
        if (k == ord('q') and (v & p.KEY_WAS_RELEASED)):
            gripper_open = 0
        if (k == ord('e') and (v & p.KEY_WAS_TRIGGERED)):
            gripper_open = 1
        if (k == ord('e') and (v & p.KEY_WAS_RELEASED)):
            gripper_open = 0

    base_control(mobot, p, forward, turn)
    arm_control(mobot, p, up, stretch, roll, yaw)
    gripper_control(mobot, p, gripper_open)
    
    p.stepSimulation()
   
    mobot.get_observation()

