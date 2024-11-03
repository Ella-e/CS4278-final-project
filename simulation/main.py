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
from utils.tools import *

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.setGravity(0, 0, -9.81)

mobot = init_scene(p, mug_random=False)
    
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


total_driving_distance = 0
previous_position, _, _ = get_robot_base_pose(p, mobot.robotId)
current_position = previous_position

constraint = None

navi_flag = False
grasp_flag = False

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

    if gripper_open == 1:
        constraint = attach(21, mobot.robotId, 18)
    elif gripper_open == -1:
        detach(constraint)
        constraint = None
    
    mobot.get_observation()
    
    current_position, _, _ = get_robot_base_pose(p, mobot.robotId)
    total_driving_distance += np.linalg.norm(np.array(current_position) - np.array(previous_position))
    previous_position = current_position

    if navi_flag == False:
        if current_position[0] > 1.6 and current_position[1] > -0.35:
            print("Reached the goal region! Total driving distance: ", total_driving_distance)
            navi_flag = True
        else:
            print("Total driving distance: ", total_driving_distance)
            print("Current position: ", current_position)
    else:
        print("Reached the goal region! Total driving distance: ", total_driving_distance)
    
    
    if grasp_flag == False:
        mug_position = get_mug_pose(p)
        print("Mug position: ", mug_position)

        if mug_position[0] > 3.3 and mug_position[0] < 3.5 \
            and mug_position[1] > -0.17 and mug_position[1] < 0.25 \
            and mug_position[2] > 0.71 and mug_position[2] < 0.75:
            print("Mug is in the drawer!")
            grasp_flag = True
    else:
        print("Mug is in the drawer!")

    ee_position, _, _ = get_robot_ee_pose(p, mobot.robotId)
    print("End-effector position: ", ee_position)
