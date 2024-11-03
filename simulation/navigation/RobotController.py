import math
import time

import numpy as np
import pybullet as p

# from ..Pose import Pose
from .Pose import Pose
from .PIDController import PIDController

class RobotController:
    def __init__(self, mobot, start_pose, isReverse):
        self.robotId = mobot.robotId
        self.target_distance = 0.0
        self.distance_controller = PIDController(
            Kp=0.01,
            Ki=0.0,
            Kd=0.0,
            setpoint=self.target_distance,
        )
        self.start_pose = start_pose
        self.current_base_yaw = self.start_pose.get_orientation("euler")[2]
        self.timestep = 0.00001
        self.base_rotated = False
        self.left_wheel = 1
        self.right_wheel = 2
        self.middle_wheel = 3
        self.maxForce = 1000
        self.wheel_speed = 1
        self.isReverse = isReverse

    def sim_get_current_base_pose(self):
        base_position, base_orientation = p.getBasePositionAndOrientation(
            self.robotId
        )
        return Pose(base_position, base_orientation)
    
    def run(self, x=1):  # steps
        """
        Step the simulation for a given number, robot=None of steps.

        Args:
            x (int): Number of simulation steps to run.
        """
        for _ in range(x):
            p.stepSimulation()
            time.sleep(self.timestep)
    
    def sim_rotate_base_to_target_yaw(
        self, target_yaw, gradual=True, step_size=0.02, delay_time=0.05
    ):
        """
        Rotate base to a specified yaw angle. Can be done gradually or at once.

        Args:
            target_yaw (float): The target yaw angle (in radians) for the base.
            gradual (bool): If True, the rotation is done gradually. Otherwise, it's instant.
            step_size (float, optional): Angle increment for each step in radians. Only used if gradual=True.
            delay_time (float, optional): Delay in seconds after each step. Only used if gradual=True.
        """
        ## Reverse
        if self.isReverse:
            target_yaw = (target_yaw + math.pi) % (2 * math.pi)
        
        def angle_to_quaternion(yaw):
            return [0, 0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

        def shortest_angular_distance(from_angle, to_angle):
            return (to_angle - from_angle + math.pi) % (2 * math.pi) - math.pi

        if gradual:

            angle_diff = shortest_angular_distance(self.current_base_yaw, target_yaw)

            while abs(angle_diff) > step_size:

                if angle_diff > 0:
                    self.current_base_yaw += step_size
                else:
                    self.current_base_yaw -= step_size

                self.current_base_yaw = (self.current_base_yaw + math.pi) % (
                    2 * math.pi
                ) - math.pi
                angle_diff = shortest_angular_distance(
                    self.current_base_yaw, target_yaw
                )
                orientation = angle_to_quaternion(self.current_base_yaw)
                position, _ = p.getBasePositionAndOrientation(
                    self.robotId
                )
                p.resetBasePositionAndOrientation(
                    self.robotId, position, orientation
                )
                self.run()

            # Ensure final orientation is set accurately
            orientation = angle_to_quaternion(target_yaw)
            position, _ = p.getBasePositionAndOrientation(
                self.robotId
            )
            p.resetBasePositionAndOrientation(
                self.robotId, position, orientation
            )
            self.current_base_yaw = target_yaw
            self.run()

        else:
            orientation = angle_to_quaternion(target_yaw)
            position, _ = p.getBasePositionAndOrientation(
                self.robotId
            )
            p.resetBasePositionAndOrientation(
                self.robotId, position, orientation
            )
            self.run(5)
            
    def sim_action(self, output):
        """
        Ajust base position using PID controller's output

        Args:
            output (float): The output of the PID controller, which is used to calculate the new position of the robot's base.
        """
        ## Reverse
        if self.isReverse:
            output = -output
        
        position, orientation = p.getBasePositionAndOrientation(
            self.robotId
        )
        euler_angles = p.getEulerFromQuaternion(
            orientation
        )
        p.resetBasePositionAndOrientation(
            self.robotId,
            [
                position[0] + output * math.cos(euler_angles[2]),
                position[1] + output * math.sin(euler_angles[2]),
                position[2],
            ],
            orientation
        )

    def sim_move_base_to_waypoint(self, waypoint, threshold=0.01):
        """
        Move base to waypoint
        The robot first rotates towards the target, and then moves towards it in a straight line.
        The movement is controlled by a controller (assumed to be a PID controller) that adjusts the velocity of the robot based on the distance to the target.

        Args:
            waypoint (Pose): The target pose (position and orientation) for the robot. This should be an instance of a Pose class, which is assumed to have 'x' and 'y' properties.
        """
        self.next_waypoint = waypoint
        self.target_distance = 0.0
        self.base_rotated = False
        cnt = 1

        while True:
            pose = self.sim_get_current_base_pose()
            target = self.next_waypoint
            x, y = pose.x, pose.y

            distance = math.sqrt((target.y - y) ** 2 + (target.x - x) ** 2)
            if distance < threshold:
                break

            self.distance_controller.set_goal(self.target_distance)
            output = self.distance_controller.calculate(distance)

            yaw = math.atan2(target.y - y, target.x - x)

            if not self.base_rotated:
                self.sim_rotate_base_to_target_yaw(yaw)
                self.base_rotated = True

            self.sim_action(-output)

            if cnt % 20 == 0:
                self.run()

            cnt += 1

        self.run()

    def sim_navigate_base(
            self, goal_base_pose, path, threshold=0.05, enable_plot=False
        ):
            """
            Navigate a robot from its current position to a specified goal position

            Args:
                goal_base_pose (Pose): The target pose (position and orientation) for the robot.
            """
            # for i, waypoint in enumerate(path, start=1):
            for i in range(len(path)):

                next_point = [path[i][0], path[i][1], 0]
                # move to each waypoint
                self.sim_move_base_to_waypoint(
                    Pose([path[i][0], path[i][1], 0], goal_base_pose.get_orientation())
                )

                # draw the trajectory
                if i != 0 and enable_plot:
                    front_point = [path[i - 1][0], path[i - 1][1], 0]
                    p.addUserDebugLine(
                        front_point,
                        next_point,
                        lineColorRGB=[1, 0, 0],
                        lineWidth=3
                    )

            self.sim_rotate_base_to_target_yaw(goal_base_pose.get_orientation("euler")[2])
            self.run(10)
            ik_error = self.sim_calculate_nav_error(goal_base_pose)
            if ik_error >= threshold:
                print(
                    f"[BestMan_Sim][Base] \033[33mwarning\033[0m: The robot base don't reach the specified position! IK error: {ik_error}"
                )
            print("[BestMan_Sim][Base] \033[34mInfo\033[0m: Navigation is done!")

    def sim_calculate_nav_error(self, goal_pose):
        """Calculate the navigation error for performing pick-and-place manipulation of an object using a robot arm.

        Args:
            goal_position: The desired goal position for the target object.
            goal_orientation: The desired goal orientation for the target object.
        """
        current_base_pose = self.sim_get_current_base_pose()
        distance = np.linalg.norm(
            np.array(current_base_pose.get_position())
            - np.array(goal_pose.get_position())
        )
        return distance
