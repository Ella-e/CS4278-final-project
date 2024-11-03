import pybullet as p
import matplotlib.pyplot as plt
import numpy as np

from .visualize_utils import *
    
def get_bounding_box(object_id):
        """
        Retrieve the bounding box of a given object in the PyBullet simulation environment.

        Args:
            object (int / str): The ID or name of the object.

        Returns:
            list: The minimum and maximum x, y, z coordinates of the bounding box.
        """

        # object_id = self.resolve_object_id(object)

        link_ids = [
            i
            for i in range(
                -1, p.getNumJoints(object_id)
            )
        ]

        aabb_bounds = np.array(
            [
                p.getAABB(object_id, link_id)
                for link_id in link_ids
            ]
        )

        min_bounds = aabb_bounds[:, 0, :]
        max_bounds = aabb_bounds[:, 1, :]
        min_x, min_y, min_z = np.min(min_bounds, axis=0)
        max_x, max_y, max_z = np.max(max_bounds, axis=0)

        return [min_x, min_y, min_z, max_x, max_y, max_z]

def get_all_link_bounding_box(object_id):
        """
        Retrieve all bounding box of a given object in the PyBullet simulation environment.

        Args:
            object (int / str): The ID or name of the object.

        Returns:
            list: The minimum and maximum x, y, z coordinates of the bounding box for all object link.
        """

        # object_id = self.resolve_object_id(object)

        link_ids = [
            i
            for i in range(
                -1, p.getNumJoints(object_id)
            )
        ]

        aabb_bounds = [
            p.getAABB(object_id, link_id)
            for link_id in link_ids
        ]

        return aabb_bounds
    
def sim_get_robot_size(robot):
        """Retrieves the maximum size of the robot (arm and base) in meters.

        Returns:
            float: The maximum size of the robot in meters.
        """
        (
            min_x_base,
            min_y_base,
            _,
            max_x_base,
            max_y_base,
            _,
        ) = get_bounding_box(robot.robotId)

        robot_size = max(
            max_x_base - min_x_base,
            max_y_base - min_y_base,
        )

        return robot_size

def simple_slam(robot, enable_plot=False):
    """
    Perform a simple SLAM (Simultaneous Localization and Mapping) operation.

    Args:
        client (pybullet): The pybullet client object.
        robot (object): The robot object.
        enable_plot (bool, optional): Flag to enable or disable plotting of the SLAM visualization. Defaults to False.

    Returns:
        list: A list of obstacle bounds in the format [x_min, y_min, x_max, y_max].
    """
    nav_obstacles_bounds = []
    nav_obstacle_ids = list(range(p.getNumBodies()))
    nav_obstacle_ids.remove(0)  # remove plane
    nav_obstacle_ids.remove(robot.robotId)
    # nav_obstacle_ids.remove(robot.sim_get_base_id())  # remove base
    # nav_obstacle_ids.remove(robot.sim_get_arm_id())  # remove arm

    for object_id in nav_obstacle_ids:
        object_link_bounds = get_all_link_bounding_box(object_id)
        for link_bounds in object_link_bounds:
            nav_obstacles_bounds.append(
                [
                    link_bounds[0][0],
                    link_bounds[0][1],
                    link_bounds[1][0],
                    link_bounds[1][1],
                ]
            )
            
    if enable_plot:
        plt.clf()
        for x_min, y_min, x_max, y_max in nav_obstacles_bounds:
            plot_rectangle(x_min, y_min, x_max, y_max)
        area = AreaBounds(nav_obstacles_bounds)
        plt.axis([area.x_min, area.x_max, area.y_min, area.y_max])
        plt.axis("equal")
        # plt.grid(True)    # grid line
        plt.title("SLAM Visualization")
        plt.pause(0.01)
         # Save the figure as a .jpg file
        # plt.savefig("slam.jpg", format="jpg", dpi=300)  # Adjust dpi for quality
        plt.show()

    return nav_obstacles_bounds