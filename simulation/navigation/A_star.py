# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : A_star.py
# @Time           : 2024-08-03 15:06:43
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A star Navigation algorithm
"""


import matplotlib.pyplot as plt
import networkx as nwx
import numpy as np
from matplotlib.patches import Rectangle

# from .visualize_utils import AreaBounds, plot_rectangle
class AreaBounds:

    def __init__(self, start, goal, obstacles_bounds):
        obstacles_bounds_arr = np.array(obstacles_bounds)
        if obstacles_bounds_arr.size == 0:
            self.x_min = min([start[0], goal[0]]) - 2
            self.y_min = min([start[1], goal[1]]) - 2
            self.x_max = max([start[0], goal[0]]) + 2
            self.y_max = max([start[1], goal[1]]) + 2
        else:
            self.x_min, self.y_min, _, _ = np.min(obstacles_bounds_arr, axis=0)
            _, _, self.x_max, self.y_max = np.max(obstacles_bounds_arr, axis=0)
            self.x_min = min([self.x_min, start[0], goal[0]]) - 2
            self.y_min = min([self.y_min, start[1], goal[1]]) - 2
            self.x_max = max([self.x_max, start[0], goal[0]]) + 2
            self.y_max = max([self.y_max, start[1], goal[1]]) + 2


def plot_rectangle(x_min, y_min, x_max, y_max):  # pragma: no cover
    width = x_max - x_min
    height = y_max - y_min
    rect = Rectangle(
        (x_min, y_min), width, height, edgecolor="black", facecolor="black"
    )
    plt.gca().add_patch(rect)
    
def visualize_nkx_graph(graph):
        """
        Visualize a 2D grid graph using matplotlib.

        Parameters:
        graph (networkx.Graph): The graph to be visualized.
        """
        # Set positions for each node in the grid
        pos = {(x, y): (y, -x) for x, y in graph.nodes()}  # Position nodes in a grid layout

        # Plot the graph
        plt.figure(figsize=(8, 8))
        nwx.draw(graph, pos,
                with_labels=True,
                node_color='lightblue',
                node_size=200,
                font_size=2,
                font_color='black',
                font_weight='bold',
                edge_color='gray')

        plt.title("2D Grid Graph")
        plt.show()
        
def visualize_static_map(static_map):
    """
    Visualize a static occupancy map using matplotlib.

    Parameters:
    static_map (2D list or numpy array): The occupancy map to be visualized.
    """
    static_map = np.array(static_map)
    plt.figure(figsize=(8, 8))
    plt.imshow(static_map.T, origin='lower', cmap='Greys', alpha=0.7)
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Static Occupancy Map')
    plt.show()

class AStarPlanner:
    """AStar Navigation planner"""

    def __init__(
        self,
        robot_size,
        obstacles_bounds,
        resolution,
        x_max=6,
        y_max=6.2,
        enable_plot=False,
    ):
        self.robot_size = robot_size
        self.obstacles_bounds = obstacles_bounds
        self.resolution = resolution
        self.x_max = x_max
        self.y_max = y_max
        self.enable_plot = enable_plot

    # use A* algorithm to find a Manhattan path
    def plan(self, start_pose, goal_pose):
        """Find a path from a specified initial position to a goal position in a 2D grid representation

        Args:
            init_base_position (Pose): The initial position of the robot.
            goal_base_pose (Pose): The goal pose of the robot.

        Returns:
            The function returns a list of waypoints that form a path from the initial position to the goal position.
            Each waypoint is a list [x, y] representing a position in the world coordinates.

        Note:
            A* for path searching in map: 10 meter * 10 meter
            resolution is 0.1 meter
            a 2D grid: 1 for obstacles
        """

        self.path = None

        # only care about x, y
        self.start_position = start_pose.get_position()[0:2]
        self.goal_position = goal_pose.get_position()[0:2]

        self.area = AreaBounds(
            self.start_position, self.goal_position, self.obstacles_bounds
        )

        # grid to world coords
        def to_grid_coordinates(point, resolution):
            return [
                int(round((coordinate + max_val) / resolution))
                for coordinate, max_val in zip(point, [self.x_max, self.y_max])
            ]

        # world to grid coords
        def to_world_coordinates(point, resolution):
            return [
                (coordinate * resolution) - max_val
                for coordinate, max_val in zip(point, [self.x_max, self.y_max])
            ]

        # Defining the environment size (in meters) and resolution
        size_x = 2 * self.x_max
        size_y = 2 * self.y_max
        n_points_x = int(size_x / self.resolution)
        n_points_y = int(size_y / self.resolution)

        # Create a 2D grid representing the environment
        self.static_map = np.zeros((n_points_x, n_points_y))

        # Compute the number of grid cells to inflate around each obstacle
        inflate_cells = int(round(self.robot_size / 2 / self.resolution))
        print("number of inflated cells: ", inflate_cells)

        # get occupancy map
        for obstacle_bound in self.obstacles_bounds:
            for i in range(
                max(
                    0,
                    int((obstacle_bound[0] + self.x_max) / self.resolution)
                    - inflate_cells,
                ),
                min(
                    n_points_x,
                    int((obstacle_bound[2] + self.x_max) / self.resolution)
                    + inflate_cells,
                ),
            ):
                for j in range(
                    max(
                        0,
                        int((obstacle_bound[1] + self.y_max) / self.resolution)
                        - inflate_cells,
                    ),
                    min(
                        n_points_y,
                        int((obstacle_bound[3] + self.y_max) / self.resolution)
                        + inflate_cells,
                    ),
                ):
                    self.static_map[i][j] = 1

        start_grid = to_grid_coordinates(self.start_position, self.resolution)
        goal_grid = to_grid_coordinates(self.goal_position, self.resolution)
        
        # print("start_grid: ", start_grid)
        # print("goal_grid: ", goal_grid)
        # visualize_static_map(self.static_map)
        
        
        # return True
    
        # Make sure the positions are within the environment and not on the table
        assert (
            0 <= start_grid[0] < n_points_x and 0 <= start_grid[1] < n_points_y
        ), "Initial base position is out of boundary!"
        assert (
            0 <= goal_grid[0] < n_points_x and 0 <= goal_grid[1] < n_points_y
        ), "Goal base position is out of boundary!"
        assert (
            self.static_map[start_grid[0], start_grid[1]] != 1
        ), "Initial base position is in an obstacle!"
        assert (
            self.static_map[goal_grid[0], goal_grid[1]] != 1
        ), "Goal base position is in an obstacle!"

        # Convert the numpy grid map to NetworkX graph
        graph = nwx.grid_2d_graph(n_points_x, n_points_y)
        for i in range(n_points_x):
            for j in range(n_points_y):
                if self.static_map[i, j] == 1:
                    graph.remove_node((i, j))
        # visualize_nkx_graph(graph)

        # return True
        
        # A* star algorithm
        self.path = nwx.astar_path(graph, tuple(start_grid), tuple(goal_grid))

        # print("path:{}".format(path))

        # Convert grid coordinates back to world coordinates
        self.path = [
            to_world_coordinates(point, self.resolution) for point in self.path
        ]
        # print('raw path:{}'.format(path))

        if self.enable_plot:
            self.visual()

        return self.path

    def visual(self):
        """
        Visualization of routes generated by A_star navigation algorithm
        """

        # clear current figure
        plt.clf()

        for x_min, y_min, x_max, y_max in self.obstacles_bounds:
            plot_rectangle(x_min, y_min, x_max, y_max)

        plt.plot(self.start_position[0], self.start_position[1], "og")
        plt.plot(self.goal_position[0], self.goal_position[1], "xr")

        plt.plot([x for (x, _) in self.path], [y for (_, y) in self.path], "-r")

        # print([self.area.x_min, self.area.x_max, self.area.y_min, self.area.y_max])
        plt.axis([self.area.x_min, self.area.x_max, self.area.y_min, self.area.y_max])
        plt.axis("equal")
        # plt.grid(True)    # grid line
        plt.title("Navigation Visualization")
        plt.pause(0.01)
        plt.show()