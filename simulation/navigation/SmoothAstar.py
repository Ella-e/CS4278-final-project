import matplotlib.pyplot as plt
import pyfmm
import time
import cv2
import numpy as np
import imutils
import random

np.float = float
np.bool = bool

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
        # new distance cost
        self.dc = 0

    def __eq__(self, other):
        return self.position == other.position


def convert2list(img):
    height, width = img.shape
    maze = np.zeros((height, width), np.uint8)
    for i in range(width):
        for j in range(height):
            maze[j][i] = 1 if img[j][i] > 0 else 0

    return maze.tolist()


def img2binList(img, lenWidth, GRID_SIZE=50, verbose=1):
    global DISTANCECOSTMAP
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    _, gray = cv2.threshold(gray, 112, 255, cv2.THRESH_BINARY_INV)
    if verbose:
        showmaze = cv2.resize(gray, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_NEAREST)
        cv2.imshow("img", showmaze)
        cv2.waitKey(0)

    cnts = cv2.findContours(gray.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    locs = []

    height, width = gray.shape
    tmp = np.zeros((height, width), np.uint8)

    idxLargest = 0
    areaLargest = 0
    # loop over the contours
    for (i, c) in enumerate(cnts):
        # compute the bounding box of the contour, then use the
        # bounding box coordinates to derive the aspect ratio
        (x, y, w, h) = cv2.boundingRect(c)
        if w * h > areaLargest:
            idxLargest = i
            areaLargest = w * h
        cv2.rectangle(tmp, (x, y), (x + w, y + h), (255, 0, 0), 2)

    if verbose:
        # print("found largest contour outline")
        showmaze = cv2.resize(tmp, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_NEAREST)
        cv2.imshow("img", showmaze)
        cv2.waitKey(0)

    # print("cropping image as largest contour")
    (x, y, w, h) = cv2.boundingRect(cnts[idxLargest])
    gray = gray[y:y + h, x:x + w]

    if verbose:
        showmaze = cv2.resize(gray, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_NEAREST)
        cv2.imshow("img", showmaze)
        cv2.waitKey(0)

    mapWidth = (int)(lenWidth // GRID_SIZE)
    mapHeight = (int)((h / w) * lenWidth // GRID_SIZE)
    print("the map will be created by the size: " + str(mapWidth) + " X " + str(mapHeight))

    resized_gray = imutils.resize(gray, width=mapWidth)  # resize the map for convolution
    _, resized_gray = cv2.threshold(resized_gray, 1, 255, cv2.THRESH_BINARY)
    if verbose:
        showmaze = cv2.resize(resized_gray, None, fx=4.7, fy=4.7, interpolation=cv2.INTER_NEAREST)
        cv2.imshow("img", showmaze)
        cv2.waitKey(0)
    maze = convert2list(resized_gray)
    my_maze = np.array(maze)
    solution = pyfmm.march(my_maze == 1, batch_size=100000)[0] # NOTE : white area means walkable area
    DISTANCECOSTMAP = solution

    # cv2.destroyAllWindows()
    return maze


def distcost(x, y, safty_value=2):
    # large safty value makes the path more away from the wall
    # However, if it is too large, almost grid will get max cost
    # which leads to eliminate the meaning of distance cost.
    max_distance_cost = np.max(DISTANCECOSTMAP)
    distance_cost = max_distance_cost-DISTANCECOSTMAP[x][y]
    #if distance_cost > (max_distance_cost/safty_value):
    #    distance_cost = 1000
    #    return distance_cost
    return 50 * distance_cost # E5 223 - 50


def random_walkable_position(x_range, y_range, rigidity=1):
    # High rigidity means smaller area will be determined as walkable plane
    walkable_limit = np.max(DISTANCECOSTMAP)/rigidity
    while True:
        x = random.randrange(x_range)
        y = random.randrange(y_range)
        if distcost(x, y)/50 < walkable_limit:
            break
    return (x, y)


def walkable_plane_list(x_range, y_range, rigidity=1.8):
    # Creating the list of positions which are on walkable plane
    walkable_plane = []
    walkable_limit = np.max(DISTANCECOSTMAP)/rigidity
    for i in range(x_range):
        for j in range(y_range):
            if distcost(i, j)/50 < walkable_limit:
                walkable_plane.append((i, j))
    return walkable_plane


def convert2meter(path, scale=0.2):
    """convert the path in meter scale"""
    """in general, one grid represent 0.5 meter"""
    path_list = [list(elem) for elem in path]
    metered_path = []
    for grid in path_list:
        metered_grid = [i * scale for i in grid]
        metered_path.append(metered_grid)
    return metered_path

def visualize_maze(maze):
    """
    Visualize the binary maze using Matplotlib, including the x and y indices of the cells.
    
    Parameters:
    maze (list of lists): A 2D list representing the binary maze, where 0 represents a walkable cell and 1 represents an obstacle.
    """
    # Create a numpy array from the maze list
    maze_arr = np.array(maze)
    
    # Create a figure and axis
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Plot the maze
    ax.imshow(maze_arr, cmap='binary')
    
    # Add the x and y indices as labels
    for x in range(maze_arr.shape[1]):
        for y in range(maze_arr.shape[0]):
            ax.text(x, y, f'({x}, {y})', ha='center', va='center', color='w', fontsize=8)
    
    # Remove axis ticks
    ax.set_xticks([])
    ax.set_yticks([])
    
    # Add a title
    ax.set_title('Visualization of Binary Maze with Indices')
    
    # Show the plot
    plt.show()

def astar(maze, start, end):

    """Returns a list of tuples as a path from the given start to the given end in the given maze"""
    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    global checked_positions
    checked_positions = []
    open_list = []
    closed_list = []
    # Check if start or end node is on the obstacle
    if maze[start[0]][start[1]] == 1:
        print("Start node is not walkable terrain")
    if maze[end[0]][end[1]] == 1:
        print("End node is not walkable terrain")
    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:
        # Get the current node
        # Refresh the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal node
        if current_node == end_node:
            path = []
            current = current_node
            # accumulate parents nodes to draw the path
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position (8 neighborhoods)
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Avoid infinite loop by checking closed list
            if Node(current_node, node_position) in closed_list:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)
            checked_positions.append(new_node.position)
        # Loop through children
        for child in children:
            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    break
            else:
                # Create the f, g, and h values
                # child.g = (current_node.g + ((child.position[0]-current_node.position[0])**2+(child.position[1]-current_node.position[1])**2))
                child.g = current_node.g + 1
                child.h = (((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2))
                # New cost 'distance cost' as dc
                # The weight of the distance cost has been set to make the path at least 3 grid away from the obstacles.
                child.dc = 5*distcost(child.position[0], child.position[1])
                child.f = child.g + child.h + child.dc

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g >= open_node.g:
                    break
            else:
                # Add the child to the open list
                open_list.append(child)


def pathplanning(start, end, image_path, verbose=False):
    # Running Time Check
    starttime = time.time()

	# Convert map image to binary list
    img = cv2.imread(image_path)
    maze = img2binList(img, lenWidth=3580, GRID_SIZE=20, verbose=0) #cm, 1000 for E5-223 lobby 3580
    # visualize_maze(maze)
    
    # Start and End point setting

    print("Start =", start, '\n', "End =", end)

    # Procedure Checking
    print(" ", "Path planning Proceeding...", " ")

    path = astar(maze, start, end)
    print("Path planning Succeed")
    print("time :", time.time() - starttime)

    if verbose:
        # Print generated Path (in grid scale and meter scale)
        print("Path : ", path)
        print("Meter scale Path : ", convert2meter(path))

        # Visualizing binary map and generated path
        showmaze = np.array(maze).astype(np.uint8)
        showmaze *= 255
        showmaze = np.stack((showmaze,)*3, axis=-1)
        num_of_searched_node = 0
        """
        for walkable in walkable_plane_list(100, 100):          # checking walkable plane
        showmaze[walkable[0]][walkable[1]] = 60
        """
        for searched in checked_positions:
            showmaze[searched[0]][searched[1]] = [40, 40, 40]
        for colorpath in path:
            showmaze[colorpath[0]][colorpath[1]] = [200, 50, 200]
            num_of_searched_node += 1
        print(num_of_searched_node)

        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                showmaze[start[0] - i][start[1] - j] = [0, 254, 0]
                showmaze[end[0] - i][end[1] - j] = [0, 0, 254]
        showmaze = cv2.resize(showmaze, None, fx=7, fy=7, interpolation=cv2.INTER_NEAREST)
        cv2.imshow('A* algorithm run with distance cost', showmaze)
        cv2.waitKey(0)
        plt.imshow(DISTANCECOSTMAP, interpolation='None')
        plt.colorbar()
        plt.title('DISTANCECOSTMAP')
        plt.show()
        plt.close()  # press 'Q' to exit

    return path

# if __name__ == '__main__':
#     start = (100, 55)
#     end = (30, 144) # (45,33) green sofa (87,76) desk (70, 115) tree (75, 160) dosirak (100,144) gs
#     pathplanning(start, end, image_path="lobby.jpg", verbose=True)