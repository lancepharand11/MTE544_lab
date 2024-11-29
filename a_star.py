import numpy as np
import matplotlib.pyplot as plt
from math import sqrt


class Node:
    """
        A node class for A* Pathfinding
        parent is parent of the current Node
        position is current position of the Node in the maze
        g is cost from start to current Node
        h is heuristic based estimated cost for current Node to end Node
        f is total cost of present node i.e. :  f = g + h
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

# This function return the path of the search


def return_path(current_node, maze):
    path = []
    num_rows, num_columns = np.shape(maze)
    # here we create the initialized result maze with -1 in every position
    result = [[-1 for i in range(num_columns)] for j in range(num_rows)]
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    # Return reversed path as we need to show from start to end path
    path = path[::-1]
    start_value = 0
    # we update the path of start to end found by A-star serch with every step incremented by 1
    for i in range(len(path)):
        result[path[i][0]][path[i][1]] = start_value
        start_value += 1

    return path

def compute_heuristic(node, end, euclidean_dist=True):
    if euclidean_dist:
        return sqrt((node.position[0] - end.position[0]) ** 2 +
                        (node.position[1] - end.position[1]) ** 2)
    else: # manhattan dist
        return np.abs(node.position[0] - end.position[0]) + \
                    np.abs(node.position[1] - end.position[1])


def search(maze, start, end, euclidean_dist=True):

    print("searching ....")

    maze = maze.T

    """
        Returns a list of tuples as a path from the given start to the given end in the given maze
        :param maze:
        :param cost
        :param start:
        :param end:
        :param euclidean_dist: decides whether euclidean dist or manhattan dist is used for heuristic calcs
        :return:
    """

    # DONE PART 4 Create start and end node with initized values for g, h and f
    # Use None as parent if not defined
    end_node = Node(position=end)
    end_node.g = np.inf       # set a large value if not defined
    end_node.h = compute_heuristic(end_node, end_node, euclidean_dist=euclidean_dist)
    end_node.f = end_node.g + end_node.h

    start_node = Node(position=start) # start and end are already tuples of (x, y)
    start_node.g = 0.0    # cost from start Node
    start_node.h = compute_heuristic(start_node, end_node, euclidean_dist=euclidean_dist)
    start_node.f = start_node.g + start_node.h


    # Initialize both yet_to_visit and visited dictionary
    # in this dict we will put all node that are yet_to_visit for exploration.
    # From here we will find the lowest cost node to expand next
    yet_to_visit_dict = {}  # key is the position (tuple), value is the node
    # in this list we will put all node those already explored so that we don't explore it again
    # key is the position (tuple), value is True (boolean)
    visited_dict = {}

    # Add the start node
    yet_to_visit_dict[start_node.position] = start_node

    # Adding a stop condition. This is to avoid any infinite loop and stop
    # execution after some reasonable number of steps
    outer_iterations = 0
    max_iterations = (len(maze) // 2) ** 10

    # DONE: Check these, dont know if right. PART 4 what squares do we search . serarch movement is left-right-top-bottom
    # (4 or 8 movements) from every positon
    # move = [[...],  # go up
    #         [...],  # go left
    #         [...],  # go down
    #         [...],  # go right
    #         [...],  # go up left
    #         [...],  # go down left
    #         [...],  # go up right
    #         [...]]  # go down right

    # 4 moves -- Assuming it would be [x, y] for 2D maze
    # move = [[0, -1],   # Up
    #         [-1, 0],   # Left
    #         [0, 1],    # Down
    #         [1, 0]]    # Right
    
    # # 8 moves --  update comments
    move = [[0, -1],   # Up
            [-1, 0],   # Left
            [0, 1],    # Down
            [1, 0],    # Right
            [-1, -1],  # Up-Left
            [-1, 1],   # Down-Left
            [1, -1],   # Up-Right
            [1, 1]]    # Down-Right

    """
        1) We first get the current node by comparing all f cost and selecting the lowest cost node for further expansion
        2) Check max iteration reached or not . Set a message and stop execution
        3) Remove the selected node from yet_to_visit dict and add this node to visited dict
        4) Perofmr Goal test and return the path else perform below steps
        5) For selected node find out all children (use move to find children)
            a) get the current postion for the selected node (this becomes parent node for the children)
            b) check if a valid position exist (boundary will make few nodes invalid)
            c) if any node is a wall then ignore that
            d) add to valid children node list for the selected parent
            
            For all the children node
                a) if child in visited dict then ignore it and try next node
                b) calculate child node g, h and f values
                c) if child in yet_to_visit dict then ignore it
                d) else move the child to yet_to_visit dict
    """
    # DONE PART 4 find maze has got how many rows and columns
    num_rows, num_columns = np.shape(maze)

    # Loop until you find the end

    while len(yet_to_visit_dict) > 0:

        # Every time any node is referred from yet_to_visit list, counter of limit operation incremented
        outer_iterations += 1

        # Get the current node with the lowest f value
        current_node = None
        current_fscore = None
        for position, node in yet_to_visit_dict.items():
            if current_fscore is None or node.f < current_fscore:
                current_fscore = node.f
                current_node = node

        # if we hit this point return the path such as it may be no solution or
        # computation cost is too high
        if outer_iterations > max_iterations:
            print("giving up on pathfinding too many iterations")
            return return_path(current_node, maze)

        # Pop current node out off yet_to_visit dict, add to visited list
        yet_to_visit_dict.pop(current_node.position)
        visited_dict[current_node.position] = True

        # test if goal is reached or not, if yes then return the path
        if current_node == end_node:
            return return_path(current_node, maze)

        # Generate children from all adjacent squares
        children = []

        for new_position in move:
            # DONE PART 4 Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # DONE PART 4 Make sure within range (check if within maze boundary)
            # Check if within range (maze boundaries)
            if (node_position[0] > (num_rows - 1) or node_position[0] < 0 or
                    node_position[1] > (num_columns - 1) or node_position[1] < 0):
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] > 0.8:
                continue

            # directions = move.copy()

            # # Loop through all surrounding positions
            # for i in range(3):
            #     for direction in directions * i:
            #         neighbor_row = node_position[0] + direction[0]
            #         neighbor_col = node_position[1] + direction[1]

            #         # Check if the neighbor is within the maze bounds
            #         if neighbor_row < 0 or neighbor_row >= num_rows or neighbor_col < 0 or neighbor_col >= num_columns:
            #             continue  # Skip this neighbor if it's out of bounds

            #         # Make sure it's walkable terrain
            #         if maze[neighbor_row][neighbor_col] > 0.8:
            #             continue  # Skip this neighbor if it's not walkable

            # Create new node
            new_node = Node(parent=current_node, position=node_position)
            children.append(new_node)

        # Loop through children
        for child in children:
            # DONE PART 4 Child is on the visited dict (use get method to check if child is in visited dict, if not found then default value is False)
            if visited_dict.get(child.position, False):
                continue

            # DONE PART 4 Create the f, g, and h values
            # Diagonal movements have higher cost (sqrt(2))
            if abs(child.position[0] - current_node.position[0]) == 1 and \
               abs(child.position[1] - current_node.position[1]) == 1:
                movement_cost = sqrt(2)
            else:
                movement_cost = 1

            # directions = move.copy()
            # cost = 1

            # # Loop through all surrounding positions
            # for i in range(3):
            #     for direction in directions:
            #         neighbor_row = child.position[0] + direction[0] * i
            #         neighbor_col = child.position[1] + direction[1] * i

            #         # Make sure it's walkable terrain
            #         if maze[neighbor_row][neighbor_col] > 0.8:
            #             cost = 5

            # Heuristic costs calculated here, this is uses eucledian dist or manhattan dist
            child.g = current_node.g + movement_cost
            child.h = compute_heuristic(child, end_node, euclidean_dist=euclidean_dist)
            child.f = child.g + (child.h)

            # Child is already in the yet_to_visit list and g cost is already lower
            existing_node = yet_to_visit_dict.get(child.position)
            if existing_node and child.g >= existing_node.g:
                continue

            # Add the child to the yet_to_visit list
            yet_to_visit_dict[child.position] = child
    
