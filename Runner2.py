#!/usr/bin/env python3

import cv2
import numpy as np
from math import sqrt, cos, sin, pi as PI

BOARD_H = 300
BOARD_W = 400
BOARD_O = 30

GRID_H = BOARD_H * 2
GRID_W = BOARD_W * 2
GRID_O = int(360 / BOARD_O)

DEG2RAD = PI / 180

# Board Obstacles
quads = [[36.53, 124.38, 48, 108, 170.87, 194.04, 159.40, 210.42],
         [200,280,200,230,210,230,210,280],
         [210,280,210,270,230,270,230,280],
         [210,240,210,230,230,230,230,240]]
elips = [[90, 70, 35, 35],
         [246, 145, 60, 30]]


def quad_check(x0, y0, quad):
    # extract coords out of quad list
    x1 = quad[0]
    y1 = quad[1]
    x2 = quad[2]
    y2 = quad[3]
    x3 = quad[4]
    y3 = quad[5]
    x4 = quad[6]
    y4 = quad[7]

    # check if the point is within the restricted half-plane side of each line
    chk1 = line_check(x0, y0, x1, y1, x2, y2, False, False)
    chk2 = line_check(x0, y0, x2, y2, x3, y3, False, True)
    chk3 = line_check(x0, y0, x3, y3, x4, y4, True, True)
    chk4 = line_check(x0, y0, x4, y4, x1, y1, True, False)

    # check if point is within resitected half place side of all lines --> in object
    if chk1 and chk2 and chk3 and chk4:
        return False  # point is in obstacle space
    else:
        return True  # point is not in obstacle space


def line_check(x0, y0, x1, y1, x2, y2, UD, LR):
    # UD = True  if object is bottom side of line
    # UD = False if object is top    side of line
    # LR = True  if object is left   side of line
    # LR = False if object is right  side of line
    if x2 != x1:  # not vertical line
        m = (y2 - y1) / (x2 - x1)  # get the slope
        b = y1 - m * x1  # get the intercept
        # check if point is within the restriced half-plane
        if (y0 >= m * x0 + b and not UD) or (y0 <= m * x0 + b and UD):
            return True  # True means point is within the restriced half-plane
        else:
            return False  # False means point is not within the restriced half-plane

    else:  # x2 == x1 --> vertical line
        if (x0 >= x1 and not LR) or (x0 <= x1 and LR):
            return True  # True means point is within the restriced half-plane
        else:
            return False  # False means point is not within the restriced half-plane


def elip_check(x0, y0, elip):
    # extract dimensions out of elip list
    xc = elip[0]
    yc = elip[1]
    a2 = elip[2]**2  # horizontal dimension
    b2 = elip[3]**2  # vertical dimension

    if (x0 - xc)**2 / a2 + (y0 - yc)**2 / b2 <= 1:  # check if point is within ellipse
        return False  # point is in obstacle space
    else:
        return True  # point is not in obstacle space
    

def setup_graph(robot_radius, clearance, point_robot = True):
    obst = np.ones((BOARD_H,BOARD_W))
    for x in range(BOARD_W):
        for y in range(BOARD_H):
            for quad in quads:  # check quads
                if not quad_check(x, y, quad):  # see if point is near the quad
                    obst[BOARD_H-y, x] = 0
                    break
    
            for elip in elips:  # check elips
                if not elip_check(x, y, elip):  # see if point is near the elip
                    obst[BOARD_H-y, x] = 0
                    break
                
    if not point_robot:  # used to override the expansion for the vizualization step
        return obst
                       
    newObst = np.ones((BOARD_H,BOARD_W))  # create new obstacle array that will have r
    r = robot_radius + clearance  # point robot radius
    for x in range(BOARD_W):
        for y in range(BOARD_H):
            for i in range(x-r, x+r):  # window each pixel and check for an obstacle in radius
                for j in range(y-r, y+r):
                    if i >= 0 and j >= 0 and i < BOARD_W and j < BOARD_H:  # makes sure point is within bounds
                        if obst[j, i] == 0 and sqrt((x-i)**2+(y-j)**2) < r:  # if window point is in obstacle
                            newObst[y, x] = 0
                            break
                else:
                    continue
                break
            
    return newObst            


# A single state of the maze's search
class MazeVertexNode(object):

    # The parent MazeVertexNode to this instance
    parent = None

    # A 3-tuple, (y,x,t) coordinate pair, where t is an orientation theta
    position = None

    # The current tentative distance from the start to this node
    distG = None

    # The current tentative distance from this node to the goal, given some heuristic
    distF = None

    # Constructor, given all values
    def __init__(self, parent, position, distG, distF):
        self.parent = parent
        self.position = position
        self.distG = distG
        self.distF = distF

    # 
    @staticmethod
    def resort_queue_idx(nodes, low, high, f):
        if high < low:
            return -1
        idx = (high + low) // 2
        currF = nodes[idx].distF
        if idx == (len(nodes) - 1):
            if currF >= f:
                return idx + 1
            else:
                # look left in the array
                return MazeVertexNode.resort_queue_idx(nodes, low, idx-1, f)
        elif idx == 0:
            if currF <= f:
                return 0
            else:
                # look right in the array
                return MazeVertexNode.resort_queue_idx(nodes, idx+1, high, f)
        else:
            if (currF >= f) and (nodes[idx+1].distF <= f):
                return idx + 1
            elif f > currF:
                # look left in the array
                return MazeVertexNode.resort_queue_idx(nodes, low, idx-1, f)
            else:
                # look right in the array
                return MazeVertexNode.resort_queue_idx(nodes, idx+1, high, f)

# 
class Maze(object):

    # The DiscreteGraph representation of the maze
    obst = None

    # Build the graph with the list of semi-algebraic models
    def __init__(self, robot_radius, clearance):
        self.obst = setup_graph(robot_radius, clearance)  # creates the obstacle space. 0 for obstacle, 1 for open space

    # Determine if a coordinate pair is in a traversable portion of the maze
    def is_in_board(self, j, i):
        sh = self.obst.shape
        return (j >= 0) and (i >= 0) and (j < sh[0]) and (i < sh[1]) and (self.obst[j,i] == 1)

    # Calculate the distance between two points
    def dist(self, n1, n2):
        return sqrt((n2[1]-n1[1])**2 + (n2[0]-n1[0])**2)

    # Calculate the tentative remaining distance from n to goal, given this heuristic
    def h(self, n, goal):
        return self.dist(n, goal)

    # Run A* between a start and goal point, using a forward step length
    def astar(self, start, goal, step):
        start = start[0]*2, start[1]*2, start[2]
        goal = goal[0]*2, goal[1]*2, goal[2]
        remaining_nodes = []
        node_indices = {}
        for j in range(GRID_H):
            for i in range(GRID_W):
                if self.is_in_board(int(j/2), int(i/2)):
                    for o in range(GRID_O):
                        v = (j,i,o)
                        if v != start:
                            n = MazeVertexNode(None, v, 999999999, 999999999)
                            remaining_nodes.append(n)
                            node_indices[v] = n
            print("  - building initial priority queue for A*: {0}/{1}".format(j, GRID_H), end="\r")
        print()
        start_node = MazeVertexNode(None, start, 0, self.h(start, goal))
        remaining_nodes.append(start_node)
        node_indices[start] = start_node

        # Track the nodes that were visited in the order they were, to visualize later
        nodes_visited = []

        # Start the main part of the algorithm, tracking the node that can be used to recover the path
        final_node = None
        pxidx = 0
        while (final_node is None) and (len(remaining_nodes) != 0):
            # Essentially, mark this node as "visited" and capture its position
            node = remaining_nodes.pop(-1)
            np = node.position

            # Check if this is the goal position
            if self.dist(np, goal) <= 1.5*step:
                final_node = node
                continue

            # Track the neighbors of this node that were explored
            neighbors_explored = []

            # Get each of the neighbors of this node by looping through the five possible actions
            nj, ni, orientation = np
            for o in range(orientation - 2, orientation + 3):
                # Given the orientation 'coordinate', calculate theta and put in the bounds [0, 360)
                ori = (o + GRID_O) % GRID_O
                theta = ori * BOARD_O * DEG2RAD
                jj = int(nj + (step * sin(theta)))
                ii = int(ni + (step * cos(theta)))
                neighbor = (jj,ii,ori)

                neighbor_node = node_indices.get(neighbor, None)
                if neighbor_node is None:
                    # This node was already visited and removed, continue to the next neighbor
                    continue

                # Add the position of this neighbor to visualize later
                neighbors_explored.append((ii, jj))

                # Calculate the adjusted distance
                node_distG = node.distG + self.dist(np, neighbor)
                if node_distG < neighbor_node.distG:
                    # Set this node as this neighbor's shortest path
                    remaining_nodes.remove(neighbor_node)
                    neighbor_node.distG = node_distG
                    neighbor_node.distF = node_distG + self.h(neighbor, goal)
                    neighbor_node.parent = node
                    # Do a less costly sort by simply moving the neighbor node in the list
                    # If early on in the program, just search from right to left
                    new_idx = None
                    if len(nodes_visited) < 100000:
                        new_idx = len(remaining_nodes) - 1
                        while True:
                            if remaining_nodes[new_idx].distF >= neighbor_node.distF:
                                new_idx = new_idx + 1
                                break
                            new_idx = new_idx - 1
                    else:
                        new_idx = MazeVertexNode.resort_queue_idx(remaining_nodes, 0, len(remaining_nodes), neighbor_node.distF)
                        if new_idx == -1:
                            raise ValueError("Could not sort node.")
                    remaining_nodes.insert(new_idx, neighbor_node)

            # Add this position as having visited each of these neighbors
            nodes_visited.append(((ni, nj), neighbors_explored))
            pxidx = pxidx + 1
            print("Visited {0}, {1} remain".format(pxidx, len(remaining_nodes)), end="\r")

        # If there's no path, the final_node will be null, but nodes_visited could still have content
        return final_node, nodes_visited

def main():
    # Capture required user input
    s = None
    try:
        s_str = input("Enter the start position: ")
        s_comma = s_str.index(",")
        s = int(s_str[:s_comma]), int(s_str[s_comma+1:]), 0
    except:
        print("Please enter the start position in \"y,x\" format, where y and x are integers.")
        return

    g = None
    try:
        g_str = input("Enter the goal position: ")
        g_comma = g_str.index(",")
        g = int(g_str[:g_comma]), int(g_str[g_comma+1:]), 0
    except:
        print("Please enter the goal position in \"y,x\" format, where y and x are integers.")
        return

    robot_radius = None
    try:
        r_str = input("Enter the robot radius: ")
        robot_radius = int(r_str)
    except:
        print("Please enter the robot radius as an integer.")
        return
    if robot_radius <= 0:
        print("Please enter the robot radius as a positive integer.")
        return

    clearance = None
    try:
        c_str = input("Enter the clearance: ")
        clearance = int(c_str)
    except:
        print("Please enter the clearance as an integer.")
        return
    if clearance < 0:
        print("Please enter the clearance as a non-negative integer.")
        return

    step = None
    try:
        t_str = input("Enter the robot movement step (integer between 1 and 10, inclusive): ")
        step = int(t_str)
    except:
        print("Please enter an integer.")
        return
    if (step < 1) or (step > 10):
        print("Please enter an integer between 1 and 10, inclusive.")
        return

    vid_name = input("Enter the name of the output file (no file extension, ex. 'output1'): ")

    # Build the maze and underlying graph object
    print("Starting maze generation...")
    maze = Maze(robot_radius, clearance)
    # Check if they're traversable positions in the maze, continue if so
    if maze.is_in_board(s[0],s[1]) and maze.is_in_board(g[0],g[1]):
        # Do Dijkstra
        print("Done. Planning path...")
        path_node, nodes_visited = maze.astar(s,g,step)
        print("Done (visited {0} positions). Starting render...".format(len(nodes_visited)))
        # Build video writer to render the frames at 120 FPS
        vid_write = cv2.VideoWriter(
            "{0}.mp4".format(vid_name),
            cv2.VideoWriter_fourcc(*'mp4v'),
            120.0,
            (BOARD_W, BOARD_H)
        )
        # Build image to be white and draw the obstacles
        temp = np.uint8(setup_graph(robot_radius, clearance, point_robot = False))
        temp *= 255
        img = np.empty((BOARD_H,BOARD_W,3), dtype=np.uint8)
        img[:, :, 0] = temp
        img[:, :, 1] = temp
        img[:, :, 2] = temp
        img[s[:2]] = (0,255,0)
        img[g[0]-2:g[0]+2,g[1]-2:g[1]+2] = (0,0,255)

        # Go through the pixels visited
        for conn in nodes_visited:
            src = conn[0]
            for dest in conn[1]:
                img = cv2.line(
                    img,
                    (int(src[0]/2), int(src[1]/2)),
                    (int(dest[0]/2), int(dest[1]/2)),
                    (255,0,0),
                    1
                )
                vid_write.write(img)
        # Draw the final path
        img = cv2.line(
            img,
            (int(path_node.position[1]/2), int(path_node.position[0]/2)),
            (int(path_node.parent.position[1]/2), int(path_node.parent.position[0]/2)),
            (0,255,0),
            1
        )
        for i in range(10):
            vid_write.write(img)
        path_node = path_node.parent
        while path_node.parent is not None:
            img = cv2.line(
                img,
                (int(path_node.position[1]/2), int(path_node.position[0]/2)),
                (int(path_node.parent.position[1]/2), int(path_node.parent.position[0]/2)),
                (0,255,0),
                1
            )
            for i in range(10):
                vid_write.write(img)
            path_node = path_node.parent
        vid_write.release()
        print("Finished.")
    else:
        print("Either the start {0} or the goal {1} was not valid.".format(s, g))

if __name__ == "__main__":
    main()
