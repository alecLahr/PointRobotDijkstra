#!/usr/bin/env python3

import abc
import cv2
from math import sqrt

# Board dimensions, in millimeters as well as pixels
BOARD_H = 300
BOARD_W = 400

# The base semi-algebraic model
class BaseSemiAlgebraicModel(abc.ABC):

    # The (y,x) 2-tuple coordinates
    position = None

    # Capture the y,x position
    def __init__(self, position):
        self.position = position

    # Break the position and given coordinates as a 4-tuple
    def get_coords(self, coord):
        return self.position[0], self.position[1], coord[0], coord[1]

    # Check if any of the given models contain the given coordinate
    @staticmethod
    def any_contains(coord, models):
        return any(m.contains(coord) for m in models)

    # Check if this model contains the given coordinate
    @abc.abstractmethod
    def contains(self, coord):
        return False

    # Draw this shape on the given image
    @abc.abstractmethod
    def draw_on(self, img):
        return None

# A model for a rectangle
class RectangleModel(BaseSemiAlgebraicModel):

    # The height and width of the rectangle
    h = None
    w = None

    # Take the position to be the top-left corner of the rectangle, as well as its height and width
    def __init__(self, position, h, w):
        super().__init__(position)
        self.h = h
        self.w = w

    # Overriden
    def contains(self, coord):
        py, px, cy, cx = self.get_coords(coord)
        return (cx >= px) and (cx <= px+self.w) and (cy >= py) and (cy <= py+self.h)

    # Overriden
    def draw_on(self, img):
        return cv2.rectangle(
            img,
            (self.position[1], self.position[0]),
            (self.position[1]+self.w, self.position[0]+self.h),
            (0,0,0),
            -1
        )

# A model for a square
class SquareModel(RectangleModel):

    # Uses the rectangle model but assumes width==height
    def __init__(self, position, s):
        super().__init__(position, s, s)

# A model for a circle
class CircleModel(BaseSemiAlgebraicModel):

    # The radius of the circle
    r = None

    # Take the position to be the center of the circle, as well as the radius
    def __init__(self, position, r):
        super().__init__(position)
        self.r = r

    # Overriden
    def contains(self, coord):
        py, px, cy, cx = self.get_coords(coord)
        return (cx-px)**2 + (cy-py)**2 <= self.r**2

    # Overriden
    def draw_on(self, img):
        return cv2.circle(
            img,
            (self.position[1], self.position[0]),
            self.r,
            (0,0,0),
            -1
        )

# A model for an ellipse
class EllipseModel(BaseSemiAlgebraicModel):

    # The radius for each of the x and y axes
    rx = None
    ry = None

    # Take the position to be the center of the ellipse, as well as the radii
    def __init__(self, position, rx, ry):
        super().__init__(position)
        self.rx = rx
        self.ry = ry

    # Overriden
    def contains(self, coord):
        k, h, cy, cx = self.get_coords(coord)
        rx2 = self.rx**2; ry2 = self.ry**2
        return (ry2*(cx-h)**2) + (rx2*(cy-k)**2) <= rx2*ry2

    # Overriden
    def draw_on(self, img):
        return cv2.ellipse(
            img,
            (self.position[1], self.position[0]),
            (self.rx, self.ry),
            0,
            0,
            360,
            (0,0,0),
            -1
        )

# A representation of the traversable portions of the maze
class DiscreteGraph(object):

    # The square root of 2, to reuse in NEIGHBOR_DISPLACEMENTS
    R2 = sqrt(2)

    # An 8-tuple of 2-tuples, describing the (y,x) displacements and total distance to each neighbor
    NEIGHBOR_DISPLACEMENTS = (
        (-1, -1, R2), (-1, +0, +1), (-1, +1, R2),
        (+0, -1, +1),               (+0, +1, +1),
        (+1, -1, R2), (+1, +0, +1), (+1, +1, R2)
    )

    # A dict, where the key is a vertex, and the value is a list of 2-tuples:
    # - A vertex is itself a 2-tuple (y,x), where the y and x is the pixel
    #   position / millimeter displacement from the top left corner of the board
    # - For the 2-tuple elements:
    #   - The first element is another vertex that the vertex key can connect to
    #   - The second element is the distance between these two vertices, it's
    #     calculated here once and only once to save time
    edges = None

    # Build the graph with the given obstacles
    def __init__(self, obstacles):
        self.build(obstacles)

    # Build the graph with the given obstacles
    def build(self, obstacles):
        # Clear the set of edges
        self.edges = dict()

        # Loop through all the pixels / each millimeter to create the edges by
        # visiting each pixel's (at most) eight neighbors
        # A neighbor is valid if it's within the bounds of the maze and is not
        # inside of any of the given obstacles
        rh = range(BOARD_H); rw = range(BOARD_W)
        for j in rh:
            for i in rw:
                v = (j,i)
                self.edges[v] = list()
                for dj, di, dd in DiscreteGraph.NEIGHBOR_DISPLACEMENTS:
                    jj = j + dj; ii = i + di
                    if (jj in rh) and (ii in rw) and (not BaseSemiAlgebraicModel.any_contains((jj,ii), obstacles)):
                        self.edges[v].append(((jj,ii), dd))

# A maze object that uses a DiscreteGraph instance with some other utilities
class Maze(object):

    # The DiscreteGraph representation of the maze
    graph = None

    # Build the graph with the list of semi-algebraic models
    def __init__(self, obstacles):
        self.graph = DiscreteGraph(obstacles)

    # Determine if a coordinate pair is in a traversable portion of the maze
    def is_in_board(self, position):
        return position in self.graph.edges
