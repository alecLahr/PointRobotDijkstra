#!/usr/bin/env python3

import abc
import cv2

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
