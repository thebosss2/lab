import math
from numpy.core.umath import square
import numpy as np

import Point


class Line:
    def __init__(self, point1=None, point2=None, slope=None, y_intercept=None):  # two parameters at least
        if point1 is None and point2 is None and slope is None and y_intercept is None:
            return
        self.point1 = point1
        self.point2 = point2
        if slope is None:
            slope = (point2.y - point1.y) / (point2.x - point1.x)
        self.slope = slope
        if y_intercept is None:
            y_intercept = point1.y - slope * point1.x
        self.y_intercept = y_intercept

    def get_distance_to_point(self, point):
        up = abs(self.slope * point.x - point.y + self.y_intercept)
        down = math.sqrt(square(self.slope) + 1)
        distance = up / down
        return distance

    def get_line_by_two_points(self, point1, point2):
        slope = (point2.y - point1.y) / (point2.x - point1.x)
        y_intercept = point1.y - slope * point1.x
        return Line(point1, point2, slope, y_intercept)

    def get_line_by_point_and_y_intercept(self, point, y_intercept):
        slope = (point.y - y_intercept) / point.x
        point2 = Point.Point(0, y_intercept, 0)
        return Line(point, point2, slope, y_intercept)

    def get_line_by_point_and_slope(self, point, slope):
        y_intercept = point.y - point.x * slope
        point2 = Point.Point(3, slope * 3 + y_intercept, 0)
        return Line(point, point2, slope, y_intercept)

    def get_line_by_slope_and_y_intercept(self, slope, y_intercept):
        point1 = Point.Point(0, 0, 0)
        point2 = Point.Point(1, slope + y_intercept, 0)
        return Line(point1, point2, slope, y_intercept)

    def convert_to_points(self):
        return (self.point1.x, self.point1.y), (self.point2.x, self.point2.y)

    def line_intersection(self, line2):
        tuple_line1 = self.convert_to_points()
        tuple_line2 = line2.convert_to_points()
        xdiff = (tuple_line1[0][0] - tuple_line1[1][0], tuple_line2[0][0] - tuple_line2[1][0])
        ydiff = (tuple_line1[0][1] - tuple_line1[1][1], tuple_line2[0][1] - tuple_line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            return None

        d = (det(*tuple_line1), det(*tuple_line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return Point.Point(x, y, 0)

    def get_point_on_line_with_length(self, base, length):
        pt = np.array([self.point2.x - self.point1.x, self.point2.y - self.point1.y], dtype=float)
        pt = pt/np.linalg.norm(pt)
        pt = pt*length
        ptpt = Point.Point(pt[0] + base.x, pt[1] + base.y, 0)
        return ptpt



    def intersection_with_segment(self, line):
        intersection_point = self.line_intersection(line)
        if line.point1.x <= intersection_point.x <= line.point2.x or line.point2.x <= intersection_point.x <= line.point1.x:
            if intersection_point.x != self.point1.x and intersection_point.y != self.point1.y:
                return intersection_point
        return None

    def length(self):
        return math.sqrt((self.point1.x-self.point2.x)**2 + (self.point1.y-self.point2.y)**2)

    def plot_line(self, plt):
        x = np.linspace(min(self.point1.x, self.point2.x), max(self.point1.x, self.point2.x), 100)
        y = self.slope*x + self.y_intercept
        plt.plot(x, y)
