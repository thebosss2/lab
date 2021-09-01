from math import tan, radians, atan2, pi
from Line import Line
import Point


class Pizza:
    def __init__(self, center, angle, lines, slices):
        self.center = center
        self.angle = angle
        self.lines = lines
        self.slices = slices


def create_pizza_lines(center, angle):
    lines = []
    for i in range(0, 180 // angle):
        slope = tan(i * radians(angle))
        lines.append(Line().get_line_by_point_and_slope(center, slope))
    return lines


def create_pizza_lines2(center, angle):
    lines = []
    for i in range(0, 360 // angle):
        slope = tan(i * radians(angle))
        lines.append(Line().get_line_by_point_and_slope(center, slope))
    return lines


def create_slices(lines, points, angle):
    lines_amount = len(lines)
    slices = [None] * lines_amount * 2
    center = lines[0].line_intersection(lines[1])
    for point in points:
        point = Point.Point(point.x-center.x, point.y-center.y, point.z)
        at = atan2(point.y, point.x)
        if at >= 0:
            deg = at
        else:
            deg = 2*pi + at
        deg = (int(deg * 360 / (2*pi))+360) % 360
        if not slices[deg // angle]:
            slices[deg // angle] = []
        point = Point.Point(point.x + center.x, point.y + center.y, point.z)
        slices[deg // angle].append(point)

    return slices
