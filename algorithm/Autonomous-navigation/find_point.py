from math import sqrt

from Point import Point


def getOtherPointOnLine(slope, point, dist):
    if slope == 0:
        xValue = point.x + dist
        yValue = point.y

    else:
        dx = (dist / sqrt(1 + (slope * slope)))
        dy = slope * dx
        xValue = point.x + dx
        yValue = point.y + dy

    return Point(xValue, yValue, 0)


def line(p1, p2):
    A = (p1.y - p2.y)
    B = (p2.x - p1.x)
    C = (p1.x * p2.y - p2.x * p1.y)
    return A, B, -C


def intersection(L1, L2):
    D = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return True, Point(x, y, 0)
    else:
        return False, None


def getSlope(p1, p2):
    m = (p2.y - p1.y) / (p2.x - p1.x)
    return m


def distance(a, b):
    return sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def is_between(a, c, b):
    return distance(a, c) + distance(c, b) == distance(a, b)


def get_intersection(edge1, edge2, exitPoint, points):
    slope = getSlope(edge1, edge2)
    dist = max(distance(edge1, exitPoint), distance(edge2, exitPoint))
    slope = -1 / slope
    newPointOnLine = getOtherPointOnLine(slope, exitPoint, dist)
    L1 = line(edge1, edge2)
    L2 = line(exitPoint, newPointOnLine)
    answer, R = intersection(L1, L2)
    if answer:
        print("Intersection detected: (" + str(R.x) + " , " + str(R.y) + ")")
    else:
        print("No single intersection point detected")

    if is_between(edge1, R, edge2):
        print("Intersection point between edges")
    else:
        print("Intersection point not between edges")
        R = exitPoint

    return R
