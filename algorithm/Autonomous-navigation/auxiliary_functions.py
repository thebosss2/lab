#!/usr/bin/python3
from math import pow, sqrt, tan, radians, degrees
from sys import maxsize

from matplotlib.pyplot import plot, show, axis
import numpy as np
from numpy import square, sqrt, isclose, linspace, argsort, vstack, cross, arccos, dot, array
from numpy.linalg import linalg
from pyquaternion import Quaternion
from scipy import stats
from sklearn.cluster import DBSCAN
from sympy import symbols, Eq, solve

from Point import Point, get_x_dimension, get_y_dimension


def create_labels(points, eps, min_samples):
    index = 0
    for label in DBSCAN(eps=eps, min_samples=min_samples).fit(
            vstack((get_x_dimension(points), get_y_dimension(points))).T).labels_:  # 0.0125
        points[index].label = label
        index += 1
    return points


def plot_pizza_with_given_center(points, angle, center):
    leftest_x, rightest_x = find_leftest_and_rightest_x(points, False)
    x = linspace(leftest_x, rightest_x, 100)
    for i in range(0, 180 // angle + 1):
        y = tan(i * radians(angle)) * (x - center.x) + center.y
        plot(x, y, '-r')
    axis('equal')
    show()


def get_original_angle(line1, line2):
    vector_1 = np.array([1, line1.slope], dtype=float)
    vector_2 = np.array([1, line2.slope], dtype=float)

    # it assumes that the destination is in front of you
    unit_vector_1 = vector_1 / linalg.norm(vector_1)
    unit_vector_2 = vector_2 / linalg.norm(vector_2)
    dot_product = dot(unit_vector_1, unit_vector_2)
    angle = degrees(arccos(dot_product))
    return angle


def center_of_mass(points):
    x = 0
    y = 0
    for point in points:
        x += point.x
        y += point.y
    length = len(points)
    return Point(x/length, y/length, 0)


def check_if_point_in_rectangle(point, lines):
    rect_lines = turn_lines_to_rect_lines(lines)
    parallel_lines1, parallel_lines2 = find_parallel_lines(rect_lines)
    if (check_point_between_two_lines(point, parallel_lines1[0], parallel_lines1[1]) and
            check_point_between_two_lines(point, parallel_lines2[0], parallel_lines2[1])):
        return True
    return False


def get_motion_vector_as_numpy_array(start_point, end_point):
    return array([start_point.x - end_point.x,
                  start_point.y - end_point.y])


def center_of_mass(points):
    x = 0
    y = 0
    for point in points:
        x += point.x
        y += point.y
    length = len(points)
    return Point(x/length, y/length,0)


def find_segment_center(segment):
    return Point((segment[0].x + segment[1].x) / 2, (segment[0].y + segment[1].y) / 2, 0)


def find_parallel_lines(rect_lines):
    parallel_to_line0_index = 0
    parallel_lines1 = [rect_lines[0], rect_lines[1]]
    for i in range(1, 4):
        if isclose(rect_lines[i][0], rect_lines[0][0]):  # float changes after same calculation
            parallel_lines1 = [rect_lines[0], rect_lines[i]]
            parallel_to_line0_index = i

    parallel_lines2 = []
    for j in range(1, 4):
        if j != parallel_to_line0_index:
            parallel_lines2.append(rect_lines[j])

    return parallel_lines1, parallel_lines2


def right_angle(angle):
    if angle < 0:
        angle = 180 + (180 + angle)
    return angle


def from_quaternion_to_degree(point):
    yaw_current, pitch_current, roll_current = Quaternion(x=point.qx, y=point.qy,
                                                          z=point.qz,
                                                          w=point.qw).yaw_pitch_roll

    return right_angle(degrees(yaw_current))


def create_frames_by_frame_id(points):
    frames = {}
    for point in points:
        if point.frame_id in frames.keys():
            frames[point.frame_id].append(point)
        else:
            frames[point.frame_id] = [point]
    return frames


def create_frames_by_degree(points):
    frames = {}
    for point in points:
        point_degree = from_quaternion_to_degree(point)
        if point_degree in frames.keys():
            frames[point_degree].append(point)
        else:
            frames[point_degree] = [point]
    return frames


def turn_lines_to_rect_lines(lines):
    rect_lines = []
    for line in lines:
        slope, y_intercept = lin_equ(Point(line[0][0], line[1][0], 0), Point(line[0][1], line[1][1], 0))
        rect_lines.append([slope, y_intercept])

    return rect_lines


def check_point_between_two_lines(point, line1, line2):
    y_intercept1 = line1[1]
    slope1 = line1[0]
    y_intercept2 = line2[1]
    result = - slope1 * point.x + point.y
    if y_intercept2 < result < y_intercept1 or y_intercept1 < result < y_intercept2:
        return True
    return False


def get_farthest_point(point, points):
    farthest_point = None
    max_distance = -1
    for p in points:
        distance = calculate_distance(p, point)
        if distance > max_distance:
            max_distance = distance
            farthest_point = p
    return farthest_point


def get_angle(vector_1, vector_2):
    # it assumes that the destination is in front of you
    unit_vector_1 = vector_1 / linalg.norm(vector_1)
    unit_vector_2 = vector_2 / linalg.norm(vector_2)
    dot_product = dot(unit_vector_1, unit_vector_2)
    angle = int(degrees(arccos(dot_product)))
    clockwise = not (cross(unit_vector_1, unit_vector_2) > 0)
    if angle > 90:
        angle = 180 - angle
        clockwise = not clockwise
        print("angle is greater than 180")
    return angle, clockwise


def distance_between_point_and_segment(x_point, y_point, x_segment_1, y_segment_1, x_segment_2, y_segment_2):
    side_difference_x_1 = x_point - x_segment_1
    side_difference_y_1 = y_point - y_segment_1
    segment_difference_x = x_segment_2 - x_segment_1
    segment_difference_y = y_segment_2 - y_segment_1

    dot = side_difference_x_1 * segment_difference_x + side_difference_y_1 * segment_difference_y
    len_sq = segment_difference_x * segment_difference_x + segment_difference_y * segment_difference_y
    param = -1
    if len_sq != 0:  # in case of 0 length line
        param = dot / len_sq

    if param < 0:
        xx = x_segment_1
        yy = y_segment_1
    elif param > 1:
        xx = x_segment_2
        yy = y_segment_2

    else:
        xx = x_segment_1 + param * segment_difference_x
        yy = y_segment_1 + param * segment_difference_y

    distance_x = x_point - xx
    distance_y = y_point - yy
    return sqrt(distance_x * distance_x + distance_y * distance_y)


# https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment, second best answer
def find_best_segment(point, lines):
    min_distance = maxsize
    min_index = -1
    counter = 0
    for line in lines:
        current_distance = distance_between_point_and_segment(point.x, point.y, line[0][0], line[1][0], line[0][1],
                                                              line[1][1])
        if current_distance < min_distance:
            min_distance = current_distance
            min_index = counter
        counter = counter + 1

    return find_correct_line(lines[min_index]), min_distance


def find_worst_segment(point, lines):
    max_distance = 0
    min_index = -1
    counter = 0
    for line in lines:
        current_distance = distance_between_point_and_segment(point.x, point.y, line[0][0], line[1][0], line[0][1],
                                                              line[1][1])
        if current_distance > max_distance:
            max_distance = current_distance
            min_index = counter
        counter = counter + 1

    return find_correct_line(lines[min_index]), max_distance


# Calc minimum distance from a point and a line segment (i.e. consecutive vertices in a polyline).

def find_correct_line(line):
    x_1 = line[0][0]
    x_2 = line[0][1]
    y_1 = line[1][0]
    y_2 = line[1][1]
    correct_line = ((x_1, y_1), (x_2, y_2))
    return correct_line


"""def minMaxRect(rect):
    minX = min(rect[0].x, rect[1].x, rect[2].x, rect[3].x)
    maxX = max(rect[0].x, rect[1].x, rect[2].x, rect[3].x)
    minY = min(rect[0].y, rect[1].y, rect[2].y, rect[3].y)
    maxY = max(rect[0].y, rect[1].y, rect[2].y, rect[3].y)

    return minX, maxX, minY, maxY"""

"""def getRoomCenterPoint(points, rect):
    minX, maxX, minY, maxY = minMaxRect(rect)

    centerX = (minX + maxX) / 2
    centerY = (minY + maxY) / 2

    center = Point(centerX, centerY, 0)
    center = p_closest(points, center, 1)
    return find_point_index(center, points)"""

"""def getRoomCenterPoint_raw(rect):
    minX, maxX, minY, maxY = minMaxRect(rect)

    centerX = (minX + maxX) / 2
    centerY = (minY + maxY) / 2
    return Point(centerX, centerY, 0)"""


def is_point_in_room(point, rectangle_corner):
    A = rectangle_corner[0]
    B = rectangle_corner[1]
    C = rectangle_corner[2]
    D = rectangle_corner[3]

    f_AB_P = (A.x - B.x) * (point.y - B.y) - (A.y - B.y) * (point.x - B.x)
    f_BC_P = (B.x - C.x) * (point.y - C.y) - (B.y - C.y) * (point.x - C.x)
    f_CD_P = (C.x - D.x) * (point.y - D.y) - (C.y - D.y) * (point.x - D.x)
    f_DA_P = (D.x - A.x) * (point.y - A.y) - (D.y - A.y) * (point.x - A.x)
    f_AB_C = (A.x - B.x) * (C.y - B.y) - (A.y - B.y) * (C.x - B.x)
    f_BC_D = (B.x - C.x) * (D.y - C.y) - (B.y - C.y) * (D.x - C.x)
    f_CD_A = (C.x - D.x) * (A.y - D.y) - (C.y - D.y) * (A.x - D.x)
    f_DA_B = (D.x - A.x) * (B.y - A.y) - (D.y - A.y) * (B.x - A.x)

    if f_AB_P * f_AB_C > 0 and f_BC_P * f_BC_D > 0 and f_CD_P * f_CD_A > 0 and f_DA_P * f_DA_B > 0:
        return True

    return False


def get_index_of_closest_point_to_rectangle(label, labels, points, lines):
    cluster_points = []
    indexes_points = []
    j = 0
    for i in labels:
        if i == label:
            indexes_points.append(j)
            cluster_points.append(points[j])
        j += 1
    min_distances = []
    for point in cluster_points:
        min_distances.append(find_best_segment(point, lines)[1])
    index = indexes_points[int(len(argsort(min_distances)) / 2)]
    return index


def get_index_of_closest_point_to_further_rectangle(points, lines):
    indexes_points = []
    min_distances = []
    for point in points:
        min_distances.append(find_worst_segment(point, lines)[1])
    index = indexes_points[int(len(argsort(min_distances)) / 2)]
    return index


def p_closest(points, originPoint, sizeOfSet=20):
    sorted_points = [Point(point.x, point.y, point.z, point.qx, point.qy, point.qz, point.qw, point.frame_id) for point
                     in points]
    sorted_points.sort(key=lambda SetSize: (SetSize.x - originPoint.x) ** 2 + (SetSize.y - originPoint.y) ** 2,
                       reverse=False)
    if sizeOfSet == 1:
        return sorted_points[0]
    return sorted_points[:sizeOfSet]


def find_point_index(point, points):
    # return exit point index
    epdist = (points[0].x - point.x) * (points[0].x - point.x) + (points[0].y - point.y) * (
            points[0].y - point.y)
    epindex = 0
    i = 0
    for p in points:
        dp = (p.x - point.x) * (p.x - point.x) + (p.y - point.y) * (p.y - point.y)
        if dp < epdist:
            epdist = dp
            epindex = i

        i += 1

    return epindex


def find_y_intercept(line, epsilon, reverse):
    y_intercept = symbols('y_intercept')
    eq = Eq(((y_intercept - line[1]) / sqrt((line[0]) + 1) ** 2) * (-1 if reverse else 1) - epsilon)
    sol = solve(eq, y_intercept)
    return sol[0]


"""def find_possible_lines(lines, epsilon):
    solution = []
    for line in lines:
        solution.append([find_y_intercept(line, epsilon, False), find_y_intercept(line, epsilon, True)])
    return solution"""


def clean_data(X_x, X_y, zscore):
    z_score_x = stats.zscore(X_x)
    z_score_y = stats.zscore(X_y)
    X_x_temp = []
    X_y_temp = []
    for i in range(len(X_x)):
        if not (z_score_x[i] > zscore or z_score_y[i] > zscore or z_score_x[i] < -zscore or z_score_y[i] < -zscore):
            X_x_temp.append(X_x[i])
            X_y_temp.append(X_y[i])

    return X_x_temp, X_y_temp


def clean_noises(points):
    # TODO: check indexes changes
    clean_points = []
    for point in points:
        if point.label != -1:
            clean_points.append(Point(point.x, point.y, point.z, point.qx, point.qy, point.qz, point.qw, point.label,
                                      point.frame_id))
    return clean_points


def calculate_distance(p1, p2):
    return sqrt(((p2.x - p1.x) ** 2) + ((p2.y - p1.y) ** 2))


def write_points_to_file(file_name_and_path, points):
    with open(file_name_and_path, "w") as file:
        for point in points:
            file.write(str(point.x) + ',' + str(point.z) + ',' + str(point.y) + ',' + str(point.qx)
                       + ',' + str(point.qy) + ',' + str(point.qz) + ',' + str(point.qw) + ',' + str(
                point.frame_id) + '\n')


def calculate_distance_3d(p1, p2):
    return sqrt(((p2.x - p1.x) ** 2) + ((p2.y - p1.y) ** 2) + ((p2.z - p1.z) ** 2))


def lin_equ(point1, point2):
    """Line encoded as l=(x,y)."""
    if point2.x == point1.x:
        slope = maxsize
    else:
        slope = (point2.y - point1.y) / (point2.x - point1.x)
    y_intercept = (point2.y - (slope * point2.x))
    return slope, y_intercept


# TODO: find formula for rectangle sides
def get_rectangle_sides(point_0, point_1, point_2, point_3):
    l1 = lin_equ(point_0, point_1)
    # l2 = lin_equ(point_0, point_2)
    l3 = lin_equ(point_0, point_3)
    l4 = lin_equ(point_1, point_2)
    # l5 = lin_equ(point_1, point_3)
    l6 = lin_equ(point_2, point_3)
    return l1, l3, l4, l6  # rect_lines[0], rect_lines[1], rect_lines[2], rect_lines[3]


def lineMagnitude(x1, y1, x2, y2):
    lineMagnitude = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2))
    return lineMagnitude


def distance_point_from_line(point, line):
    slope = line[0]
    y_intercept = line[1]
    up = abs(slope * point.x - point.y + y_intercept)
    down = sqrt(square(slope) + 1)
    distance = up / down
    return distance


# Calc minimum distance from a point and a line segment (i.e. consecutive vertices in a polyline).
# TODO: simplify
def distance_point_line(px, py, x1, y1, x2, y2):
    # http://local.wasp.uwa.edu.au/~pbourke/geometry/pointline/source.vba
    LineMag = lineMagnitude(x1, y1, x2, y2)

    if LineMag < 0.00000001:
        distance_point_line = 9999
        return distance_point_line

    u1 = (((px - x1) * (x2 - x1)) + ((py - y1) * (y2 - y1)))
    u = u1 / (LineMag * LineMag)

    if (u < 0.00001) or (u > 1):
        # // closest point does not fall within the line segment, take the shorter distance
        # // to an endpoint
        ix = lineMagnitude(px, py, x1, y1)
        iy = lineMagnitude(px, py, x2, y2)
        if ix > iy:
            distance_point_line = iy
        else:
            distance_point_line = ix
    else:
        # Intersecting point is on the line, use the formula
        ix = x1 + u * (x2 - x1)
        iy = y1 + u * (y2 - y1)
        distance_point_line = lineMagnitude(px, py, ix, iy)

    return distance_point_line


def find_center(points):
    center_x = 0
    center_y = 0
    size = 0
    for point in points:
        center_x += point.x
        center_y += point.y
        size += 1
    center_x /= size
    center_y /= size
    return center_x, center_y


def find_real_rectangle_lines(possible_lines, lines, center_x, center_y):
    new_rect_line_c_1 = find_closer_line(lines[0][0], possible_lines[0][0], possible_lines[0][1], center_x, center_y)
    new_rect_line_c_2 = find_closer_line(lines[1][0], possible_lines[1][0], possible_lines[1][1], center_x, center_y)
    new_rect_line_c_3 = find_closer_line(lines[3][0], possible_lines[3][0], possible_lines[3][1], center_x, center_y)
    new_rect_line_c_4 = find_closer_line(lines[2][0], possible_lines[2][0], possible_lines[2][1], center_x, center_y)
    new_rect_c = [new_rect_line_c_1, new_rect_line_c_2, new_rect_line_c_3, new_rect_line_c_4]
    return new_rect_c


def find_closer_line(slope, y_intercept_1, y_intercept_2, x, y):
    up_1 = (-x * slope + y - y_intercept_1)
    if up_1 > 0:
        distance1 = up_1 / sqrt(square(slope) + 1)
    else:
        distance1 = -up_1 / sqrt(square(slope) + 1)
    up_2 = (-x * slope + y - y_intercept_2)
    if up_2 > 0:
        distance2 = up_2 / sqrt(square(slope) + 1)
    else:
        distance2 = -up_2 / sqrt(square(slope) + 1)
    return y_intercept_2 if distance1 > distance2 else y_intercept_1


def find_intersection(a, b, c, d):
    x = (d - c) / (a - b)
    y = a * x + c

    print("(x,y) = ", x, y)
    return x, y


def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        return None

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return Point(x, y, 0)


def find_lines_from_points(new_rect):
    lines = [((new_rect[0].x, new_rect[1].x), (new_rect[0].y, new_rect[1].y)),
             ((new_rect[1].x, new_rect[2].x), (new_rect[1].y, new_rect[2].y)),
             ((new_rect[2].x, new_rect[3].x), (new_rect[2].y, new_rect[3].y)),
             ((new_rect[0].x, new_rect[3].x), (new_rect[0].y, new_rect[3].y))]

    return lines


def get_slope_and_intercept_from_lines(lines):
    new_lines = []
    for line in lines:
        current_m = (line[1][1] - line[1][0]) / (line[0][1] - line[0][0])
        current_y_intercept = line[1][0] - line[0][0] * current_m
        new_lines.append((current_m, current_y_intercept))
    return new_lines


def find_2epsilon(points, corner_points):
    two_epsilon = 0
    for point in points:
        d1 = distance_point_from_line(point, lin_equ(corner_points[0], corner_points[1]))
        d2 = distance_point_from_line(point, lin_equ(corner_points[0], corner_points[3]))
        d3 = distance_point_from_line(point, lin_equ(corner_points[1], corner_points[2]))
        d4 = distance_point_from_line(point, lin_equ(corner_points[2], corner_points[3]))
        min_distance = min(d1, d2, d3, d4)
        two_epsilon = min_distance if min_distance > two_epsilon else two_epsilon
    return two_epsilon


def find_rectangle_points(l1, l3, new_rect_c):
    x1, y1 = find_intersection(l1[0], l3[0], new_rect_c[0], new_rect_c[1])  # the point closest to point 0
    x2, y2 = find_intersection(l1[0], l3[0], new_rect_c[0], new_rect_c[3])  # the point closest to point 1
    # the line between point 1 and 2 is parallel to the line between point 3 and 4
    x3, y3 = find_intersection(l1[0], l3[0], new_rect_c[2], new_rect_c[1])  # the point closest to point 3
    x4, y4 = find_intersection(l1[0], l3[0], new_rect_c[2], new_rect_c[3])  # the point closest to point 2

    new_rect = [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
    return new_rect


def check_results(data, lines):
    print("number of data points:" + str(data.shape[0]))
    sum = 0
    for i in range(data.shape[0]):
        minimum = maxsize
        for j in range(4):
            d = distance_point_line(data[i][0], data[i][1], lines[j][0][0], lines[j][1][0], lines[j][0][1],
                                    lines[j][1][1])

            if d < minimum:
                minimum = d
        sum += square(minimum)

    print("the squared sum is: " + str(sum))
    avr = sum / data.shape[0]
    print("the averaged squared distance from the rectangle: " + str(avr))
    return sum


def find_leftest_and_rightest_x(points, return_point=True):
    max_x = points[0].x
    min_x = points[0].x
    min_point_by_x = points[0]
    max_point_by_x = points[0]
    for point in points:
        if point.x > max_x:
            max_x = point.x
            max_point_by_x = point
        if point.x < min_x:
            min_x = point.x
            min_point_by_x = point
    if return_point:
        return max_point_by_x, min_point_by_x
    else:
        return min_x, max_x


def find_leftest_and_rightest_y(points, return_point=True):
    max_y = points[0].y
    min_y = points[0].y
    min_point_by_y = points[0]
    max_point_by_y = points[0]
    for point in points:
        if point.y > max_y:
            max_y = point.y
            max_point_by_y = point
        if point.y < min_y:
            min_y = point.y
            min_point_by_y = point
    if return_point:
        return max_point_by_y, min_point_by_y
    else:
        return min_y, max_y


def plot_rectangle_points(corner_points):
    plot(corner_points[0].x, corner_points[0].y, 'ro')
    plot(corner_points[1].x, corner_points[1].y, 'ro')
    plot(corner_points[2].x, corner_points[2].y, 'ro')
    plot(corner_points[3].x, corner_points[3].y, 'ro')


def plot_rectangle_lines(lines, bb_or_min):
    print("these are the bounding box lines") if bb_or_min == "bb" else print("these are the min rectangle lines")
    for i in range(4):
        plot(lines[i][0], lines[i][1], c='r')


def plot_rectangle_lines_class(lines, bb_or_min):
    print("these are the bounding box lines") if bb_or_min == "bb" else print("these are the min rectangle lines")
    for line in lines:
        points_x = (line.point1.x, line.point2.x)
        points_y = (line.point1.y, line.point2.y)
        plot(points_x, points_y, c='r')


def plot_pizza(points, angle, corner_points):
    rectangle_center_x, rectangle_center_y = find_center(corner_points)
    leftest_x, rightest_x = find_leftest_and_rightest_x(points)
    x = linspace(leftest_x, rightest_x, 100)
    for i in range(0, 180 // angle + 1):
        y = tan(i * radians(angle)) * (x - rectangle_center_x) + rectangle_center_y
        plot(x, y, '-r')
    axis('equal')
    show()


# A utility function to calculate
# area of triangle formed by (x1, y1),
# (x2, y2) and (x3, y3)
def area_triangle(point1, point2, point3):
    return abs((point1.x * (point2.y - point3.y) +
                point2.x * (point3.y - point1.y) +
                point3.x * (point1.y - point2.y)) / 2.0)


def filter_floor(points):
    lowest_point = points[0].z
    for point in points:
        if lowest_point > point.z:
            lowest_point = point.z
    points_without_floor = []
    for point in points:
        if not isclose(point.z, lowest_point, atol=1.5):
            points_without_floor.append(point)
    return points_without_floor


def get_floor(points, percent):
    lowest_point = points[0].z
    highest_point = points[0].z
    for point in points:
        if point.z < lowest_point:
            lowest_point = point.z

        if point.z > highest_point:
            highest_point = point.z
    threshold = (percent / 100.0) * (highest_point - lowest_point) + lowest_point
    floor = []
    for point in points:
        if point.z < threshold:
            floor.append(point)
    return floor


def filter_points_in_rectangle(rectangle_corner, points, exit_mode, filter_eps=1):
    filtered_points = []
    A = (area_triangle(rectangle_corner[0], rectangle_corner[1], rectangle_corner[2]) +
         area_triangle(rectangle_corner[0], rectangle_corner[3], rectangle_corner[2]))
    min_side = 100000
    for i in range(0, 4):
        current_side = calculate_distance(rectangle_corner[i % 4], rectangle_corner[(i + 1) % 4])
        if current_side < min_side:
            min_side = current_side

    if exit_mode:
        atol = min_side * filter_eps
    else:
        atol = min_side * 2 / 3

    print("Min side atol: " + str(atol))

    for point in points:
        # Calculate area_triangle of triangle PAB
        A1 = area_triangle(point, rectangle_corner[0], rectangle_corner[1])
        # Calculate area_triangle of triangle PBC
        A2 = area_triangle(point, rectangle_corner[1], rectangle_corner[2])
        # Calculate area_triangle of triangle PCD
        A3 = area_triangle(point, rectangle_corner[2], rectangle_corner[3])
        # Calculate area_triangle of triangle PAD
        A4 = area_triangle(point, rectangle_corner[0], rectangle_corner[3])
        # Check if sum of A1, A2, A3
        # and A4 is same as A
        triangle_sum = A1 + A2 + A3 + A4
        if not isclose(A, triangle_sum, atol=atol) or A == 0:  # 1 for exit
            filtered_points.append(point)

    amount_filtered_points = len(filtered_points)

    filtered_percent = amount_filtered_points * 100 / len(points)
    print("Percent of filtered points:" + str(filtered_percent))
    return filtered_points, filtered_percent


def filter_points_in_rectangle_new(rectangle_corner, points, lines, exit_mode, filter_eps=1):
    filtered_points = []

    min_side = maxsize
    for i in range(0, 4):
        current_side = calculate_distance(rectangle_corner[i % 4], rectangle_corner[(i + 1) % 4])
        if current_side < min_side:
            min_side = current_side

    if exit_mode:
        atol = min_side * filter_eps
    else:
        atol = min_side * 2 / 3 * filter_eps

    print("Min side atol: " + str(atol))

    for point in points:
        if not check_if_point_in_rectangle(point, lines) and find_best_segment(point, lines)[1] > atol:
            filtered_points.append(point)

    amount_filtered_points = len(filtered_points)

    filtered_percent = amount_filtered_points * 100 / len(points)
    print("Percent of filtered points:" + str(filtered_percent))
    return filtered_points, filtered_percent
