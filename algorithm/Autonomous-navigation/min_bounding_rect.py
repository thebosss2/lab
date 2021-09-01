from numpy import *
from sys import maxsize
from Point import get_x_dimension, get_y_dimension,Point


def create_point_angle(points):
    points_angles = []
    for index in range(len(points) - 1):
        next_point = points[index + 1]
        current_point = points[index]
        point_difference = [next_point.x - current_point.x, next_point.y - current_point.y]
        # Calculate edge angles   atan2(y/x)
        point_angle = math.atan2(point_difference[1], point_difference[0])
        # Check for angles in 1st quadrant
        points_angles.append(abs(point_angle) % (math.pi / 2))
    return unique(array(points_angles))


def get_rectangle_area(angle_point, points_2d):
    # Create rotation matrix to shift points to baseline
    # R = [ cos(theta)      , cos(theta-PI/2)
    #       cos(theta+PI/2) , cos(theta)     ]

    rotation_matrix = array([[math.cos(angle_point), math.cos(angle_point - (math.pi / 2))],
                             [math.cos(angle_point + (math.pi / 2)), math.cos(angle_point)]])
    # Apply this rotation to convex hull points
    convex_hull_point = dot(rotation_matrix, transpose(points_2d))  # 2x2 * 2xn
    min_x = nanmin(convex_hull_point[0], axis=0)
    max_x = nanmax(convex_hull_point[0], axis=0)
    min_y = nanmin(convex_hull_point[1], axis=0)
    max_y = nanmax(convex_hull_point[1], axis=0)
    # Calculate height/width/area of this bounding rectangle
    width = max_x - min_x
    height = max_y - min_y
    area = width * height
    return area, width, height, min_x, max_x, min_y, max_y


# https://geidav.wordpress.com/2014/01/23/computing-oriented-minimum-bounding-boxes-in-2d/


def find_bounding_box(points):
    points_angles = create_point_angle(points)
    # Test each angle to find bounding box with smallest area
    min_bbox = (0, maxsize, 0, 0, 0, 0, 0, 0)  # rot_angle, area, width, height, min_x, max_x, min_y, max_y
    points_2d = vstack((get_x_dimension(points), get_y_dimension(points))).T
    for angle in points_angles:
        area, width, height, min_x, max_x, min_y, max_y = get_rectangle_area(angle, points_2d)
        if area < min_bbox[1]:
            min_bbox = (angle, area, width, height, min_x, max_x, min_y, max_y)
    # Re-create rotation matrix for smallest rect
    point_angle = min_bbox[0]
    R = array([[math.cos(point_angle), math.cos(point_angle - (math.pi / 2))],
               [math.cos(point_angle + (math.pi / 2)), math.cos(point_angle)]])
    min_x = min_bbox[4]
    max_x = min_bbox[5]
    min_y = min_bbox[6]
    max_y = min_bbox[7]
    # Calculate corner points and project onto rotated frame
    corner_points = zeros((4, 2))  # empty 2 column array
    corner_points[0] = dot([max_x, min_y], R)
    corner_points[1] = dot([min_x, min_y], R)
    corner_points[2] = dot([min_x, max_y], R)
    corner_points[3] = dot([max_x, max_y], R)
    return [Point(point[0], point[1], 0) for point in
            corner_points]  # rot_angle, area, width, height, center_point, corner_points
