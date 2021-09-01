from scipy.spatial import ConvexHull

from auxiliary_functions import *
from clean_data_dbscan import create_labels
from exit import *
from min_bounding_rect import find_bounding_box


def find_farest_point(index_point, points):
    max = -1
    max_index = 0
    for i in range(len(points)):
        if index_point != i:
            d = calculate_distance(points[i], points[index_point])
            if d > max:
                max = d
                max_index = i

    return points[max_index]


def find_two_closest_points_from_point(index_point, points):
    two_closest_points = []
    min = 1000
    min_index = 0
    for i in range(len(points)):
        if index_point != i:
            d = calculate_distance(points[i], points[index_point])
            if d < min:
                min = d
                min_index = i
    two_closest_points.append(points[min_index])
    points.remove(points[min_index])

    min = 1000
    min_index = 0
    for i in range(len(points)):
        if index_point != i:
            d = calculate_distance(points[i], points[index_point])
            if d < min:
                min = d
                min_index = i
    two_closest_points.append(points[min_index])
    points.remove(points[min_index])

    return two_closest_points[0], two_closest_points[1]


def get_square_distances_from_rectangle(points, corner_points):
    sum = 0
    for point in points:
        d1 = distance_point_from_line(point, lin_equ(corner_points[0], corner_points[1]))
        d2 = distance_point_from_line(point, lin_equ(corner_points[0], corner_points[3]))
        d3 = distance_point_from_line(point, lin_equ(corner_points[1], corner_points[2]))
        d4 = distance_point_from_line(point, lin_equ(corner_points[2], corner_points[3]))
        min_distance = min(d1 ** 2, d2 ** 2, d3 ** 2, d4 ** 2)
        sum += min_distance
    return sum


def get_best_rectangle(points, eps, min_samples):
    min, eps, down = decrement_or_increment_eps(points, eps, get_distances_sum(points, eps, min_samples), min_samples)
    min_eps = eps
    for i in range(50):
        sum_distances = get_distances_sum(points, eps, min_samples)
        if sum_distances < min:
            min_eps = eps
            min = sum_distances
            eps += 0.0001 * (-1 if down else 1)
        else:
            break
    return min_eps


def check_if_there_are_labels(points):
    for point in points:
        if point.label != -1:
            return True
    return False


def get_best_rectangle_by_min_samples(points):
    min_cost = 10000000
    min_samples_of_min_cost = -1
    for i in range(2, 31, 2):  # pay attention that jumpings of 1 are probably al little bit more accurate
        x = get_distances_sum_min_samples(points, i)
        if x < min_cost:
            min_cost = x
            min_samples_of_min_cost = i
    print("min samples: ", min_samples_of_min_cost)
    return min_samples_of_min_cost


def get_min_eps(points):
    return ConvexHull(vstack((get_x_dimension(points), get_y_dimension(points))).T).area / len(points)


def get_distances_sum(points, eps, min_samples):
    points = create_labels(points, eps, min_samples)
    clean_points = clean_noises(points)
    corner_points = [Point(point[0], point[1], 0) for point in find_bounding_box(clean_points)]
    sum_distances = get_square_distances_from_rectangle(points, corner_points)
    return sum_distances


def get_distances_sum_min_samples(points, min_samples):
    points = create_labels(points, 0.0125, min_samples)
    clean_points = clean_noises(points)
    corner_points = find_bounding_box(clean_points)
    sum_distances = get_square_distances_from_rectangle(points, corner_points)
    return sum_distances


def decrement_or_increment_eps(points, eps, min, min_samples):
    decrement = True
    eps -= 0.0001
    sum_distances = get_distances_sum(points, eps, min_samples)
    if min < sum_distances:
        eps += 0.0002
        sum_distances = get_distances_sum(points, eps, min_samples)
        decrement = False
    return sum_distances, eps, decrement
