from math import pi, radians, tan

from matplotlib.pyplot import scatter, plot, clf
from numpy import isclose

from Line import Line
from Point import Point, get_x_dimension, get_y_dimension
from auxiliary_functions import lin_equ, find_leftest_and_rightest_y, find_leftest_and_rightest_x, calculate_distance, \
    get_floor, create_frames_by_degree, plot_rectangle_lines_class, plot_rectangle_points, line_intersection
from min_bounding_rect import find_bounding_box


# This is Ripley's K function and is a measure of spatial homogeneity.
# If K(h) > pi()h^2 then the data is apparently clustered"
def K_function(points, A, h):
    N = len(points)
    Lamb = N / A
    I = 0

    for i in range(N - 1):
        for j in range(i + 1, N):
            dij = calculate_distance(points[i], points[j])
            if dij < h:
                I += 1
    return I / (Lamb * N)


# See Noel Cressie - Statistics for Spatial Data (1991)
def are_points_scatterd(points, epsilon, h, A):
    return K_function(points, A, h) < (pi * h * h) * (1 + epsilon)


def fit_points_to_plane(points, m, n, epsilon=0.1):
    # print("line solution is y = {m}x + {n}".format(m=m, n=n))

    fitted_points = []
    for p in points:
        result = (m * p.x) + n
        if isclose(result, p.y, atol=epsilon):
            fitted_points.append(p)

    fitted_percent = len(fitted_points) * 100 / len(points)
    # print("percent of fitted points:" + str(fitted_percent))
    return fitted_points, fitted_percent


def get_close_points_to_line_precent(frame_points, line, epsilon=0.1):
    fitted_points = []
    for point in frame_points:
        if line.get_distance_to_point(point) <= epsilon:
            fitted_points.append(point)
    fitted_percent = len(fitted_points) * 100 / len(frame_points)
    return fitted_points, fitted_percent


def find_central_ray_in_frame(frame_points, line):
    segment_length = calculate_distance(line.point1, line.point2)
    epsilon_to_fit_plane = 0.05  # Todo:check correct epsilon
    fitted_points, fitted_percent = get_close_points_to_line_precent(frame_points, line, epsilon_to_fit_plane)
    if len(fitted_points) < 10:
        return None
    A = segment_length * 2 * epsilon_to_fit_plane
    epsilon_for_ripples = 0.7933
    h1 = 0.10928
    h2 = 0.38298
    return fitted_points if (are_points_scatterd(fitted_points, epsilon_for_ripples, h1, A) or (
        are_points_scatterd(fitted_points, epsilon_for_ripples, h2, A))) else None


def is_corridor_to_room(frame_points, reference_point, points):
    if len(frame_points) == 0:
        return None
    # sort all points according to their distance from current position
    sorted_points = [Point(point.x, point.y, point.z) for point in frame_points]
    sorted_points.sort(key=lambda p: (p.x - reference_point.x) ** 2 + (p.y - reference_point.y) ** 2,
                       reverse=False)

    # get the farthest point
    farthest_point_from_drone = sorted_points[-1]

    # find the linear equation that represent the depth of the frame

    # check which axis represent the depth of the frame
    is_axis_y_means_depth = abs(farthest_point_from_drone.y - reference_point.y) > abs(
        farthest_point_from_drone.x - reference_point.x)
    # print("is axis y means depth: " + str(is_axis_y_means_depth))

    if is_axis_y_means_depth:
        max_point, min_point = find_leftest_and_rightest_x(frame_points)
        epsilon_to_fit_plane = abs(max_point.x - min_point.x)

    else:
        max_point, min_point = find_leftest_and_rightest_y(frame_points)
        epsilon_to_fit_plane = abs(max_point.y - min_point.y)

    # use range of opposite axis to fit points to plane (the depth linear equation)
    epsilon_to_fit_plane /= 3
    m, n = lin_equ(reference_point, farthest_point_from_drone)
    fitted_points, fitted_percent = fit_points_to_plane(sorted_points, m, n, epsilon_to_fit_plane)
    if len(fitted_points) == 0:
        return None

    # Use a statistical model to determine whether the distribution is uniform
    # Todo: Change "Magic numbers" with ML
    closest_point_to_drone = fitted_points[0]
    A = calculate_distance(closest_point_to_drone, farthest_point_from_drone) * 2 * epsilon_to_fit_plane
    epsilon_for_Ripples = 0.5
    h1 = epsilon_to_fit_plane / 200.0
    h2 = calculate_distance(closest_point_to_drone, farthest_point_from_drone) / 200.0
    threshold_percent = 33.3
    floor_points = get_floor(fitted_points, threshold_percent)
    floor_points_amount = len(floor_points)
    if floor_points_amount >= 10 and (are_points_scatterd(floor_points, epsilon_for_Ripples, h1, A) or (
            are_points_scatterd(floor_points, epsilon_for_Ripples, h2, A))):
        # fitted_points_amount = len(fitted_points)

        # percent_on_floor = 100.0 * floor_points_amount / fitted_points_amount
        # print("Found " + str(floor_points_amount) + " points on floor, out of " + str(
        # fitted_points_amount) + " points, " + str(percent_on_floor) + " precent")
        # ("Return points")
        return fitted_points
    else:
        # print("Return None")
        return None


def check_if_point_on_line(point, line):
    return isclose(point.y, point.x * line.slope + line.y_intercept)


def get_points_on_lines(points, lines):
    new_points = []
    for point in points:
        for line in lines:
            if check_if_point_on_line(point, line):
                if point not in new_points:
                    new_points.append(point)
    return new_points


def is_point_between_lines(point_to_check, line1, line2):
    slope = -1 / line1.slope
    y_intercept = point_to_check.y - slope * point_to_check.x

    a = (line1.point1.x, line1.point1.y)
    b = (line1.point2.x, line1.point2.y)
    c = (point_to_check.x, point_to_check.y)
    d = ((point_to_check.x + 1), (point_to_check.x + 1) * slope + y_intercept)
    point_1_intersection = line_intersection((a, b), (c, d))

    a = (line2.point1.x, line2.point1.y)
    b = (line2.point2.x, line2.point2.y)
    point_2_intersection = line_intersection((a, b), (c, d))
    if point_1_intersection is None or point_2_intersection is None:
        return -1

    return calculate_distance(point_1_intersection, point_to_check) + \
           calculate_distance(point_to_check, point_2_intersection) <= \
           calculate_distance(point_1_intersection, point_2_intersection)


# find the middle line between two close pizza lines
def find_lines_from_points_class(points):
    return [Line().get_line_by_two_points(points[i], points[(i + 1) % 4]) for i in range(4)]


def corridor_identification(points):
    frames = create_frames_by_degree(points)
    pizza_lines_in_suspected_points = []
    for frame in frames.keys():
        points = []
        for point in frames[frame]:
            points.append(point)
        corner_points = find_bounding_box(points)
        lines = find_lines_from_points_class(corner_points)
        points_on_lines = get_points_on_lines(points, lines)
        angle_of_pizza = 20
        for point in points_on_lines:
            scatter(get_x_dimension(points), get_y_dimension(points), s=2)
            plot_rectangle_points(corner_points)
            plot_rectangle_lines_class(lines, "bb")
            pizza_lines_in_suspected_points.append(plot_pizza(angle_of_pizza, point))
        for point_on_line in range(len(points_on_lines)):
            temp = None
            points_between_lines = []
            for line in pizza_lines_in_suspected_points[point_on_line]:
                if temp is None:
                    temp = line
                    continue
                if temp.slope == 0:
                    continue
                for point in points:
                    if is_point_between_lines(point, temp, line):
                        points_between_lines.append(point)
                # try to do the pizza lines creation with point and slope and not slope and intercept
                if is_corridor_to_room(points_between_lines, line.point1, points) is not None:
                    pass

        # trails containes all the suspected trails


def corridor_identification_by_frame(frame_points):
    corner_points = find_bounding_box(frame_points)
    lines = find_lines_from_points_class(corner_points)
    points_on_lines = get_points_on_lines(frame_points, lines)
    angle_of_pizza = 25
    max_points = -1
    ray_points = None
    reference_point = None
    farthest_point = None
    for point in points_on_lines:
        pizza_lines = plot_pizza(angle_of_pizza, point)
        previous_line = pizza_lines[0]
        for line in pizza_lines[1:]:
            if previous_line.slope == 0:
                continue
            points_between_lines = []
            for frame_point in frame_points:
                if is_point_between_lines(frame_point, previous_line, line):
                    points_between_lines.append(frame_point)
            # try to do the pizza lines creation with point and slope and not slope and intercept
            if len(points_between_lines) == 0:
                continue
            for rectangle_line in lines:
                intersection_point = line.intersection_with_segment(rectangle_line)
                if intersection_point is not None:
                    line.point2 = intersection_point
                    break
            result_points = find_central_ray_in_frame(points_between_lines, line)
            if result_points is not None:
                points_amount = len(result_points)
                if points_amount > max_points:
                    max_points = points_amount
                    ray_points = result_points
                    reference_point = line.point1
                    farthest_point = line.point2
            previous_line = line
    ray_line_segment = [reference_point, farthest_point]
    return ray_points, ray_line_segment


def plot_pizza(angle, center_point):
    pizza_lines = []
    for i in range(0, 180 // angle + 1):
        slope = tan(radians(i * angle))
        if slope == 0:
            continue
        pizza_lines.append(Line().get_line_by_point_and_slope(center_point, slope))
    return pizza_lines


def plot_frames(points):
    frames = create_frames_by_degree(points)
    plot(points[0].x, points[0].y)
    for frame in frames.keys():
        clf()
        frame_points = []
        for point in frames[frame]:
            frame_points.append(point)
        scatter(get_x_dimension(frame_points), get_y_dimension(frame_points), s=2)
        # show()

# print(create_data("~/Documents/scans_with_full_frames_data/dani/scan_1/pointData33.csv"))
# corridor_identification(points)
