from auxiliary_functions import *


def create_degrees_list(degrees, low, high):
    degrees_low_high = []
    is_cycle = False
    if high < low:
        is_cycle = True
    for degree in degrees:
        if not is_cycle:
            if not (degree in degrees_low_high) and low < degree < high:
                degrees_low_high.append(degree)
        else:
            if not (degree in degrees_low_high) and (degree > low or degree < high):
                degrees_low_high.append(degree)
    return degrees_low_high


def create_array_of_points_by_degrees(degrees, frames):
    points = []
    for key in degrees:
        for point in list(frames[key]):
            if point not in points:
                points.append(point)
    return points


def find_rect_center(rect):
    x_center = 0
    y_center = 0
    for point in rect:
        x_center += point.x
        y_center += point.y
    return x_center / 4, y_center / 4


def nlogn_median(points_with_distance):  # l already sorted by second index
    return points_with_distance[int(0.5 * len(points_with_distance))]


def closest_corner_index(point, corner_points):
    min_distance = maxsize
    closest_index = 0
    for i in range(4):
        d = calculate_distance(point, corner_points[i])
        if min_distance > d:
            min_distance = d
            closest_index = i

    return closest_index


def get_slope(point1, point2):
    return (point1.y - point2.y) / (point1.x - point2.x)


def find_intersection(m, b, n, c):  # intersection of mx+b, nx+c

    x = (c - b) / (m - n)
    y = m * x + b
    return Point(x, y, 0)


def find_rect_point(closest_line, rect):
    index_of_rect = 0
    for point in rect:
        if point.x == closest_line[0] and point.y == closest_line[1]:
            break
        index_of_rect += 1
    return index_of_rect


def find_best_segment_without_fix(point, lines):
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

    return lines[min_index], min_distance


"""def check_if_point_is_between_two_parallel_lines(slope, y_intercept1, y_intercept2, point):
    min = y_intercept1 if y_intercept1<y_intercept2 else y_intercept2
    max = y_intercept1 + y_intercept2 - min
    value = point.y - slope * point.x
    if value>=min and value<=max:
        return True
    return False"""


def check_if_point_in_corner(point, lines):
    rect_lines = turn_lines_to_rect_lines(lines)
    parallel_lines1, parallel_lines2 = find_parallel_lines(rect_lines)
    return (not check_point_between_two_lines(point, parallel_lines1[0], parallel_lines1[1]) and
            not check_point_between_two_lines(point, parallel_lines2[0], parallel_lines2[1]))


# ToDo: fix slope == 0 (in all of the code)
def update_rect_and_lines(rect, point_median, lines):  # update by the point(expand the rect)
    """if check_if_point_in_rectangle(point_median, lines):
        return rect, lines"""
    if check_if_point_in_corner(point_median, lines):
        closest_corner = closest_corner_index(point_median, rect)

        # new sides
        slope1 = get_slope(rect[(closest_corner - 1) % 4], rect[closest_corner])
        slope2 = get_slope(rect[(closest_corner + 1) % 4], rect[closest_corner])

        rect[closest_corner] = point_median

        y_intercept1 = rect[closest_corner].y - slope1 * rect[closest_corner].x
        y_intercept2 = rect[closest_corner].y - slope2 * rect[closest_corner].x

        # old_sides
        slope3 = slope1
        slope4 = slope2
        y_intercept3 = rect[(closest_corner + 1) % 4].y - slope1 * rect[(closest_corner + 1) % 4].x
        y_intercept4 = rect[(closest_corner - 1) % 4].y - slope2 * rect[(closest_corner - 1) % 4].x

        rect[(closest_corner + 1) % 4] = find_intersection(slope2, y_intercept2, slope3, y_intercept3)
        rect[(closest_corner - 1) % 4] = find_intersection(slope1, y_intercept1, slope4, y_intercept4)
    else:
        closest_line = find_best_segment_without_fix(point_median, lines)[0]
        slope_new = (closest_line[1][1] - closest_line[1][0]) / (closest_line[0][1] - closest_line[0][0])
        new_y_intercept = point_median.y - slope_new * point_median.x
        index_of_rect1, index_of_rect2 = find_rect_point((closest_line[0][0], closest_line[1][0]),
                                                         rect), find_rect_point(
            (closest_line[0][1], closest_line[1][1]), rect)
        if slope_new != 0:
            slope_other_side = -1 / slope_new
            index_of_rect1_intercept = rect[index_of_rect1].y - slope_other_side * rect[index_of_rect1].x
            index_of_rect2_intercept = rect[index_of_rect2].y - slope_other_side * rect[index_of_rect2].x
            rect[index_of_rect1] = find_intersection(slope_new, new_y_intercept, slope_other_side,
                                                     index_of_rect1_intercept)
            rect[index_of_rect2] = find_intersection(slope_new, new_y_intercept, slope_other_side,
                                                     index_of_rect2_intercept)
        else:
            rect[index_of_rect1] = Point(rect[index_of_rect1].x, point_median.y, 0)
            rect[index_of_rect2] = Point(rect[index_of_rect2].x,  point_median.y, 0)
    for i in range(4):
        lines[i] = ((rect[i].x, rect[(i + 1) % 4].x), (rect[i].y, rect[(i + 1) % 4].y))
    return rect, lines


def find_frame_id_of_array_points_from_rect_center(points, rect):
    points_with_distance = []
    rect_center_x, rect_center_y = find_rect_center(rect)
    rect_center = Point(rect_center_x, rect_center_y, 0)
    for point in points:
        points_with_distance.append([point, calculate_distance(point, rect_center)])
    points_with_distance.sort(key=lambda point_with_distance: point_with_distance[1])
    point_with_distance_median = nlogn_median(points_with_distance)
    return point_with_distance_median[0], point_with_distance_median[1]


def rect_area(rect):
    return calculate_distance(rect[0], rect[1]) * calculate_distance(rect[1], rect[2])


def expend_rectangle(points, rect, lines):
    frames = create_frames_by_degree(points)

    degrees_350_10 = create_degrees_list(frames.keys(), 340, 20)
    degrees_80_100 = create_degrees_list(frames.keys(), 70, 110)
    degrees_170_190 = create_degrees_list(frames.keys(), 160, 200)
    degrees_260_280 = create_degrees_list(frames.keys(), 250, 290)

    points_350_10 = create_array_of_points_by_degrees(degrees_350_10, frames)
    points_80_100 = create_array_of_points_by_degrees(degrees_80_100, frames)
    points_170_190 = create_array_of_points_by_degrees(degrees_170_190, frames)
    points_260_280 = create_array_of_points_by_degrees(degrees_260_280, frames)

    for degree_change in range(0, 90, 1):
        is_happened = False
        if len(degrees_350_10) > 0:
            point_350_10, frame_id_350_10 = find_frame_id_of_array_points_from_rect_center(points_350_10, rect)
            if is_happened:
                curr_rect, curr_lines = update_rect_and_lines(curr_rect, point_350_10, curr_lines)
            else:
                curr_rect, curr_lines = update_rect_and_lines(rect, point_350_10, lines)
            is_happened = True
        if len(degrees_80_100) > 0:
            point_80_100, frame_id_80_100 = find_frame_id_of_array_points_from_rect_center(points_80_100, rect)
            rect, lines = update_rect_and_lines(rect, point_80_100, lines)
            if is_happened:
                curr_rect, curr_lines = update_rect_and_lines(curr_rect, point_80_100, curr_lines)
            else:
                curr_rect, curr_lines = update_rect_and_lines(rect, point_80_100, lines)
            is_happened = True
        if len(degrees_170_190) > 0:
            point_170_190, frame_id_170_190 = find_frame_id_of_array_points_from_rect_center(points_170_190, rect)
            rect, lines = update_rect_and_lines(rect, point_170_190, lines)
            if is_happened:
                curr_rect, curr_lines = update_rect_and_lines(curr_rect, point_170_190, curr_lines)
            else:
                curr_rect, curr_lines = update_rect_and_lines(rect, point_170_190, lines)
            is_happened = True
        if len(degrees_260_280) > 0:
            point_260_280, frame_id_260_280 = find_frame_id_of_array_points_from_rect_center(points_260_280, rect)
            rect, lines = update_rect_and_lines(rect, point_260_280, lines)
            if is_happened:
                curr_rect, curr_lines = update_rect_and_lines(curr_rect, point_260_280, curr_lines)
            else:
                curr_rect, curr_lines = update_rect_and_lines(rect, point_260_280, lines)
        if rect_area(curr_rect) > rect_area(rect):
            rect = curr_rect
            lines = curr_lines
    return rect, lines
