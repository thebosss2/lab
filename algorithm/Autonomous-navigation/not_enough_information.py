from math import isclose


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


def check_point_between_two_lines(point, line1, line2):
    y_intercept1 = line1[1]
    slope1 = line1[0]
    y_intercept2 = line2[1]
    result = - slope1 * point.x + point.y
    if (y_intercept1 > result > y_intercept2) or (y_intercept2 > result > y_intercept1):
        return True
    return False


def check_if_cluster_in_corner(cluster_points, rect_lines):
    counter = 0
    parallel_lines1, parallel_lines2 = find_parallel_lines(rect_lines)
    for point in cluster_points:
        if (((check_point_between_two_lines(point, parallel_lines1[0], parallel_lines1[1])) and
             (not check_point_between_two_lines(point, parallel_lines2[0], parallel_lines2[1])))
                or (not check_point_between_two_lines(point, parallel_lines1[0], parallel_lines1[1])) and
                (check_point_between_two_lines(point, parallel_lines2[0], parallel_lines2[1]))):
            counter += 1
    if counter / len(cluster_points) >= 0.2:
        return True
    return False


def check_if_there_is_entrance(clusters, rect_lines):
    for cluster in clusters:
        if check_if_cluster_in_corner(cluster, rect_lines):
            return True
    return False
