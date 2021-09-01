#!/usr/bin/python3
import os
from time import time

from matplotlib.pyplot import scatter

from Point import create_data, create_date_from_colmap
from adjust_rectangle_from_frame_ids import expend_rectangle
from auxiliary_functions import *
from best_rectangle import get_best_rectangle_by_min_samples
from clean_data_dbscan import create_labels
from entrance import find_filtered_clusters_entrance
from exit import find_filtered_clusters_exit
from join_clusters import exit_by_frame
from min_bounding_rect import find_bounding_box
from not_enough_information import check_if_there_is_entrance


def build_first_rectangle(min_eps_magic, points, is_debug):
    min_samples = get_best_rectangle_by_min_samples(points)

    points = create_labels(points, min_eps_magic, min_samples)
    clean_points_magic = clean_noises(points)
    corner_points_magic = find_bounding_box(clean_points_magic)
    clean_points, corner_points, eps = clean_points_magic, corner_points_magic, min_eps_magic
    lines = find_lines_from_points(corner_points)
    if is_debug:
        scatter(get_x_dimension(points), get_y_dimension(points), linewidth=0.1, s=2)
        plot_rectangle_points(corner_points)
        plot_rectangle_lines(lines, "bb")
        show()

    return points, clean_points, corner_points, eps, lines


def build_expanded_rectangle(points, corner_points, lines, start, is_debug):
    corner_points, lines = expend_rectangle(points, corner_points, lines)
    print("expand rectangle time:", str(time() - start), "seconds")
    rect_lines = turn_lines_to_rect_lines(lines)
    if is_debug:
        scatter(get_x_dimension(points), get_y_dimension(points), linewidth=0.1, s=2)
        plot_rectangle_points(corner_points)
        plot_rectangle_lines(lines, "bb")
        # savefig("./Files_original/" + str(walk) + ".png")
        show()

    return corner_points, lines, rect_lines


def find_best_filter(corner_points, points, lines, is_exit):
    filter_eps = 1
    amount_of_times = 0
    while amount_of_times != 5:
        outside_points, filter_percentage = filter_points_in_rectangle_new(corner_points, points, lines, is_exit,
                                                                           filter_eps)
        if filter_percentage < 0.1:
            filter_eps = filter_eps * 2 / 3
        elif filter_percentage > 17:
            filter_eps = filter_eps * 3 / 2
        else:
            break
        amount_of_times += 1
    return outside_points


def find_clusters_and_exit(corner_points, outside_points, points, eps, lines, rect_lines, is_up_minus, is_exit,
                           is_debug):
    filter_eps = 1
    count_until_startover = 0
    not_enough_information = False
    check_otherwise = False
    while True:
        if count_until_startover >= 5:
            return None, None, None, None, None
        if is_exit:
            exit_points_index, num_of_clusters, clusters = find_filtered_clusters_exit(outside_points, points,
                                                                                       lines,
                                                                                       is_up_minus, is_debug)
        else:
            exit_points_index, num_of_clusters, clusters, labels = find_filtered_clusters_entrance(outside_points,
                                                                                                   points, eps, lines)
        if exit_points_index[0] is None or check_otherwise:
            print("couldn't find clusters. Try again")
            not_enough_information = True
            filter_eps = filter_eps * 2 / 3
            outside_points, filter_percentage = filter_points_in_rectangle_new(corner_points, points, lines, is_exit,
                                                                               filter_eps)
        else:
            break
        there_is_entrance = check_if_there_is_entrance(clusters, rect_lines)
        if there_is_entrance:
            break
        elif exit_points_index[0] is not None and not there_is_entrance:
            check_otherwise = True
        count_until_startover += 1

    print("estimated exits: ", num_of_clusters)
    return exit_points_index, num_of_clusters, clusters, not_enough_information, outside_points


def find_exit_points(points, outside_points, clusters, lines, is_exit, exit_points_index, not_enough_information,
                     is_debug):
    if not_enough_information:
        max = - maxsize + 10
        for cluster_index in range(len(clusters)):
            is_max = len(clusters[cluster_index])
            if is_max > max:
                max_cluster_index = cluster_index
                max = is_max

        closest_point_index = -1
        min_distance = maxsize
        for point in range(len(clusters[max_cluster_index])):
            is_min = find_best_segment(clusters[max_cluster_index][point], lines)[1]
            if is_min < min_distance:
                min_distance = is_min
                closest_point_index = point
        point = clusters[max_cluster_index][closest_point_index]
        points_with_best_segment = [[point, find_best_segment(point, lines)[0]]]
        plot(point.x, point.y, 'ro')
    else:
        points_with_best_segment = []
        for index in exit_points_index:
            point = outside_points[index]
            points_with_best_segment.append([point, find_best_segment(point, lines)[0]])
    if not is_exit:
        points_with_best_segment = exit_by_frame(points_with_best_segment)
    print("exits after join:", len(points_with_best_segment))
    if is_debug:
        scatter(get_x_dimension(points), get_y_dimension(points), linewidth=0.1, s=2)
        for point, seg in points_with_best_segment:
            plot(point.x, point.y, 'ro')
        show()

    return points_with_best_segment


def get_exit_point(points, is_up_minus=True, is_exit=True, output_path="", is_debug=False, walk=0):
    # Constants
    if len(points) > 100:
        start = time()

        eps = 0.0125
        points, clean_points, corner_points, eps, lines = build_first_rectangle(eps, points, is_debug)
        print("building first rectangle time: " + str(time() - start) + " seconds")

        time1 = time()
        # corner_points, lines, rect_lines = build_expanded_rectangle(points, corner_points, lines, time1, is_debug)
        print("build expanded rectangle: " + str(time() - time1) + " seconds")

        time2 = time()
        outside_points = find_best_filter(corner_points, points, lines, is_exit)
        print("find best filter time: " + str(time() - time2) + " seconds")

        time3 = time()
        exit_points_index, num_of_clusters, clusters, not_enough_information, outside_points = find_clusters_and_exit(
            corner_points, outside_points, points, eps, lines, lines, is_up_minus, is_exit, is_debug)
        print("find clusters and exit points time: ", str(time() - time3))

        time4 = time()
        points_with_best_segment = find_exit_points(points, outside_points, clusters, lines, is_exit, exit_points_index,
                                                    not_enough_information, is_debug)
        print("find exit points time: " + str(time() - time4) + " seconds")

        print("Program Time: " + str(time() - start))
        if output_path != "":
            exit_point = points_with_best_segment[0][0]
            with open(output_path + "exitPoint.csv", "w") as exitPointFile:
                exitPointFile.write(str(exit_point.x) + "," + str(exit_point.y) + "," + str(exit_point.z) + "\n")
        return points_with_best_segment, corner_points


def get_all_data_files():
    i = 0
    for dirpath, dirnames, filenames in os.walk("./Database"):
        for filename in [f for f in filenames if f.endswith(".csv")]:
            try:
                # title(os.path.join(dirpath, filename))
                print(os.path.join(dirpath, filename))
                get_exit_point(create_data(os.path.join(dirpath, filename)), True, True, i)
            except Exception as e:
                print(e.args[0])
            i += 1


def main():
    # get_all_data_files()
    # get_exit_point(create_data("/tmp/pointData0.csv"), True, True)
    points0 = create_date_from_colmap("/home/rbdstudent/Documents/colmap/Dani_up_down/images1fps/images.txt",
                                      "/home/rbdstudent/Documents/colmap/Dani_up_down/images1fps/points3D.txt")
    returned_values = get_exit_point(points0, True, True, "/tmp/", True)
    pass


if __name__ == '__main__':
    main()
