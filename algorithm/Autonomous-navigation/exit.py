from math import isnan
from sys import maxsize
from time import time

import matplotlib.pyplot as plt
from numpy import vstack, zeros, zeros_like, linspace
from sklearn.cluster import DBSCAN

from Point import get_y_dimension, get_x_dimension
from auxiliary_functions import get_index_of_closest_point_to_rectangle, calculate_distance


def average_height_of_cluster(labels, cluster_label, data_heights):
    sum = 0
    count = 0
    for i in range(len(labels)):
        if labels[i] == cluster_label:
            sum = sum + data_heights[i]
            count = count + 1
    if count != 0:
        return sum / count
    return None


def min_height_of_cluster(labels, cluster_label, data_heights):
    min_height = maxsize
    for i in range(len(labels)):
        if data_heights[i] < min_height and labels[i] == cluster_label:
            min_height = data_heights[i]
    return min_height


def max_height_of_cluster(labels, cluster_label, data_heights, is_up_minus):
    max_height = -3000
    for i in range(len(labels)):
        if is_up_minus:
            if data_heights[i] < max_height and labels[i] == cluster_label:
                max_height = data_heights[i]
        else:
            if data_heights[i] > max_height and labels[i] == cluster_label:
                max_height = data_heights[i]

    return max_height


def return_cluster_with_min_height(min_heights):
    min_height = None
    min_cluster = None
    for i in range(len(min_heights)):
        print("min Height: " + str(min_heights[i]))
        print("cluster : " + str(i))
        if not isnan(min_heights[i]):
            if min_cluster is not None:
                if min_heights[i] < min_height:
                    min_height = min_heights[i]
                    min_cluster = i
            else:
                min_height = min_heights[i]
                min_cluster = i
    return min_cluster


def return_cluster_with_max_height(max_heights):
    max_height = None
    max_cluster = None
    for i in range(len(max_heights)):
        if not isnan(max_heights[i]):
            if max_cluster is not None:
                if max_heights[i] > max_height:
                    max_height = max_heights[i]
                    max_cluster = i
            else:
                max_height = max_heights[i]
                max_cluster = i
    return max_cluster


def return_cluster_with_min_average_height(average_heights):
    min_average = None
    min_cluster = None
    for i in range(len(average_heights)):
        if not isnan(average_heights[i]):
            if min_cluster is not None:
                if average_heights[i] < min_average:
                    min_average = average_heights[i]
                    min_cluster = i
            else:
                min_average = average_heights[i]
                min_cluster = i
    return min_cluster


def return_first_point_from_cluster(label, labels):
    for i in range(len(labels)):
        if labels[i] == label:
            return i
    return -1  # happens??


def closest_corner(point, corner_points):
    min_distance = maxsize
    for i in range(4):
        d = calculate_distance(point, corner_points[i])
        if min_distance > d:
            min_distance = d
            closest_corner = corner_points[i]

    return closest_corner


def find_filtered_clusters_exit(points, original_points, lines, is_up_minus, is_debug):
    start = time()
    # Compute DBSCAN\
    Data = vstack((get_x_dimension(points), get_y_dimension(points))).T
    db = DBSCAN(eps=0.2, min_samples=2).fit(Data)  # 0.1 , 10

    labels = db.labels_
    data_heights = [point.z for point in points]
    min_heights = zeros(max(labels) + 1)
    max_heights = zeros(max(labels) + 1)
    clusters_amount = max(labels + 1)

    if clusters_amount > 0:
        for i in range(len(min_heights)):
            if min_heights[i] is not None:
                min_heights[i] = min_height_of_cluster(labels, i, data_heights)

        for i in range(len(max_heights)):
            if max_heights[i] is not None:
                max_heights[i] = max_height_of_cluster(labels, i, data_heights, is_up_minus)
        max_height_cluster = return_cluster_with_max_height(max_heights)
        exit_point_index_max = get_index_of_closest_point_to_rectangle(max_height_cluster, labels, points, lines)
        x_exit_max = points[exit_point_index_max].x
        y_exit_max = points[exit_point_index_max].y

    if is_debug:
        core_samples_mask = zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True
        unique_labels = set(labels)
        colors = [plt.cm.Spectral(each)
                  for each in linspace(0, 1, len(unique_labels))]
        plt.scatter(get_x_dimension(original_points), get_y_dimension(original_points), linewidth=0.1, s=2)

        for k, col in zip(unique_labels, colors):
            if k == -1:
                # Black used for noise.
                col = [0, 0, 0, 1]

            class_member_mask = (labels == k)

            xy = Data[class_member_mask & core_samples_mask]
            plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                     markeredgecolor='k', markersize=14)

            xy = Data[class_member_mask & ~core_samples_mask]
            plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                     markeredgecolor='k', markersize=6)
            if clusters_amount > 0:
                # plt.plot(exit_point.x, exit_point.y, 'bo')
                plt.plot(x_exit_max, y_exit_max, 'ro')
        plt.title('Estimated number of clusters:' + str(max(labels) + 1))
        plt.show()
    exit_point = []
    if max(labels) == -1:
        exit_point.append(None)
    else:
        # exit_point = [exit_point_index]
        exit_point = [exit_point_index_max]

    print("DBscan takes: " + str(time() - start))
    for i in range(len(labels)):
        points[i].label = labels[i]

    clusters = []
    for i in range(clusters_amount):
        clusters.append([])

    for point in points:
        if point.label != -1:
            clusters[point.label].append(point)
    return exit_point, (max(labels) + 1), clusters
