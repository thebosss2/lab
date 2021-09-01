from numpy import *
import sys
import math
from auxiliary_functions import calculate_distance

def find_center(X_x, X_y):
    sum_x = 0
    sum_y = 0
    for i in range(len(X_x)):
        sum_x += X_x[i][0]
        sum_y += X_y[i][1]

    center_x = sum_x / len(X_x)
    center_y = sum_y / len(X_y)
    return center_x, center_y


def min_distance_between_two_clusters(cluster1, cluster2):
    min = sys.maxsize
    for point1 in cluster1:
        for point2 in cluster2:
            if calculate_distance(point1, point2) < min:
                min = calculate_distance(point1, point2)
    return min


def join_two_clusters(cluster1, cluster2, labels):
    label1 = labels[cluster1[0]]
    for index in cluster2:
        labels[index] = label1
    k = 0
    for i in range(max(labels) + 1):
        is_empty = True
        for label in labels:
            if label == i:
                is_empty = False
                break
        if not is_empty:
            for label in range(len(labels)):
                if labels[label] == i:
                    labels[label] = k
            k += 1
    return labels


def cluster_edges(cluster):
    x_min = sys.maxsize
    x_max = -sys.maxsize + 1
    y_min = sys.maxsize
    y_max = -sys.maxsize + 1
    for point in cluster:
        if point.x < x_min:
            x_min = point.x
        if point.x > x_max:
            x_max = point.x
        if point.y < y_min:
            y_min = point.y
        if point.y > y_max:
            y_max = point.y
    return x_min, x_max, y_min, y_max


def is_above(cluster1, cluster2, distance):  # cluster 2 above cluster 1
    x_min_1, x_max_1, y_min_1, y_max_1 = cluster_edges(cluster1)
    x_min_2, x_max_2, y_min_2, y_max_2 = cluster_edges(cluster2)

    if x_min_1 < x_min_2 and x_max_1 > x_max_2 and y_min_2 > y_max_1:
        if y_min_2 - y_max_1 <= distance:
            return True
    return False


def join_clusters(points, labels, num_of_clusters, eps):
    is_joined = False
    input_clusters = []
    distances = []
    point_index_clusters = []
    for num_clusters in range(num_of_clusters):
        point_index_clusters.append([])
    for i in range(len(points)):
        if num_of_clusters > labels[i] >= 0:
            point_index_clusters[labels[i]].append(i)
    for k in range(num_of_clusters):
        input_clusters.append([])
    for l in range(len(labels)):
        if labels[l] < num_of_clusters and labels[l] >= 0:
            input_clusters[labels[l]].append(points[l])

    for cluster_index1 in range(num_of_clusters):
        for cluster_index2 in range(num_of_clusters):
            i += 1
            # added

            # until here
            if cluster_index1 != cluster_index2:
                cluster1_values = input_clusters[cluster_index1]
                cluster2_values = input_clusters[cluster_index2]
                cluster1_indexes = point_index_clusters[cluster_index1]
                cluster2_indexes = point_index_clusters[cluster_index2]
                min_distance_cluster1_cluster2 = min_distance_between_two_clusters(cluster1_values, cluster2_values)
                distances.append(min_distance_cluster1_cluster2 / eps)
                condition1 = min_distance_cluster1_cluster2 <= eps * 10
                condition2 = is_above(cluster1_values, cluster2_values, eps * 20)
                if condition1 or condition2:
                    labels = join_two_clusters(cluster1_indexes, cluster2_indexes, labels)
                    is_joined = True
                    break
                else:
                    pass
        if is_joined:
            break
    distances.sort()
    if is_joined:
        num_of_clusters -= 1
        labels, num_of_clusters = join_clusters(points, labels, num_of_clusters, eps)

    return labels, num_of_clusters


def point_in_which_cluster(point, clusters):
    for cluster_index in range(len(clusters)):
        for point_cluster in clusters[cluster_index]:
            if point == point_cluster:
                return cluster_index


def exit_by_frame(points_with_best_segment):
    is_with_median = False
    for point, seg in points_with_best_segment:
        if point.frame_id != 0:
            is_with_median = True
    if not is_with_median:
        return points_with_best_segment

    new_points_with_best_segment = []
    new_points = {}
    for point, seg in points_with_best_segment:
        if point.frame_id not in new_points.keys():
            new_points[point.frame_id] = [point, seg]
            new_points_with_best_segment.append([point, seg])
    return new_points_with_best_segment

