from math import isnan
from sys import maxsize

from sklearn.cluster import DBSCAN


def min_height_of_cluster(labels, cluster_label, data_heights):
    min_height = maxsize
    for i in range(len(labels)):
        if data_heights[i] < min_height and labels[i] == cluster_label:
            min_height = data_heights[i]
    return min_height


def max_height_of_cluster(labels, cluster_label, data_heights):
    max_height = maxsize
    for i in range(len(labels)):
        if data_heights[i] < max_height and labels[i] == cluster_label:
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


def return_first_point_from_cluster(label, labels):
    for i in range(len(labels)):
        if labels[i] == label:
            return i
    return -1


def create_labels(points, eps, min_samples):
    index = 0
    for label in DBSCAN(eps=eps, min_samples=min_samples).fit(
            [[point.x, point.y] for point in points]).labels_:  # 0.0125
        points[index].label = label
        index += 1
    return points
