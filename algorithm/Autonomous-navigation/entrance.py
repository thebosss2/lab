import matplotlib.pyplot as plt
from numpy import vstack, zeros_like, linspace
from sklearn.cluster import DBSCAN

from Point import get_y_dimension, get_x_dimension
from auxiliary_functions import get_index_of_closest_point_to_rectangle
from join_clusters import join_clusters


def return_first_point_from_cluster(label, labels):
    for i in range(len(labels)):
        if labels[i] == label:
            return i
    return -1  # happens??


def find_filtered_clusters_entrance(points, original_points, eps, lines,is_debug):
    # Compute DBSCAN
    data = vstack((get_x_dimension(points), get_y_dimension(points))).T
    db = DBSCAN(eps=0.15, min_samples=3).fit(data)  # eps = 0.2, 3

    labels = db.labels_
    num_of_clusters = max(labels) + 1
    labels, num_of_clusters = join_clusters(points, labels, num_of_clusters, eps)

    # Black removed and is used for noise instead.

    if max(labels) == -1:
        return [None], 0
    num_of_clusters = max(labels) + 1
    exit_points = []
    for cluster in range(num_of_clusters):
        entrance_point_index = get_index_of_closest_point_to_rectangle(cluster, labels, points, lines)
        exit_points.append(entrance_point_index)

    #
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

            xy = data[class_member_mask & core_samples_mask]
            plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                     markeredgecolor='k', markersize=14)

            xy = data[class_member_mask & ~core_samples_mask]
            plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                     markeredgecolor='k', markersize=6)
        plt.title('Estimated number of clusters:' + str(max(labels) + 1))
        for cluster in exit_points:
            point = points[cluster]
            plt.plot(point.x, point.y, 'ro')
        plt.show()
    for i in range(len(labels)):
        points[i].label = labels[i]

    clusters = []
    for i in range(num_of_clusters):
        clusters.append([])

    for point in points:
        if point.label != -1:
            clusters[point.label].append(point)

    return exit_points, num_of_clusters, clusters, labels
