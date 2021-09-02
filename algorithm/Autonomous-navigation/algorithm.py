import math
import matplotlib.pyplot as plt
import csv
import numpy as np
from scipy.spatial import distance
import cmath
import sklearn.neighbors as kdt
import time

# settings:
angel = 10 * (math.pi / 180)  # the amount of degrees in each section default prob 20
# folder = "data"  # temporary variables for our convenient
suffix = "PointData"
# points_file = "{}/{}".format(folder, suffix)
height_removal = -2.0  # only looks at points from this height and more, put this to -100.0 if you don't want limit
radius = 0.2  # variables used to clean isolated points
density = 2


def clean_noise(points, rad, den):
    new_points = []
    tree = kdt.KDTree(points, leaf_size=2)

    # read more about this data structure: https://en.wikipedia.org/wiki/K-d_tree

    for x in points:  # count the number of points within the given radius in O(log n) per query
        if tree.query_radius([x], r=rad, count_only=True) > den:
            new_points.append(x)

    return new_points


def sort_by_index(arr, i):  # quick sort of a list by the i_th index
    less = []
    equal = []
    greater = []
    if len(arr) > 1:
        pivot = arr[0][i]
        for x in arr:
            if x[i] < pivot:
                less.append(x)
            elif x[i] == pivot:
                equal.append(x)
            elif x[i] > pivot:
                greater.append(x)
        return sort_by_index(less, i) + equal + sort_by_index(greater, i)
    else:
        return arr


def to_polar(points):  # convert cartesian coordinates to polar
    polar_points = []

    for x in points:
        rho, phi = cmath.polar(complex(x[0], x[1]))
        polar_points.append([phi + math.pi, rho])

    return polar_points


def find_exit_point(points):
    polar_points = to_polar(points)

    polar_points = sort_by_index(polar_points, 0)  # sort the points by the angular coordinate

    approx_polygon_polar = []
    approx_polygon = []
    for section in np.arange(0, 2 * math.pi, math.pi / 90):  # finding approximate room edge in the given section
        distances = [x[1] for x in polar_points if section < x[0] < section + angel]
        if len(distances) == 0:
            continue
        approx_polygon_polar.append([section + angel / 2 - math.pi, np.mean(distances)])

    x_arr = []
    y_arr = []
    for p in approx_polygon_polar:  # convert the points creating the approximate room to cartesian coordinates
        tmp = [p[1] * np.cos(p[0]), p[1] * np.sin(p[0])]
        approx_polygon.append(tmp)
        x_arr.append(tmp[0])
        y_arr.append(tmp[1])

    plt.scatter(x_arr, y_arr, color="green")  # paint those points in green

    center = [np.mean(x_arr), np.mean(y_arr)]  # calculate the approximate room center and paint it in pink
    plt.scatter([center[0]], [center[1]], color="pink")

    exit_distance = -1  # determine the exit point to be the farthest from the approximate room center
    exit_point = []
    for point in approx_polygon:
        tmp = distance.euclidean(point, center)
        if exit_distance < tmp:
            exit_distance = tmp
            exit_point = point
    tree = kdt.KDTree(points, leaf_size=2)
    i = tree.query([exit_point], k=1, return_distance=False)  # return the closest feature to the selected exit point
    x = i[0]
    return points[x[0]]


def exit_algorithm(points_file):
    points = []
    points3d = []

    with open(points_file, newline='') as f:  # read the points from the file edited by ORB_SLAM2
        reader = csv.reader(f)
        data = list(reader)
    for i in data:
        points.append([float(i[0]), float(i[2])])
        points3d.append([float(i[0]), float(i[2]), float(i[1])])

    # print(points3d) # optional printing of the points received- for debug

    for point in points3d:  # remove unnecessary features on the floor- optional by modify the height_removal variable
        if point[2] <= height_removal:
            points.remove([point[0], point[1]])

    points = [list(x) for x in set(tuple(x) for x in points)]  # delete duplications

    x = []
    y = []
    for point in points:  # paint all the points in grey- points to be removed will not be painted in blue
        x.append(float(point[0]))
        y.append(float(point[1]))
    plt.scatter(x, y, color="grey")

    clean_time = time.time()

    points = clean_noise(points, radius, density)  # clean the isolated points and paint the rest in blue
    x = []
    y = []
    for point in points:
        x.append(float(point[0]))
        y.append(float(point[1]))
    plt.scatter(x, y, color="blue")

    exit_time = time.time()
    exit_point = find_exit_point(points)  # find the exit point and paint it in yellow
    plt.scatter(exit_point[0], exit_point[1], color="yellow")

    print("Time to clean noise: {}".format(exit_time - clean_time))
    print("Time to calculate exit point: {}".format(time.time() - clean_time))

    # plt.show()  # show the features` map
    plt.savefig(f"{suffix}.png")  # save the map for later analysis

    exit_point3d = [x for x in points3d if x[0] == exit_point[0] and x[1] == exit_point[1]][0]
    return exit_point3d
