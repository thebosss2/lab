import math
import matplotlib.pyplot as plt
import csv
import numpy as np
from scipy.spatial import distance
import cmath
# settings:
angel = math.pi / 9  # the amount of degrees in each section default prob 20
folder = "data"
suffix = "PointData"
points_file = "{}/{}".format(folder, suffix)
height_removal = -2.0  # only looks at points from this height and more, put this to -100.0 if u dont want limit

'''
"test list:
"pointData0",
"pointData8",
"pointDataDekel",
"pointDataVered" 
"pointData0 (1)"
'''

def points_in_section(points, degree1, degree2):
    # returns the points in the section from degree1 to degree2
    new_points = []
    e1 = [1.0, 0.0]

    v1_cross = rotate(e1, degree1 + 90)
    v2_cross = rotate(e1, degree2 + 90)

    for point in points:
        if np.dot(v1_cross, np.array(point)) > 0 and np.dot(v2_cross, np.array(point)) < 0:
            new_points.append(point)
    return new_points


def rotate(vector, degree):
    # rotates vector by 'degree' degrees
    rotation = np.deg2rad(degree)
    rot = np.array([[math.cos(rotation), -math.sin(rotation)], [math.sin(rotation), math.cos(rotation)]])

    v = np.array(vector)
    new_v = np.dot(rot, v)
    return new_v

def clean_noise(points):

    to_remove = []
    list_of_mean_dist_per_point = []
    for x in points:
        mean_dists = np.mean([distance.euclidean(x,p) for p in points])
        list_of_mean_dist_per_point.append(mean_dists)

    cutoff = sorted(list_of_mean_dist_per_point)[len(points)-10]

    for i in range(len(points)):
        p = points[i]
        if list_of_mean_dist_per_point[i] > cutoff:
            to_remove.append(p)
    points = [x for x in points if not x in to_remove]
    return points

def sort_by_index(arr, i):
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
        return sort_by_index(less,i)+equal+sort_by_index(greater,i)
    else:
        return arr


def get_exit_point(points):
    
    polar_points = []
    approx_poligon_polar = []

    x_arr = []
    y_arr = []

    for x in points:
        rho, phi = cmath.polar(complex(x[0],x[1]))
        polar_points.append([phi + math.pi,rho])
     
    polar_points = sort_by_index(polar_points, 0)

    for section in np.arange(0, 2* math.pi, math.pi / 90):
        distances = [x[1] for x in polar_points if x[0] > section and x[0] < section + angel]
        if len(distance) == 0:
            continue
        approx_poligon_polar.append([section + angel / 2 - math.pi, np.mean(distances)])

    for p in approx_poligon_polar:
        tmp = [p[1] * np.cos(approx_poligon_polar[0]), p[1] * np.sin(approx_poligon_polar[0])]
        approx_poligon_polar.append(tmp)
        x_arr.append(tmp[0])
        y_arr.append(tmp[1])
    
    plt.scatter(x_arr, y_arr, color="green")


def main():
    max_value = 0.0
    points = []
    points3d = []

    x = []
    y = []
    with open(points_file + ".csv", newline='') as f:
        reader = csv.reader(f)
        data = list(reader)
    for i in data:
        points.append([float(i[0]),float(i[2])])  # reads the points, z value= float(i[1])
        points3d.append([float(i[0]),float(i[2]),float(i[1])])

    print(points3d)
    for point in points3d:  # deletes points for height_removal height (point below cause they have more noise)
        if point[2] <= height_removal:
            points.remove([point[0], point[1]])

    points = [list(x) for x in set(tuple(x) for x in points)]

    # points = clean_noise(points)

    # TODO: maybe add clusters and noise cleaning

    # TODO: maybe only take points from some height

    # for section in range(0, 360, 2):  # checks each section
    #     sub_points = points_in_section(points, section, section + angel)  # all the points in the section
    #     if len(sub_points) == 0:
    #         continue
    #     per_point_distance = []
    #     for point in sub_points:
    #         per_point_distance.append(math.sqrt(point[0] ** 2 + point[1]**2))

    #     # TODO: add finding longest path of dots

    #     max = 0
    #     second_score = 0

    #     max_index = 0
    #     for i in range(len(per_point_distance)):
    #         if per_point_distance[i] > max:
    #             max = per_point_distance[i]
    #             max_index = i
    #     value = max  # - np.amin(distance)

    #     # if value > max_value:
    #     #     max_value = value
    #     #     out_of_room = sub_points[max_index]
    #     #     max_section = section

    #     value = 0
    #     mean_dist = np.mean(per_point_distance)
    #     mean_std = np.mean(per_point_distance)
    #     std_cuttoffs = [mean_dist - mean_std * 2.5, mean_dist + mean_std * 2.5]
    #     centered_distances = [x for x in per_point_distance if x > std_cuttoffs[0] and x < std_cuttoffs[1]]
    #     interval_h = np.max(centered_distances) - np.min(centered_distances)
    #     interval_den_in = interval_h / len(centered_distances)
    #     segment_score = interval_h ** 2 + interval_den_in
    #     if segment_score > max_value:
    #         max_value = segment_score
    #         out_of_room = sub_points[max_index]
    #         max_section = section

    for point in points:
        x.append(float(point[0]))
        y.append(float(point[1]))

    plt.scatter(x, y, color="blue")
    # x1 = []
    # y1 = []
    # for i in points_in_section(points, max_section, max_section + angel):
    #     x1.append(i[0])
    #     y1.append(i[1])
    # plt.scatter(x1, y1, color="yellow")

    # plt.scatter([out_of_room[0]], [out_of_room[1]], color="black")

    get_exit_point(points)

    plt.savefig(f"data/map/{suffix}.png")  # prints the map, REMOVE WHEN IN DRONE

    # TODO: add a way to return the point but to the drone


main()
