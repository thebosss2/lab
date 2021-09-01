import math
import matplotlib.pyplot as plt
import csv
import numpy as np

# settings:
angel = 20.0  # the amount of degrees in each section default prob 20
points_file = "data2//PointData"
# only looks at points from this height and more, put this to -100.0 if u dont want limit
height_removal = -2.0

'''
test list:
pointData0
pointData8
pointDataDekel
pointDataVered
pointData0(1)
'''


def polar(points):
    # moves to polar representation and sorting by the angle
    polar_points = []
    for point in points:
        polar_point = (math.sqrt(point[0]**2 + point[1]**2), np.arctan2(point[1], point[0]))
        polar_points.append(polar_point)
    polar_points.sort(key=lambda point: point[1])
    return polar_points


def binary_search(points, angle):
    #finds the nearest point to "angle"
    low = 0
    high = len(points)-1
    while low <= high:
        middle = low + (high - low) // 2

        if (points[middle])[1] == angle:
            return middle
        elif (points[middle])[1] < angle:
            low = middle + 1
        else:
            high = middle - 1
    return middle


def points_in_section(points, base_deg, range_deg):
    # returns the points in the section from around base_deg
    section = []
    start = binary_search(points, base_deg - range_deg/2)
    stop = binary_search(points, base_deg + range_deg/2)
    for i in range(start, stop):
        section.append(points[i])
    return section


def main():
    max_value = 0.0
    points = []
    points3d = []

    with open(points_file + ".csv", newline='') as f:
        reader = csv.reader(f)
        data = list(reader)
    for i in data:
        # reads the points, z value= float(i[1])
        points.append((float(i[0]), float(i[2])))
        points3d.append((float(i[0]), float(i[2]), float(i[1])))

    print(points3d)
    # deletes points for height_removal height (point below cause they have more noise)
    for point in points3d:
        if point[2] <= height_removal:
            points.remove((point[0], point[1]))

    # TODO: maybe add clusters and noise cleaning

    # TODO: maybe only take points from some height


    x = []
    y = []

    print (points)
    points = polar(points)
    print(points)
    for section in range(0, 360, 2):  # checks each section
        # all the points in the section
        sub_points = points_in_section(points, section, angel)
        distance = []
        for point in sub_points:
            distance.append(point[0])

        # TODO: add finding longest path of dots
        max = 0
        max_index = 0
        for i in range(len(distance)):
            if distance[i] > max:
                max = distance[i]
                max_index = i
        value = max  # - np.amin(distance)

        if value > max_value:
            max_value = value
            out_of_room = sub_points[max_index]
            max_section = section

    for point in points:
        x.append(float(point[0]*math.cos(point[1])))
        y.append(float(point[0]*math.sin(point[1])))

    plt.scatter(x, y, color="blue")

    x1 = []
    y1 = []
    for point in points_in_section(points, max_section, angel):
        x1.append(float(point[0]*np.cos(point[1])))
        y1.append(float(point[0]*np.sin(point[1])))
    plt.scatter(x1, y1, color="yellow")

    plt.scatter(float(out_of_room[0]*np.cos(out_of_room[1])),
                float(out_of_room[0]*np.sin(out_of_room[1])), color="black")

    plt.show()  # prints the map, REMOVE WHEN IN DRONE

    # TODO: add a way to return the point but to the drone


main()
