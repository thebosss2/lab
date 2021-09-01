import cv2
import matplotlib.pyplot as plt
from numpy import isclose

from Point import get_x_dimension, get_y_dimension, get_z_dimension


def three_d_by_points(points):
    ax = plt.axes(projection='3d')
    z = get_z_dimension(points)
    ax.scatter(get_x_dimension(points), get_y_dimension(points), z, c=z, cmap='viridis', linewidth=0.1, s=2)
    plt.show()
def colmap_3d(filename):
    x = []
    y = []
    z = []
    with open(filename, "r") as f:
        for line in f.readlines():
            values = line.split(" ")
            x.append(float(values[1]))
            y.append(float(values[3]))
            z.append(float(values[2]))

    ax = plt.axes(projection='3d')
    ax.scatter(x, y, z, c=z, cmap='viridis', linewidth=0.1, s=2)
    plt.show()
def colmap_2d(filename):
    x = []
    y = []
    z = []
    with open(filename, "r") as point3d_file:
        for i in range(0, 3):
            point3d_file.readline()
        row = point3d_file.readline()
        while row != '':
            values = row.split(" ")
            x.append(float(values[1]))
            y.append(float(values[3]))
            z.append(float(values[2]))
            row = point3d_file.readline()
    fig = plt.figure()
    plt.scatter(x, z, linewidth=0.1, s=0.5)
    # fig.savefig('plot.png')
    plt.show()

def threeD(file_name):
    x = []
    y = []
    z = []
    with open(file_name, "r") as f:
        for line in f.readlines():
            if "x,y,z" not in line:
                data = line.split(",")
                x.append(float(data[0]))
                y.append(float(data[1]))
                z.append(float(data[2]))

    ax = plt.axes(projection='3d')
    ax.scatter(x, y, z, c=z, cmap='viridis', linewidth=0.1, s=2)
    plt.show()
    return x, z, y


def twoD(file_name):
    x = []
    y = []
    z = []
    with open(file_name, "r") as f:
        for line in f.readlines():
            if "x,y,z" not in line:
                data = line.split(",")
                if "nan" not in data:
                    x.append(float(data[0]))
                    y.append(float(data[1]))
                    z.append(float(data[2]))
    fig = plt.figure()
    plt.scatter(x, z, linewidth=0.1, s=0.5)
    # fig.savefig('plot.png')
    plt.show()
    return x, z


def plot_two_d(points):
    fig = plt.figure()
    plt.scatter(get_x_dimension(points), get_y_dimension(points), linewidth=0.1, s=2)
    # fig.savefig('plot.png')
    plt.show()


def filter_floor(points):
    lowest_point = points[0].z
    for point in points:
        if lowest_point < point.z:
            lowest_point = point.z
    points_without_floor = []
    for point in points:
        if not isclose(point.z, lowest_point, atol=1.5):
            points_without_floor.append(point)
    return points_without_floor


if __name__ == '__main__':
    #twoD("/tmp/pointData0.csv")
    #threeD("/tmp/pcl_data_tmp.csv")
    colmap_3d("/home/rbdstudent/points3D.txt")
    # points = create_data("/tmp/pointData0.csv")
    # three_d_by_points(filter_floor(points))
    # three_d_by_points(points)
