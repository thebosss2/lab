import numpy as np
from numpy import isclose

from Point import get_x_dimension, get_y_dimension, get_z_dimension


def filter_floor_points(points):
    floorPoints = sorted(points, key=lambda p: p.z)
    return floorPoints[:100]


def find_close_height(points, lowset_height):
    filtered_points = []
    for point in points:
        if isclose(point.z, lowset_height, atol=2.5):
            filtered_points.append(point)
    return filtered_points


def fit_points_to_plane(points, x, y, z, epsilon=0.1):
    # print("plane solution:")
    # print("%f x + %f y + %f = z" % (x, y, z))

    floor = []
    counter = 0
    for p in points:
        result = (x * p.x) + (y * p.y) + z
        if isclose(result, p.z, atol=epsilon):
            floor.append(p)
            counter = counter + 1

    # print("counter = " , counter)
    return floor


def find_floor(points):
    frames = {}
    for point in points:
        if point.frame_id in frames.keys():
            frames[point.frame_id].append(point)
        else:
            frames[point.frame_id] = [point]
    total_floor_x = []
    total_floor_y = []
    total_floor_z = []
    for key in frames.keys():
        try:
            if len(frames[key]) > 5:

                floorPoints = filter_floor_points(frames[key])
                """lowest_height = min(points,key=lambda p: p.z).z
                floorPoints = find_close_height(frames[key],lowest_height)"""

                xs = get_x_dimension(floorPoints)
                ys = get_y_dimension(floorPoints)
                zs = get_z_dimension(floorPoints)
                # find 3d plane
                tmp_A = []
                tmp_b = []
                for i in range(len(xs)):
                    tmp_A.append([xs[i], ys[i], 1])
                    tmp_b.append(zs[i])
                b = np.matrix(tmp_b).T
                A = np.matrix(tmp_A)

                fit = (A.T * A).I * A.T * b

                # plot floor fit to plane
                floor = fit_points_to_plane(frames[key], fit[0], fit[1], fit[2])
                total_floor_x += get_x_dimension(floor)
                total_floor_y += get_y_dimension(floor)
                total_floor_z += get_z_dimension(floor)

        except:
            continue
    filtered_x = []
    filtered_y = []
    filtered_z = []
    without_floor = []
    for point in points:
        if point.x not in total_floor_x:
            filtered_x.append(point.x)
            filtered_y.append(point.y)
            filtered_z.append(point.z)
            without_floor.append(point)
    """plt.figure()
    ax = plt.subplot(111, projection='3d')
    ax.scatter(filtered_x, filtered_y, filtered_z, color='b')
    ax.scatter(total_floor_x, total_floor_y, total_floor_z, color='r')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')"""
    print(str(len(without_floor)))
    print(str(len(points)))
    # plt.show()
    return without_floor


"""points = create_data("/tmp/pointData0.csv")
find_floor(points)"""
