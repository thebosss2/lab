import Line
import Point
import Polygon
from auxiliary_functions import center_of_mass, calculate_distance, plot_pizza_with_given_center, get_original_angle
from Point import create_data, get_x_dimension, get_y_dimension
import Pizza
from matplotlib.pyplot import scatter, plot, show
from math import floor
import csv


def get_path_points(points, angle):
    center = center_of_mass(points)
    lines = Pizza.create_pizza_lines(center, angle)
    slices = Pizza.create_slices(lines, points, 20)
    exit_points = []
    for slice in slices:
        if slice is not None:
            points_with_distance = []
            for point in slice:
                points_with_distance.append([point, calculate_distance(point, center)])
            points_with_distance.sort(key=lambda point_with_distance: point_with_distance[1])
            exit_points.append(points_with_distance[floor(len(points_with_distance)*0.5)][0])
    return exit_points, slices


def get_polygon(x):
    points = create_data("/tmp/pointData{}.csv".format(x))
    angle = 20
    exit_points, slices = get_path_points(points, angle)
    # scatter(get_x_dimension(points), get_y_dimension(points), linewidth=0.1, s=2)

    #for point in exit_points:
       #plot(point.x, point.y, 'ro')
    center = center_of_mass(points)
    # plot_pizza_with_given_center(points, 20, center)
    exit_points.append(exit_points[0])
    polygon = Polygon.Polygon(exit_points)
    end = False
    while True:
        edges = []
        temp = polygon.vertices[0]
        for i, vertex in enumerate(polygon.vertices):
            if i == 0:
                continue
            edges.append(Line.Line(temp, vertex))
            temp = vertex
        edges.append(edges[0])
        temp = edges[0]
        for i, edge in enumerate(edges):
            if i == 0:
                continue
            current_angle = get_original_angle(temp, edge)
            # TODO: find the right angle!
            if current_angle < 10:
                polygon.vertices.remove(polygon.vertices[i-1])
                break
            if i == len(edges) - 1:
                end = True
                break
            temp = edge
        if end:
            break

    exit_points = [Point.Point((vert.x-center.x) * 0.75 + center.x, (vert.y-center.y) * 0.75 + center.y, vert.z) for vert in polygon.vertices]

    # scatter(get_x_dimension(points), get_y_dimension(points), linewidth=0.1, s=2)
    # for point in exit_points:
    #     plot(point.x, point.y, 'ro')
    exit_points.append(exit_points[0])
    # plot([point.x for point in exit_points], [point.y for point in exit_points])
    # show()
    with open('./Data/output3/exitPoints.csv', 'w') as out:
        csv_out = csv.writer(out)
        for p in exit_points:
            csv_out.writerow((p.x, p.y, p.z))
            print("({},{},{})".format(p.x, p.y, p.z))

    return exit_points


# get_polygon(1)
