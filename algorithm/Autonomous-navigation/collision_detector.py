import math

import numpy as np

from auxiliary_functions import calculate_distance, get_motion_vector_as_numpy_array, center_of_mass


def get_details_about_ray(ray_points, current_position, ray_segment,is_up_is_minus):
    mass_center = center_of_mass(ray_points)
    to_mass_center_ray = get_motion_vector_as_numpy_array(current_position, mass_center)
    ray_motion_vector = get_motion_vector_as_numpy_array(ray_segment[0], ray_segment[1])
    angle = get_angle(to_mass_center_ray, ray_motion_vector)
    mass_center_radius = calculate_distance(mass_center, ray_segment[0]) / 2
    avg_height = get_average_height_around_central_mass(ray_points, mass_center, mass_center_radius)
    is_avg_lower = current_position.z > avg_height if is_up_is_minus else current_position.z < avg_height
    print("is_avg_lower:"+str(is_avg_lower))
    print("angle:"+str(angle))
    is_wall = angle > 30 and not is_avg_lower
    points = get_distance_to_points_dictionary(ray_points, current_position)
    keys = points.keys()
    order_keys = sorted(keys) if len(keys) > 1 else keys
    checkpoint = points[order_keys[0]] if is_wall else points[order_keys[-1]]
    return is_wall, checkpoint


def get_average_height_around_central_mass(ray_points, mass_center, mass_center_radius):
    avg = 0
    amount_of_close_points = 0
    for point in ray_points:
        if calculate_distance(mass_center, point) < mass_center_radius:
            avg += point.z
            amount_of_close_points += 1
    return avg / amount_of_close_points


def get_distance_to_points_dictionary(frame_ray_points, current_position):
    points = {}
    for point in frame_ray_points:
        points[calculate_distance(point, current_position)] = point
    return points


def get_angle(vector_1, vector_2):
    try:
        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        angle = int(math.degrees(np.arccos(dot_product)))
        if angle > 90:
            angle = 180 - angle
        return angle
    except Exception as e:
        print(e.args[0] + "vector_1:" + str(vector_1[0]) + "," + str(vector_1[1]) + "\nvector 2:" + str(
            vector_2[0]) + "," + str(vector_2[1]))
        return 90


"""path = "/home/livne/PycharmProjects/drone_project/dani"
dirs = os.listdir(path)

for i in range(1,4):
    wall_path = path + "/scan_" + i + "/scan_" + i + "_frames/wall_frames"
    wall_dirs = os.listdir(wall_path)
    for file in dirs:
        if file == '*.csv':
            print file"""
