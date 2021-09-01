import glob
import math
import os
import socket
from threading import Thread
from time import sleep, time

import cv2
import matplotlib.pyplot as plt
import numpy as np
from djitellopy import Tello
from pyquaternion import Quaternion

from Point import Point, get_x_dimension, get_y_dimension
from auxiliary_functions import is_point_in_room, calculate_distance
from auxiliary_functions import p_closest

destination_point = None

FIFO_Images = "/tmp/images.fifo"
RequestSavePoints = '1'
PointsSaved = '2'
CheckSlamStatus = '3'
RequestSaveCurrentPoints = '4'
exit_loop = '5'

exit_stay_in_the_air_loop = False
CLOSE_THRESHOLD = 0.2
is_way_blocked = False
conn_loop_closer = None
getting_further = False
getting_closer = False
number_of_room = 0
rooms = {}
is_up_minus = True


class Drone(object):
    def __init__(self):
        self.drone = None
        self.stop = False
        self.stop_collect_points = False
        self.rotate_clockwise = True
        self.speed = 30
        self.drone_rotate = False
        self.conn_loop_closer = None
        self.getting_further = False
        self.getting_closer = False
        self.close_threshold = 0.1
        self.exit_stay_in_the_air_loop = False
        self.connect_drone()
        battery = self.drone.get_battery()
        print("before scan:" + str(battery))
        if battery > 25:
            Thread(target=os.system, args=(
                "gnome-terminal -x ~/ORB_SLAM2/ORB_SLAM2/Examples/Monocular/tello ~/ORB_SLAM2/ORB_SLAM2/Vocabulary/ORBvoc.txt ~/ORB_SLAM2/ORB_SLAM2/Examples/Monocular/tello_5f217e.yaml",),
                   daemon=True).start()
            Thread(target=self.stream_image_to_pipe, daemon=True).start()
            Thread(target=self.clear_map_data_in_tmp, daemon=True).start()
            self.conn = self.create_server()
            Thread(target=self.create_server_for_alerts, daemon=True).start()
        else:
            print("not enough battery")

    def __del__(self):
        print("im been destroyed")
        self.send_signal_to_slam(exit_loop)
        self.disconnect_drone()
        self.conn.close()

    def create_server_for_alerts(self):
        serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serv.bind(("127.0.0.1", 5555))
        serv.listen()
        self.conn_loop_closer, address = serv.accept()
        while True:
            if self.conn_loop_closer.recv(1).decode() == '1':
                print("loop closer happened")
                # self.stop = True
    def create_server(self):
        serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serv.bind(("127.0.0.1", 4444))
        serv.listen()
        conn, addr = serv.accept()
        return conn

    def save_map(self):
        try:
            print("- sending signal to save points -")
            self.send_signal_to_slam(RequestSavePoints)
            print("Map Saved")
        except Exception as e:
            print(e.args[0])

    def check_slam_status(self):
        with open("/tmp/slam_to_tello.txt", "r") as f:
            line = f.readline()
            if line.isdigit():
                return int(line) == 6
            else:
                return False

    def print_point(self, point, statement=""):
        print(statement + ",x:" + str(point.x) + ",y:" + str(point.y) + ",z:" + str(point.z))

    def update_scan_sum(self, current_sum, reverse, init_yaw):
        current_yaw = self.get_current_position_yaw()
        print("current_yaw:" + str(current_yaw))
        print("current sum:" + str(current_sum))
        current_yaw += 360 if current_yaw < 0 and current_sum > init_yaw else 0
        substraction = current_sum if current_sum < current_yaw else 0
        if reverse:
            current_sum += -1 * (current_yaw - substraction)
        else:
            current_yaw += 360 if current_yaw < current_sum and current_sum > 360 + init_yaw else 0
            current_sum += current_yaw - substraction

        return current_sum

    def right_angle(self, angle):
        if angle < 0:
            angle = 180 + (180 + angle)
        return angle

    def regular_scan(self):
        if self.check_slam_status():
            self.do_triangulation()
        i = 16
        while i != 0:
            self.how_to_rotate(25, self.rotate_clockwise)
            self.do_triangulation()
            i -= 1

    def begin_scan(self):
        self.regular_scan()
        self.drone.send_rc_control(0, 0, 0, 0)

    def drone_lost_location(self):
        self.how_to_rotate(25, not self.rotate_clockwise)
        sleep(1)
        self.drone.move_back(25)
        sleep(1)

    def do_triangulation(self):
        self.drone.move_forward(20)
        sleep(1)
        self.drone.move_back(20)
        sleep(1)

    def rotate_drone(self, angle, clockwise):
        if angle > 5:
            if clockwise:
                self.drone.rotate_clockwise(angle)
            else:
                self.drone.rotate_counter_clockwise(angle)
            sleep(1)
            amount_of_lost_localizations = 0
            while True:
                if self.check_localization():
                    amount_of_lost_localizations += 1
                    if not clockwise:
                        self.drone.rotate_clockwise(angle)
                    else:
                        self.drone.rotate_counter_clockwise(angle)
                    sleep(1)
                    self.drone.move_back(20)
                    sleep(1)
                else:
                    break
            while amount_of_lost_localizations != 0:
                self.do_triangulation()
                self.rotate_drone(angle, clockwise, True)
                amount_of_lost_localizations -= 1

    def get_angle(self, vector_1, vector_2):
        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        angle = int(math.degrees(np.arccos(dot_product)))
        clockwise = not (np.cross(unit_vector_1, unit_vector_2) > 0)
        if angle > 90:
            angle = 180 - angle
            clockwise = not clockwise
            print("angle is greater than 180")
        return angle, clockwise

    def get_current_position_yaw(self):
        current_quaternion = self.get_current_drone_position()
        yaw_current, pitch_current, roll_current = Quaternion(x=current_quaternion.qx, y=current_quaternion.qy,
                                                              z=current_quaternion.qz,
                                                              w=current_quaternion.qw).yaw_pitch_roll
        return math.degrees(yaw_current)

    def is_clockwise(self, yaw_desired):
        global is_up_minus
        current_quaternion = self.get_current_drone_position()
        yaw_current, pitch_current, roll_current = Quaternion(x=current_quaternion.qx, y=current_quaternion.qy,
                                                              z=current_quaternion.qz,
                                                              w=current_quaternion.qw).yaw_pitch_roll
        print("yaw current:" + str(math.degrees(yaw_current)))
        print("yaw desired:" + str(math.degrees(yaw_desired)))
        angle = math.degrees(yaw_desired) - math.degrees(yaw_current)
        if angle < 0:
            if angle >= -180:
                angle *= -1
                clockwise = not is_up_minus
            else:
                angle += 360
                clockwise = is_up_minus
        else:
            if angle >= 180:
                clockwise = is_up_minus
                angle = 360 - angle
            else:
                clockwise = not is_up_minus

        return int(angle), clockwise

    def get_frame_angle(self, desired_quaternion):
        yaw_desired, pitch_desired, roll_desired = Quaternion(x=desired_quaternion.qx, y=desired_quaternion.qy,
                                                              z=desired_quaternion.qz,
                                                              w=desired_quaternion.qw).yaw_pitch_roll
        return self.is_clockwise(yaw_desired)

    def how_to_rotate(self, angle, clockwise):
        self.drone_rotate = True
        angle = int(angle)
        if angle <= 30:
            self.rotate_drone(angle, clockwise)
        else:
            amount_of_rotations = int(angle / 25)
            for i in range(amount_of_rotations):
                self.rotate_drone(25, clockwise)
            self.rotate_drone(int(angle % 25), clockwise)
        self.drone_rotate = False

    def get_navigation_vectors(self, previous_position, checkpoint):
        current_drone_position = self.get_current_drone_position()
        drone_vector = np.array([previous_position.x - current_drone_position.x,
                                 previous_position.y - current_drone_position.y])
        exit_vector = np.array([checkpoint.x - current_drone_position.x,
                                checkpoint.y - current_drone_position.y])
        return drone_vector, exit_vector

    def navigate_to_checkpoint(self, point):
        angle, clockwise = self.get_frame_angle(point)
        self.how_to_rotate(angle, clockwise)
        sleep(1)
        while not self.stop:
            try:
                drone_previous_position = self.get_current_drone_position()
                sleep(3 + (0 if not self.getting_closer else 2))
                if self.getting_further and not self.stop:
                    self.how_to_rotate(180, True)
                    self.getting_further = False
                else:
                    if not self.stop and not self.drone_rotate:
                        drone_vector, exit_vector = self.get_navigation_vectors(drone_previous_position, point)
                        angle, self.rotate_clockwise = self.get_angle(drone_vector, exit_vector)
                        print("vector angle:" + str(angle))
                        self.how_to_rotate(angle, self.rotate_clockwise)
            except Exception as e:
                print(e.args[0])
                continue

    def stop_by_user(self):
        self.stop = False
        input("press enter to stop navigation")
        self.stop = True

    def reset_navigation_threads(self, monitor_drone_thread, navigate_to_checkpoint_thread):
        self.stop_monitor_drone = True
        self.reached_checkpoint = True
        self.stop_navigate_to_checkpoint = True
        monitor_drone_thread.join()
        navigate_to_checkpoint_thread.join()
        # reset thread interrupters
        self.loop_closer_happened = False
        self.stop_monitor_drone = False
        self.reached_checkpoint = False
        self.stop_navigate_to_checkpoint = False
        self.stop = False

    def navigate_drone(self, point):
        global getting_further
        global getting_closer
        angle, clockwise = self.get_frame_angle(point)
        self.how_to_rotate(angle, clockwise)
        sleep(1)
        if self.stop_by_user_thread is None:
            self.stop_by_user_thread = Thread(target=self.stop_by_user)
            self.stop_by_user_thread.start()
        monitor_drone_thread = Thread(target=self.monitor_drone, args=(point,))
        monitor_drone_thread.start()
        navigate_to_checkpoint_thread = Thread(target=self.navigate_to_checkpoint, args=(point,))
        navigate_to_checkpoint_thread.start()
        while True:
            try:
                if self.reached_checkpoint or self.loop_closer_happened or self.stop:
                    self.drone.send_rc_control(0, 0, 0, 0)
                    # stop threads
                    self.reset_navigation_threads(monitor_drone_thread,navigate_to_checkpoint_thread)
                    break
                if not self.drone_rotate:
                    self.drone.send_rc_control(0, self.speed if not getting_closer else int(self.speed / 2), 0, 0)
                    # self.drone.send_rc_control(0, 25, 0, 0)
                    sleep(2)
            except Exception as e:
                print(e.args[0])
                continue

    def monitor_drone(self, checkpoint):
        global getting_further
        global getting_closer
        self.reached_checkpoint = False
        self.stop_monitor_drone = False
        previous_distance = 10000
        while not self.stop_monitor_drone:
            try:
                drone_pos = self.get_current_drone_position()
                dist = calculate_distance(drone_pos, checkpoint)
                print("distance to checkpoint:" + str(dist))
                if dist <= CLOSE_THRESHOLD:
                    self.drone.send_rc_control(0, 0, 0, 0)
                    self.reached_checkpoint = True
                    print("Reached to checkpoint")
                    break
                getting_closer = dist <= CLOSE_THRESHOLD + 0.1
                getting_further = dist > previous_distance

                previous_distance = dist
                sleep(0.3)
            except Exception as e:
                print(e.args[0])
                continue

    def get_current_map(self):
        current_points = []
        with open("/tmp/currentPointData.csv", "r") as f:
            for row in f.readlines():
                values = row.split(",")
                current_points.append(np.array(values[:3], np.float32))
        return current_points

    def get_points_by_file_name(self, file_name):
        global number_of_room
        global rooms
        points = []
        with open(file_name, 'r') as file:
            for line in file.readlines():
                values = line.split(',')
                if number_of_room != 0:
                    point = Point(float(values[0]), float(values[2]), float(values[1]), float(values[3]),
                                  float(values[4]), float(values[5]), float(values[6]), float(values[7]))
                    for room in rooms.keys():
                        if room != number_of_room:
                            if not is_point_in_room(point, rooms[room].rectangle):
                                points.append(point)
                                break
                else:
                    points.append(
                        Point(float(values[0]), float(values[2]), float(values[1]), float(values[3]), float(values[4]),
                              float(values[5]), float(values[6]), -1, float(values[7])))
        return points

    def stay_in_the_air(self):
        global exit_stay_in_the_air_loop
        while True:
            if exit_stay_in_the_air_loop:
                break
            if self.check_slam_status():
                self.drone_lost_location()
            else:
                current_height = self.drone.get_height()
                if current_height > 30:
                    self.drone.move_down(20)
                    sleep(3)
                self.drone.move_up(20)
                sleep(3)

    def onclick(self, event, points):
        global exit_stay_in_the_air_loop
        global destination_point
        destination_point = None
        if event.button == 1:
            x = event.xdata
            y = event.ydata
            print("x:", str(x), "Y:", str(y))
            destination_point = p_closest(points, Point(x, y, 0), 1)
        exit_stay_in_the_air_loop = True
        plt.close()

    def save_current_map(self):
        print("- sending signal to save current frame points -")
        self.send_signal_to_slam(RequestSaveCurrentPoints)

    def get_current_map(self):
        self.save_current_map()
        current_points = []
        with open('/tmp/currentPointData.csv', 'r') as f:
            while True:
                row = f.readline()
                if row != '':
                    values = row.split(',')
                    current_points.append(
                        Point(float(values[0]), float(values[2]), float(values[1]), float(values[3]), float(values[4]),
                              float(values[5]), float(values[6]), int(values[7])))
                else:
                    break
        return current_points

    def navigate_on_click(self):
        self.save_map()
        current_position = self.get_current_drone_position()
        points = self.get_newest_map_points()
        current_points = self.get_current_map()
        fig, ax = plt.subplots()
        point_for_plot = []
        lower_points = []
        for point in points:
            if point not in current_points:
                if point.z > current_position.z:
                    lower_points.append(point)
                else:
                    point_for_plot.append(point)
        ax.scatter(get_x_dimension(point_for_plot), get_y_dimension(point_for_plot), linewidth=0.1, s=1)
        ax.scatter(get_x_dimension(lower_points), get_y_dimension(lower_points), linewidth=0.1, s=1,
                   color=['purple'])
        ax.scatter(get_x_dimension(current_points), get_y_dimension(current_points), linewidth=0.1, s=1,
                   color=['green'])
        current_position = self.get_current_drone_position()
        plt.plot(current_position.x, current_position.y, 'rx')
        fig.canvas.mpl_connect('button_press_event', lambda event: self.onclick(event, points))
        plt.show()

    def map_and_exit(self):
        global exit_stay_in_the_air_loop
        global destination_point
        self.drone.takeoff()
        while True:
            self.begin_scan()
            battery_level = self.drone.get_battery()
            print("battery after scan:" + str(battery_level))
            self.save_map()
            self.reached_checkpoint = False
            stay_in_the_air_thread = None
            while True:
                if not self.reached_checkpoint:
                    if stay_in_the_air_thread is None:
                        stay_in_the_air_thread = Thread(target=self.stay_in_the_air)
                        stay_in_the_air_thread.start()
                    self.navigate_on_click()
                    if destination_point is not None:
                        stay_in_the_air_thread.join()
                        stay_in_the_air_thread = None
                        exit_stay_in_the_air_loop = False
                        self.navigate_drone(destination_point)
                        self.reached_checkpoint = False
                    battery_level = self.drone.get_battery()
                    print("battery after navigation:" + str(battery_level))

            self.disconnect_drone()
            user_command = input("press enter twice to resume scan, input s to stop")
            if user_command == 's':
                break
            self.connect_drone()

        return

    def stream_image_to_pipe(self):
        out = cv2.VideoWriter('outpy.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 20, (960, 720))
        frame_read = self.drone.get_frame_read()
        try:
            os.mkfifo(FIFO_Images)
        except Exception as e:
            print(e.args[0])
        while True:
            try:
                frame = frame_read.frame
                out.write(frame)
                with open(FIFO_Images, 'wb') as f:
                    f.write(cv2.resize(frame, (960, 720)).tobytes())
            except Exception as e:
                print(e.args[0])
                continue
        out.release()

    def clear_map_data_in_tmp(self):
        for filename in glob.glob("/tmp/pointData*.csv"):
            os.remove(filename)

    def get_newest_map_points(self):
        maps = glob.glob("/tmp/pointData*.csv")
        max_index = 0
        for map in maps:
            index = int(map.split("pointData")[1].split(".")[0])
            max_index = index if index > max_index else max_index
        return self.get_points_by_file_name("/tmp/pointData" + str(max_index) + ".csv")

    def get_current_drone_position(self):
        for i in range(3):
            try:
                with open("/tmp/tello_last_location.csv", "r") as f:
                    values = f.readline().split(",")
                    return Point(float(values[0]), float(values[2]), float(values[1]), float(values[3]),
                                 float(values[4]),
                                 float(values[5]), float(values[6]))
            except Exception as e:
                print(e.args[0])
                sleep(0.05)
                continue

    def connect_drone(self):
        self.drone = Tello()
        if not self.drone.connect():
            raise RuntimeError("Tello not connected")
        self.drone.streamon()

    def disconnect_drone(self):
        self.drone.land()
        sleep(5)
        self.drone.streamoff()
        self.drone = None

    def send_signal_to_slam(self, message):
        self.conn.send(str(message).encode())
        answer = self.conn.recv(1).decode()
        print("we received from socket:" + str(answer))
        return answer


def main():
    drone = Drone()
    drone.map_and_exit()


if __name__ == '__main__':
    timer = time()
    main()
    print(time() - timer)

""" """
""""""
