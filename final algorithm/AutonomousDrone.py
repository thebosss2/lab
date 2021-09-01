import glob
import math
import os
import socket
from enum import Enum
from threading import Thread
from time import sleep, time

import cv2
import numpy as np
from djitellopy import Tello
from pyquaternion import Quaternion

from Frame import Frame
from Point import Point
from auxiliary_functions import is_point_in_room, calculate_distance
from collision_detector import get_details_about_ray
from corridor_identification import corridor_identification_by_frame
# from scan_floor import get_polygon_checkpoints
from source import get_exit_point
import sys

sys.path.append('~/lab')

from algorithm import exit_algorithm

RequestSavePoints = '1'
PointsSaved = '2'
CheckSlamStatus = '3'
RequestSaveCurrentPoints = '4'
exit_loop = '5'


class Drone(object):
    def __init__(self):
        self.drone_modes = Enum('drone_modes', 'scanning navigation')
        self.current_drone_mode = self.drone_modes.scanning
        self.drone = None
        self.stop = False
        self.low_battery = False
        self.is_exit = True
        self.rotate_clockwise = True
        self.speed = 20
        self.conn = None
        self.rooms = {}
        self.frames = []
        self.where_we_lost_localization = {}
        self.current_room = None
        self.is_up_minus = None
        self.drone_rotate = False
        self.looking_back = False
        self.right_side_blocked = False
        self.left_side_blocked = False
        self.conn_loop_closer = None
        self.loop_closer_happened = False
        self.reached_checkpoint = False
        self.getting_further = False
        self.getting_closer = False
        self.current_frame = None
        self.polygon_checkpoints = []
        self.close_threshold = 0.4
        self.exit_stay_in_the_air_loop = False
        self.connect_drone()
        battery = self.drone.get_battery()
        print("before scan:" + str(battery))
        if battery > 30:
            Thread(target=os.system, args=(
                "gnome-terminal -x ~/ORB_SLAM2/Examples/Monocular/mono_tum ~/ORB_SLAM2/Vocabulary/ORBvoc.txt ~/ORB_SLAM2/Examples/Monocular/tello.yaml",),
                   daemon=True).start()
            Thread(target=self.stream_image_to_pipe, daemon=True).start()
            Thread(target=self.clear_map_data_in_tmp, daemon=True).start()
            self.create_server()
            Thread(target=self.create_server_for_alerts, daemon=True).start()
        else:
            print("not enough battery")

    def __del__(self):
        print("im been destroyed")
        self.send_signal_to_slam(exit_loop)
        self.disconnect_drone()
        self.conn.close()

    def create_server(self):
        serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serv.bind(("127.0.0.1", 4444))
        serv.listen()
        self.conn, address = serv.accept()

    def create_server_for_alerts(self):
        serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serv.bind(("127.0.0.1", 5555))
        serv.listen()
        self.conn_loop_closer, address = serv.accept()
        while True:
            if self.conn_loop_closer.recv(1).decode() == '1':
                print("loop closer happened")
                self.loop_closer_happened = True

    def save_map(self):
        print("- sending signal to save points -")
        self.send_signal_to_slam(RequestSavePoints)
        answer = self.conn.recv(1).decode('utf-8')
        print("we received from socket:" + answer)

    def save_current_map(self):
        print("- sending signal to save current frame points -")
        self.send_signal_to_slam(RequestSaveCurrentPoints)
        answer = self.conn.recv(1).decode('utf-8')
        print("we received from socket:" + answer)
        return answer

    def get_current_map(self):
        self.save_current_map()
        sleep(0.5)
        for i in range(3):
            try:
                with open('/tmp/currentPointData.csv', 'r') as f:
                    row = f.readline()
                    values = row.split(',')
                    frame = Frame(float(values[8]), float(values[10]), float(values[9]), float(values[3]),
                                  float(values[4]),
                                  float(values[5]), float(values[6]), int(values[7]))
                    while True:
                        frame.points.append(
                            Point(float(values[0]), float(values[2]), float(values[1]), float(values[3]),
                                  float(values[4]),
                                  float(values[5]), float(values[6]), int(values[7])))
                        row = f.readline()
                        if ',' not in row:
                            break
                        values = row.split(',')
                return frame
            except Exception as e:
                print(e.args[0] + "in get_current_map")
                continue

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
        subtraction = current_sum if current_sum < current_yaw else 0
        if reverse:
            current_sum += -1 * (current_yaw - subtraction)
        else:
            current_yaw += 360 if current_yaw < current_sum and current_sum > 360 + init_yaw else 0
            current_sum += current_yaw - subtraction

        return current_sum

    def check_current_frame(self, frame, save_img=False):
        if not self.check_slam_status() and frame is not None:
            if save_img:
                cv2.imwrite("./corridor_frames/frame" + str(frame.frame_id) + ".jpg", self.current_frame)
            if len(frame.points) > 0:
                ray_points, ray_line_segment = corridor_identification_by_frame(frame.points)
                if ray_points is not None and len(ray_points) > 10 and ray_line_segment[0] is not None:
                    return get_details_about_ray(ray_points, self.get_current_drone_position(), ray_line_segment,
                                                 self.is_up_minus)
                else:
                    print("couldn't get ray from function")
        return None, None

    def init_scan_variables(self):
        self.right_side_blocked, self.left_side_blocked = False, False
        self.where_we_lost_localization = {}
        self.frames = []

    def find_corridor(self):
        self.right_side_blocked, self.left_side_blocked = False, False
        if self.is_up_minus is None:
            self.get_z_axis_direction()
        for frame in self.frames:
            self.drone.move_up(20)
            is_wall, checkpoint = self.check_current_frame(frame)
            if not is_wall:
                time_of_scan = time()
                self.navigate_drone(checkpoint)
                total_navigation_time = time() - time_of_scan
                if calculate_distance(self.get_current_drone_position(), checkpoint) < self.close_threshold:
                    break
                else:
                    while total_navigation_time > 0:
                        self.drone.send_rc_control(0, -self.speed, 0, 0)
                        sleep(2)
                        total_navigation_time -= 2
            self.drone.move_down(20)

    def add_frame_to_frames(self):
        frame = self.get_current_map()
        if frame is not None:
            self.frames.append(frame)
            """is_wall, check_point = self.check_current_frame(frame)
            f = open("frames_details.txt", "a")
            f.write(
                "Frame Id: " + str(frame.frame_id) + ", Is Wall: " + str(is_wall) + ",distance to checkpoint:" + str(
                    calculate_distance(self.get_current_drone_position(), check_point)) + '\n')
            f.close()"""
        else:
            print("couldn't read frame from file")
        return frame

    def regular_scan(self):
        self.current_drone_mode = self.drone_modes.scanning
        self.drone.move_down(30)
        if self.check_slam_status():
            self.do_triangulation()
        i = 16
        self.init_scan_variables()
        while i != 0:
            self.how_to_rotate(25, self.rotate_clockwise, True)
            i -= 1

    def do_triangulation(self):
        self.drone.move_forward(30)
        sleep(1)
        self.drone.move_back(30)
        sleep(1)

    def get_z_axis_direction(self):
        prev_pos = self.get_current_drone_position()
        self.drone.move_up(20)
        sleep(1.5)
        self.is_up_minus = prev_pos.z > self.get_current_drone_position().z
        self.drone.move_down(20)
        sleep(1.5)
        print("is_up_is_minus:" + str(self.is_up_minus))

    def begin_scan(self):
        self.regular_scan()
        if self.is_up_minus is None:
            self.get_z_axis_direction()
        self.drone.send_rc_control(0, 0, 0, 0)

    def drone_lost_location(self):
        self.how_to_rotate(25, not self.rotate_clockwise)
        sleep(1)
        self.drone.move_back(25)
        sleep(1)

    def alert_low_battery(self):
        self.low_battery = False
        while True:
            battery = self.drone.get_battery()
            if battery < 25:
                self.low_battery = True
                self.stop = True
                sleep(10)
                self.disconnect_drone()
                break
            sleep(20)

    def fly_to_polygon_edges(self):
        current_pos = self.get_current_drone_position()
        for checkpoint in self.polygon_checkpoints:
            if self.navigate_drone(checkpoint):
                self.how_to_rotate(180, True, True)
                if self.loop_closer_happened:
                    break
                self.navigate_drone(current_pos, False)
            else:
                break

    def fly_to_checkpoints(self, room):
        if not self.is_exit:
            current_pos = self.get_current_drone_position()
        for checkpoint, best_segment in room.exit_points:
            success = self.navigate_drone(checkpoint)
            if not success:
                self.navigate_drone(checkpoint)

            if not self.is_exit:
                self.save_map()
                self.navigate_drone(current_pos)

    def check_if_we_can_go_back(self):
        print("check if we can go back")
        current_yaw = self.get_current_position_yaw()
        current_yaw += 360 if current_yaw < 0 else 0
        print(current_yaw)
        for frame in self.frames:
            frame_yaw = self.get_yaw_from_point(frame)
            frame_yaw += 360 if frame_yaw < 0 else 0
            print(abs(current_yaw - frame_yaw))
            if 200 > abs(current_yaw - frame_yaw) > 160:
                return True
        print("didnt found a frame 180 backwards")
        return False

    def look_back(self, previous_rotation_direction):
        print("start looking back")
        previous_yaw = self.get_current_position_yaw()
        previous_yaw += 360 if previous_yaw < 0 else 0
        self.looking_back = True
        self.how_to_rotate(180, not previous_rotation_direction, True)
        self.looking_back = False
        print("stop looking back")
        self.rotate_clockwise = not self.rotate_clockwise
        current_yaw = self.get_current_position_yaw()
        current_yaw += 360 if current_yaw < 0 else 0
        return previous_yaw + 160 < current_yaw < previous_yaw + 200

    def manage_lost_localization_counter(self, last_know_yaw):
        for angle in self.where_we_lost_localization.keys():
            if angle - 30 < last_know_yaw < angle + 30:
                self.where_we_lost_localization[angle] += 1
                return angle

        self.where_we_lost_localization[last_know_yaw] = 1
        return last_know_yaw

    def rotate_drone(self, angle, clockwise, build_map=False):
        if not self.low_battery:
            if angle > 5:
                if clockwise:
                    self.drone.rotate_clockwise(angle)
                else:
                    self.drone.rotate_counter_clockwise(angle)
                sleep(1)
                amount_of_lost_localizations = 0
                if self.check_slam_status():
                    self.drone.move_back(30)
                    while True:
                        if self.check_slam_status():
                            if not clockwise:
                                self.drone.rotate_clockwise(angle)
                            else:
                                self.drone.rotate_counter_clockwise(angle)
                            sleep(1)
                            amount_of_lost_localizations += 1
                        else:
                            break

                if build_map:
                    self.do_triangulation()
                if amount_of_lost_localizations > 0:
                    while amount_of_lost_localizations != 0:
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
        return angle, clockwise

    def get_yaw_from_point(self, point):
        yaw_current, pitch_current, roll_current = Quaternion(x=point.qx, y=point.qy,
                                                              z=point.qz,
                                                              w=point.qw).yaw_pitch_roll
        return math.degrees(yaw_current)

    def get_current_position_yaw(self):
        current_quaternion = self.get_current_drone_position()
        return self.get_yaw_from_point(current_quaternion)

    def is_clockwise(self, yaw_desired):
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
                clockwise = self.is_up_minus
            else:
                angle += 360
                clockwise = not self.is_up_minus
        else:
            if angle >= 180:
                clockwise = not self.is_up_minus
                angle = 360 - angle
            else:
                clockwise = self.is_up_minus

        return int(angle), clockwise

    def get_frame_angle(self, desired_quaternion):
        yaw_desired, pitch_desired, roll_desired = Quaternion(x=desired_quaternion.qx, y=desired_quaternion.qy,
                                                              z=desired_quaternion.qz,
                                                              w=desired_quaternion.qw).yaw_pitch_roll
        return self.is_clockwise(yaw_desired)

    def how_to_rotate(self, angle, clockwise, build_map=False):
        self.drone_rotate = True
        angle = int(angle)
        if angle <= 30:
            self.rotate_drone(angle, clockwise, build_map)
        else:
            amount_of_rotations = int(angle / 25)
            for i in range(amount_of_rotations):
                if self.low_battery:
                    break
                self.rotate_drone(25, clockwise, build_map)
            self.rotate_drone(int(angle % 25), clockwise)
        self.drone_rotate = False

    def get_navigation_vectors(self, previous_position, checkpoint):
        current_drone_position = self.get_current_drone_position()
        if current_drone_position == previous_position:
            sleep(0.2)
            current_drone_position = self.get_current_drone_position()
        drone_vector = np.array([previous_position.x - current_drone_position.x,
                                 previous_position.y - current_drone_position.y])
        exit_vector = np.array([checkpoint.x - current_drone_position.x,
                                checkpoint.y - current_drone_position.y])
        return drone_vector, exit_vector

    def navigate_to_checkpoint(self, point, rotate_to_frame_angle=True):
        if rotate_to_frame_angle:
            angle, clockwise = self.get_frame_angle(point)
            self.how_to_rotate(angle, clockwise)
        sleep(2)
        while not self.stop and not self.low_battery:
            try:
                drone_previous_position = self.get_current_drone_position()
                sleep(5 + (0 if not self.getting_closer else 2))
                if self.getting_further and not self.stop:
                    self.how_to_rotate(180, True)
                    self.getting_further = False
                elif not self.stop and not self.drone_rotate:
                    drone_vector, exit_vector = self.get_navigation_vectors(drone_previous_position, point)
                    angle, self.rotate_clockwise = self.get_angle(drone_vector, exit_vector)
                    print("vector angle:" + str(angle))
                    self.how_to_rotate(angle, self.rotate_clockwise)
            except Exception as e:
                print(e.args[0])
                continue

    def lower_drone(self):
        low_height = 20
        current_height = self.drone.get_height()
        needed_decedent = current_height - low_height
        if needed_decedent > 20:
            self.drone.move_down(needed_decedent)
            sleep(1)

    def lost_localization_during_scan(self, one_time=False):
        if one_time:
            self.lower_drone()
        while not one_time and not self.reached_checkpoint:
            self.lower_drone()

    def reset_navigation_threads(self, monitor_drone_thread, navigate_to_checkpoint_thread):
        self.drone.send_rc_control(0, 0, 0, 0)
        if monitor_drone_thread is not None:
            monitor_drone_thread.join()
        if navigate_to_checkpoint_thread is not None:
            navigate_to_checkpoint_thread.join()
        # collision_detector_thread.join()
        self.stop = False

    def getaway_from_wall(self, point, frame_id):
        print("we are about to hit a wall")
        for i in range(2):
            self.drone.send_rc_control(0, -self.speed, 0, 0)
            sleep(2)
        os.rename("/tmp/currentPointData.csv", "/tmp/currentPointData" + str(frame_id) + ".csv")
        cv2.imwrite("./wall_frames/frame" + str(frame_id) + ".jpg", self.current_frame)
        angle, clockwise = self.get_frame_angle(point)
        self.how_to_rotate(angle, clockwise)

    def collision_detector(self, point):
        lost_localiztion_wall = 0
        while not self.stop:
            sleep(1)
            if self.check_slam_status():
                self.getaway_from_wall(point, lost_localiztion_wall)
                lost_localiztion_wall += 1
            frame = self.add_frame_to_frames()
            if frame is not None:
                is_wall, checkpoint = self.check_current_frame(frame)
                current_pos = self.get_current_drone_position()
                if checkpoint is not None and current_pos is not None:
                    if is_wall:
                        distance = calculate_distance(current_pos,
                                                      checkpoint)
                        self.drone_rotate = True
                        if distance < self.close_threshold * 3:
                            self.getaway_from_wall(point, frame.frame_id)
                        self.drone_rotate = False
                        sleep(1)
            else:
                print("couldn't read frame file collision detector")

    def navigate_drone(self, point, rotate_to_frame_angle=True):
        self.stop = False
        self.loop_closer_happened = False
        self.current_drone_mode = self.drone_modes.navigation
        monitor_drone_thread = None
        navigate_to_checkpoint_thread = Thread(target=self.navigate_to_checkpoint, args=(point, rotate_to_frame_angle))
        navigate_to_checkpoint_thread.start()
        sleep(2)
        while (not self.stop or monitor_drone_thread is None) and \
                not self.loop_closer_happened and not self.low_battery:
            try:
                if not self.drone_rotate:
                    self.drone.send_rc_control(0, self.speed if not self.getting_closer else 10, 0, 0)
                    if monitor_drone_thread is None:
                        monitor_drone_thread = Thread(target=self.monitor_drone, args=(point,))
                        monitor_drone_thread.start()
                    sleep(2)
            except Exception as e:
                print(e.args[0])
                continue
        self.drone.send_rc_control(0, 0, 0, 0)
        self.stop = True
        self.reset_navigation_threads(monitor_drone_thread, navigate_to_checkpoint_thread)
        leftover_distance = calculate_distance(self.get_current_drone_position(), point)
        print("distance to checkpoint before continue: " + str(leftover_distance))
        return leftover_distance < self.close_threshold * 1.5

    def monitor_drone(self, checkpoint):
        previous_distance = 10000
        sleep(0.5)
        i = 0
        while not self.stop:
            try:
                drone_pos = self.get_current_drone_position()
                dist = calculate_distance(drone_pos, checkpoint)
                if i % 5 == 0:
                    print("distance to checkpoint:" + str(dist))
                if dist <= self.close_threshold:
                    self.stop = True
                    print("Reached to checkpoint")
                    break
                if not self.drone_rotate:
                    self.getting_closer = dist <= self.close_threshold * 1.5
                    self.getting_further = dist > previous_distance

                previous_distance = dist
                sleep(0.3)
                i += 1
            except Exception as e:
                print(e.args[0])
                continue

    def save_points_to_room(self, file_name, current_room):
        number_of_room = len(self.rooms.keys()) - 1
        with open(file_name, 'r') as file:
            for line in file.readlines():
                values = line.split(',')
                point = Point(float(values[0]), float(values[2]), float(values[1]), float(values[3]),
                              float(values[4]), float(values[5]), float(values[6]), float(values[7]))
                if number_of_room != 0:
                    for room in self.rooms.keys():
                        if room != number_of_room:
                            if not is_point_in_room(point, self.rooms[room].rectangle):
                                current_room.points.append(point)
                                if point.frame_id not in current_room.frames.keys():
                                    current_room.frames[point.frame_id] = Frame(float(values[8]), float(values[10]),
                                                                                float(values[9]), point.qx, point.qy,
                                                                                point.qz, point.qw, point.frame_id)
                                current_room.frames[point.frame_id].points.append(point)
                                break
                else:
                    current_room.points.append(point)
                    if point.frame_id not in current_room.frames.keys():
                        current_room.frames[point.frame_id] = Frame(float(values[8]), float(values[10]),
                                                                    float(values[9]), point.qx, point.qy,
                                                                    point.qz, point.qw, point.frame_id)
                    current_room.frames[point.frame_id].points.append(point)

        return current_room

    def stay_in_the_air(self, is_exit_or_polygon=True):
        # if is_exit_or_polygon:
        #     Thread(targeta=self.get_exit_points).start()
        # # else:
        Thread(target=self.get_polygon_checkpoints).start()
        while not self.low_battery:
            if self.exit_stay_in_the_air_loop:
                self.exit_stay_in_the_air_loop = False
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

    def get_exit_points(self):
        self.current_room = self.get_newest_room(self.current_room)
        checkpoints_indexes, self.current_room.rectangle = get_exit_point(self.current_room.points, self.is_up_minus,
                                                                          self.is_exit)
        if checkpoints_indexes is None:
            print("No checkpoints received after scan")
            self.exit_stay_in_the_air_loop = True
            return
        for point, best_segment in checkpoints_indexes:
            self.current_room.exit_points.append((point, best_segment))
        print("Navigate to checkpoint after scan")
        self.exit_stay_in_the_air_loop = True

    def get_polygon_checkpoints(self):
        exit_point = exit_algorithm(self.get_newest_map_name())
        self.polygon_checkpoints = [Point(exit_point[0], exit_point[1], exit_point[2])]
        self.exit_stay_in_the_air_loop = True

    def map_and_exit(self):
        self.drone.takeoff()
        """number_of_room = 0
        self.rooms[number_of_room] = Room()
        self.current_room = self.rooms[number_of_room]
        print("number of room:" + str(number_of_room))
        self.begin_scan()
        battery_level = self.drone.get_battery()
        print("battery after scan:" + str(battery_level))
        self.drone.move_up(30)
        sleep(3)
        self.drone.move_down(60)
        sleep(3)
        self.save_map()
        self.stay_in_the_air(True)
        self.fly_to_checkpoints(self.rooms[number_of_room])"""
        battery_thread = Thread(target=self.alert_low_battery, daemon=True)
        battery_thread.start()
        self.begin_scan()
        if not self.loop_closer_happened:
            self.drone.move_up(30)
            sleep(3)
            self.drone.move_down(60)
            sleep(3)
        while True:
            if not self.low_battery:
                self.save_map()
                self.stay_in_the_air(False)
                self.fly_to_polygon_edges()
            if self.low_battery:
                battery_thread.join()
                user_command = input("press enter twice to resume scan, input s to stop")
                if user_command == 's':
                    break
                self.drone = None
                sleep(4)
                self.connect_drone()
                battery_thread = Thread(target=self.alert_low_battery, daemon=True)
                battery_thread.start()
                self.drone.takeoff()
        return

    def stream_image_to_pipe(self):
        fifo_images = "/tmp/images.fifo"
        out = cv2.VideoWriter('outpy.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 20, (960, 720))
        try:
            os.mkfifo(fifo_images)
        except Exception as e:
            print(e.args[0])
        while True:
            if not self.low_battery:
                frame_read = self.drone.get_frame_read()
                while not self.low_battery:
                    try:
                        if not self.low_battery:
                            self.current_frame = frame_read.frame
                        out.write(self.current_frame)
                        with open(fifo_images, 'wb') as f:
                            f.write(cv2.resize(self.current_frame, (960, 720)).tobytes())
                    except Exception as e:
                        print(e.args[0])
                        continue
            sleep(1)
        out.release()

    def clear_map_data_in_tmp(self):
        for filename in glob.glob("/tmp/pointData*.csv"):
            os.remove(filename)
        for filename in glob.glob("/tmp/currentPointData*.csv"):
            os.remove(filename)
        for filename in glob.glob("./corridor_frames/frame*.csv"):
            os.remove(filename)
        for filename in glob.glob("./wall_frames/frame*.csv"):
            os.remove(filename)

    def get_newest_room(self, current_room):
        return self.save_points_to_room(self.get_newest_map_name(), current_room)

    def get_newest_map_name(self):
        maps = glob.glob("/tmp/pointData*.csv")
        max_index = 0
        for map_path in maps:
            index = int(map_path.split("pointData")[1].split(".")[0])
            max_index = index if index > max_index else max_index
        return "/tmp/pointData" + str(max_index) + ".csv"

    def get_current_drone_position(self):
        for i in range(3):
            try:
                with open("/tmp/tello_last_location.csv", "r") as f:
                    values = f.readline().split(",")
                    point = Point(float(values[0]), float(values[2]), float(values[1]), float(values[3]),
                                  float(values[4]),
                                  float(values[5]), float(values[6]))
                    if point is not None:
                        return point
                    else:
                        continue
            except Exception as e:
                print(e.args[0])
                sleep(0.05)
                continue

    def connect_drone(self):
        self.drone = Tello()
        self.drone.connect()
        self.drone.streamon()

    def disconnect_drone(self):
        self.drone.land()
        sleep(5)
        self.drone.streamoff()

    def send_signal_to_slam(self, message):
        self.conn.send(str(message).encode())


def main():
    drone = Drone()
    drone.map_and_exit()


if __name__ == '__main__':
    main()
