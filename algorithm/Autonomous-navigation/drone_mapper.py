import glob
import math
import os
import socket
from threading import Thread
from time import sleep

import cv2
import numpy as np
from djitellopy import Tello
from pyquaternion import Quaternion

from Frame import Frame
from Point import Point
from Room import Room
from auxiliary_functions import is_point_in_room, calculate_distance
from corridor_identification import is_corridor_to_room
from source import get_exit_point

FIFO_Images = "/tmp/images.fifo"
RequestSavePoints = '1'
PointsSaved = '2'
CheckSlamStatus = '3'
RequestSaveCurrentPoints = '4'
exit_loop = '5'
reached_checkpoint = False
exit_stay_in_the_air_loop = False
CLOSE_THRESHOLD = 0.4
is_way_blocked = False
navigate_to_checkpoint = True
rotate_clockwise = True
conn = None
conn_loop_closer = None
is_exit = True
getting_further = False
number_of_room = 0
rooms = {}
is_up_minus = None
drone_rotate = False
current_frame = None
getting_closer = False
loop_closer_happened = False


class Drone(object):
    def __init__(self):
        self.drone = None
        self.drone_speed = 25
        self.connect_drone()
        battery = self.drone.get_battery()
        print("before scan:" + str(battery))
        if battery > 50:
            Thread(target=os.system, args=(
                "gnome-terminal -x ~/ORB_SLAM3/Examples/Monocular/tello ~/ORB_SLAM3/Vocabulary/ORBvoc.txt ~/ORB_SLAM3/Examples/Monocular/tello.yaml",),
                   daemon=True).start()
            Thread(target=self.stream_image_to_pipe, daemon=True).start()
            Thread(target=self.clear_map_data_in_tmp, daemon=True).start()
            self.create_server()
            #Thread(target=self.create_server_for_alerts, daemon=True).start()
        else:
            print("not enough battery")

    def __del__(self):
        global conn
        print("im been destroyed")
        self.send_signal_to_slam(exit_loop)
        self.disconnect_drone()
        conn.close()

    def create_server(self):
        global conn
        serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serv.bind(("127.0.0.1", 4444))
        serv.listen()
        conn, address = serv.accept()

    def create_server_for_alerts(self):
        global conn_loop_closer
        global loop_closer_happened
        serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serv.bind(("127.0.0.1", 5555))
        serv.listen()
        conn_loop_closer, addr = serv.accept()
        while True:
            if conn_loop_closer.recv(1).decode() == '1':
                loop_closer_happened = True
                sleep(2)
                loop_closer_happened = False

    def save_map(self):
        print("- sending signal to save points -")
        self.send_signal_to_slam(RequestSavePoints)
        answer = conn.recv(1).decode('utf-8')
        print("we received from socket:" + answer)

    def save_current_map(self):
        print("- sending signal to save current frame points -")
        self.send_signal_to_slam(RequestSaveCurrentPoints)
        answer = conn.recv(1).decode('utf-8')
        print("we received from socket:" + answer)

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

    def right_angle(self, angle):
        if angle < 0:
            angle = 180 + (180 + angle)
        return angle

    def navigate_to_angle(self, desired_angle):
        current_angle = self.get_current_position_yaw()
        speed = -15
        rotate_clockwise = 10 * (1 if self.is_clockwise(desired_angle)[1] else -1)
        desired_angle_0_360 = self.right_angle(desired_angle)
        first_angle_0_360 = self.right_angle(current_angle)
        is_first_plus_desired_minus = desired_angle_0_360 < 0 and first_angle_0_360 > 0
        is_first_minus_desired_plus = desired_angle_0_360 > 0 and first_angle_0_360 < 0
        while True:
            self.drone.send_rc_control(0, speed, 0, rotate_clockwise)
            current_angle = self.get_current_position_yaw()
            current_angle_0_360 = self.right_angle(current_angle)
            if rotate_clockwise > 0:
                if is_first_plus_desired_minus and 180 < current_angle_0_360 <= desired_angle_0_360:
                    break
                else:
                    if current_angle_0_360 <= desired_angle_0_360:
                        break
            else:
                if is_first_minus_desired_plus and 180 > current_angle_0_360 >= desired_angle_0_360:
                    break
                else:
                    if current_angle_0_360 >= desired_angle_0_360:
                        break
            sleep(1)
            # before_current = current_angle_0_360
            speed *= -1

    def create_map(self):

        while True:
            if self.check_slam_status():
                self.drone.move_forward(25)
                sleep(1)
                self.drone.move_back(25)
                sleep(1)
                self.drone.rotate_counter_clockwise(25)
                sleep(1.5)
            else:
                break
        start_yaw = self.get_current_position_yaw()
        checkpoint = start_yaw + 180 * (-1 if start_yaw > 0 else 1)
        for point in [checkpoint, start_yaw]:
            self.navigate_to_angle(point)

    def check_current_frame(self):
        if not self.check_slam_status():
            current_frame_points = self.get_current_map()
            if len(current_frame_points) > 0:
                is_corridor_to_room(current_frame_points, self.get_current_drone_position(), current_frame)

    def regular_scan(self):
        global rotate_clockwise
        global current_frame
        global is_up_minus
        self.lost_localization_during_scan(True)
        sleep(2)
        if self.check_slam_status():
            self.do_triangulation()
        i = 16
        self.check_current_frame()
        while i != 0:
            self.how_to_rotate(25, rotate_clockwise)
            self.do_triangulation()
            self.check_current_frame()
            i -= 1

    def do_triangulation(self):
        self.drone.move_forward(20)
        sleep(1)
        self.drone.move_back(20)
        sleep(1)

    def get_z_axis_direction(self):
        global is_up_minus
        prev_pos = self.get_current_drone_position()
        self.drone.move_up(20)
        sleep(1.5)
        is_up_minus = prev_pos.z > self.get_current_drone_position().z
        self.drone.move_down(20)
        sleep(1.5)

    def begin_scan(self):
        global is_up_minus
        self.regular_scan()
        if is_up_minus is None:
            self.get_z_axis_direction()
        self.drone.send_rc_control(0, 0, 0, 0)

    def drone_lost_location(self):
        global rotate_clockwise
        self.how_to_rotate(25, not rotate_clockwise)
        sleep(1)
        self.drone.move_back(25)
        sleep(1)
        # current_frame_points = self.get_current_map()
        """if len(current_frame_points) != 0:
            twoD("/tmp/currentPointData.csv")"""

    def fly_to_checkpoints(self, room):
        global is_exit
        if not is_exit:
            current_pos = self.get_current_drone_position()
        for checkpoint, best_segment in room.exit_points:
            self.navigate_drone(checkpoint)
            self.drone.send_rc_control(0, 0, 0, 0)
            if not is_exit:
                # self.begin_scan()
                self.save_map()
                self.navigate_drone(current_pos, True)

    def rotate_clockwise(self, angle, clockwise):
        if angle > 5:
            if clockwise:
                self.drone.rotate_clockwise(angle)
            else:
                self.drone.rotate_counter_clockwise(angle)
            sleep(1)
            amount_of_lost_localizations = 0
            while True:
                if self.check_slam_status():
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
            if amount_of_lost_localizations != 0:
                self.do_triangulation()
            while amount_of_lost_localizations != 0:
                self.rotate_clockwise(angle, clockwise)
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
                clockwise = is_up_minus
            else:
                angle += 360
                clockwise = not is_up_minus
        else:
            if angle >= 180:
                clockwise = not is_up_minus
                angle = 360 - angle
            else:
                clockwise = is_up_minus

        return int(angle), clockwise

    def get_frame_angle(self, desired_quaternion):
        yaw_desired, pitch_desired, roll_desired = Quaternion(x=desired_quaternion.qx, y=desired_quaternion.qy,
                                                              z=desired_quaternion.qz,
                                                              w=desired_quaternion.qw).yaw_pitch_roll
        return self.is_clockwise(yaw_desired)

    def how_to_rotate(self, angle, clockwise):
        global drone_rotate
        drone_rotate = True
        angle = int(angle)
        if angle <= 30:
            self.rotate_clockwise(angle, clockwise)
        else:
            amount_of_rotations = int(angle / 25)
            for i in range(amount_of_rotations):
                self.rotate_clockwise(25, clockwise)
            self.rotate_clockwise(int(angle % 25), clockwise)
        drone_rotate = False

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

    def navigate_to_checkpoint(self, point):
        global getting_further
        global navigate_to_checkpoint
        global rotate_clockwise
        global getting_closer
        while navigate_to_checkpoint:
            try:
                drone_previous_position = self.get_current_drone_position()
                sleep(3 + (0 if not getting_closer else 2))
                if getting_further:
                    self.how_to_rotate(180, True)
                    getting_further = False
                if not is_way_blocked:
                    drone_vector, exit_vector = self.get_navigation_vectors(drone_previous_position, point)
                    angle, clockwise = self.get_angle(drone_vector, exit_vector)
                    rotate_clockwise = clockwise
                    print("vector angle:" + str(angle))
                    self.how_to_rotate(angle, clockwise)
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
        global reached_checkpoint
        if one_time:
            self.lower_drone()
        while not one_time and not reached_checkpoint:
            self.lower_drone()

    def navigate_drone(self, checkpoint, turn_back=False):
        global is_exit
        global navigate_to_checkpoint
        global reached_checkpoint
        global drone_rotate
        global getting_further
        global getting_closer
        self.lost_localization_during_scan(True)

        angle, clockwise = (180, True) if turn_back else self.get_frame_angle(checkpoint)
        self.how_to_rotate(angle, clockwise)
        sleep(1)
        point = checkpoint

        Thread(target=self.monitor_drone, args=(point,)).start()
        navigate_to_checkpoint = True
        Thread(target=self.navigate_to_checkpoint, args=(point,)).start()
        while True:
            try:
                if reached_checkpoint:
                    self.drone.send_rc_control(0, 0, 0, 0)
                    navigate_to_checkpoint = False
                    break
                if not is_way_blocked:
                    if not drone_rotate:
                        self.drone.send_rc_control(0, self.drone_speed if not getting_closer else int(
                            self.drone_speed / 2), 0, 0)
                        sleep(2)
                else:
                    self.lost_localization_during_scan(True)
                    if not turn_back:
                        angle, clockwise = self.get_frame_angle(checkpoint)
                        self.how_to_rotate(angle, clockwise)
            except Exception as e:
                print(e.args[0])
                continue

    def monitor_drone(self, checkpoint):
        global reached_checkpoint
        global getting_further
        reached_checkpoint = False
        previous_distance = 10000
        while True:
            try:
                drone_pos = self.get_current_drone_position()
                dist = calculate_distance(drone_pos, checkpoint)
                print("distance to checkpoint:" + str(dist))
                if dist <= CLOSE_THRESHOLD:
                    self.drone.send_rc_control(0, 0, 0, 0)
                    reached_checkpoint = True
                    print("found checkpoint")
                    break
                getting_further = dist > previous_distance
                getting_closer = dist <= CLOSE_THRESHOLD + 0.1

                previous_distance = dist
                sleep(0.3)
            except Exception as e:
                print(e.args[0])
                continue

    def save_points_to_room(self, file_name, current_room):
        global number_of_room
        global rooms
        with open(file_name, 'r') as file:
            for line in file.readlines():
                values = line.split(',')
                point = Point(float(values[0]), float(values[2]), float(values[1]), float(values[3]),
                              float(values[4]), float(values[5]), float(values[6]), float(values[7]))
                if number_of_room != 0:
                    for room in rooms.keys():
                        if room != number_of_room:
                            if not is_point_in_room(point, rooms[room].rectangle):
                                current_room.points.append(point)
                                if point.frame_id not in current_room.frames.keys():
                                    current_room.frames[point.frame_id] = Frame(float(values[8]), float(values[9]),
                                                                                float(values[10]), point.qx, point.qy,
                                                                                point.qz, point.qw, point.frame_id)
                                current_room.frames[point.frame_id].points.append(point)
                                break
                else:
                    current_room.points.append(point)
                    if point.frame_id not in current_room.frames.keys():
                        current_room.frames[point.frame_id] = Frame(float(values[8]), float(values[9]),
                                                                    float(values[10]), point.qx, point.qy,
                                                                    point.qz, point.qw, point.frame_id)
                    current_room.frames[point.frame_id].points.append(point)

        return current_room

    def stay_in_the_air(self, room):
        global is_up_minus
        Thread(target=self.get_exit_points, args=(room, is_up_minus)).start()
        global exit_stay_in_the_air_loop
        while True:
            if exit_stay_in_the_air_loop:
                exit_stay_in_the_air_loop = False
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

    def get_exit_points(self, current_room, is_up_minus):
        global is_exit
        global exit_stay_in_the_air_loop
        current_room = self.get_newest_room(current_room)
        checkpoints_indexes, room_rectangle = get_exit_point(current_room.points, is_up_minus, is_exit)
        current_room.rectangle = room_rectangle
        if checkpoints_indexes is None:
            print("No checkpoints received after scan")
            exit_stay_in_the_air_loop = True
            return
        for point, best_segment in checkpoints_indexes:
            current_room.exit_points.append((point, best_segment))
        print("Navigate to checkpoint after scan")
        exit_stay_in_the_air_loop = True

    def map_and_exit(self):
        global is_exit
        global number_of_room
        global rooms
        global conn
        self.drone.takeoff()
        while True:
            rooms[number_of_room] = Room()
            print("number of room:" + str(number_of_room))
            self.begin_scan()
            battery_level = self.drone.get_battery()
            print("battery after scan:" + str(battery_level))
            self.save_map()

            self.stay_in_the_air(rooms[number_of_room])
            if len(rooms[number_of_room].exit_points) == 0:
                number_of_room -= 1 if number_of_room != 0 else 0
            self.fly_to_checkpoints(rooms[number_of_room])
            battery_level = self.drone.get_battery()
            print("battery after navigation:" + str(battery_level))
            number_of_room += 1
            is_exit = not is_exit
            if battery_level < 20:
                self.disconnect_drone()
                user_command = input("press enter twice to resume scan, input s to stop")
                if user_command == 's':
                    break
                self.connect_drone()
        return

    def stream_image_to_pipe(self):
        global current_frame
        out = cv2.VideoWriter('outpy.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 20, (960, 720))
        frame_read = self.drone.get_frame_read()
        try:
            os.mkfifo(FIFO_Images)
        except Exception as e:
            print(e.args[0])
        while True:
            try:
                current_frame = frame_read.frame
                out.write(current_frame)
                with open(FIFO_Images, 'wb') as f:
                    f.write(cv2.resize(current_frame, (960, 720)).tobytes())
            except Exception as e:
                print(e.args[0])
                continue

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
        maps = glob.glob("/tmp/pointData*.csv")
        max_index = 0
        for map in maps:
            index = int(map.split("pointData")[1].split(".")[0])
            max_index = index if index > max_index else max_index
        return self.save_points_to_room("/tmp/pointData" + str(max_index) + ".csv", current_room)

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
        if not self.drone.connect():
            raise RuntimeError("drone not connected")
        self.drone.streamon()

    def disconnect_drone(self):
        self.drone.land()
        sleep(5)
        self.drone.streamoff()
        self.drone = None

    def send_signal_to_slam(self, message):
        global conn
        conn.send(str(message).encode())


def main():
    drone = Drone()
    drone.map_and_exit()


if __name__ == '__main__':
    main()
