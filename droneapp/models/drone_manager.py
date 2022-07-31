from dronekit import VehicleMode
import logging
import numpy as np
import os
# import threading
import sys
import time


from pymavlink import mavutil
import cv2 as cv

# from contextlib2 import ExitStack

from droneapp.models.base import Singleton

logging.basicConfig(level=logging.INFO, stream=sys.stdout)
logger = logging.getLogger(__name__)

P_Error_yaw = 0
P_Error_Vz = 0
P_Error_Vy = 0
P_Error_Vx = 0
DEFAULT_ALTITUDE = 1.4

PID = [0.5, 0.5]
FRAME_X = int(960 / 3)
FRAME_Y = int(720 / 3)
FRAME_AREA = FRAME_X * FRAME_Y

FRAME_CENTER_X = FRAME_X / 2
FRAME_CENTER_Y = FRAME_Y / 2

FACE_DETECT_XML_FILE = './droneapp/models/haarcascade_frontalface_default.xml'
BODY_DETECT_XML_FILE = './droneapp/models/haarcascade_fullbody.xml'

SNAPSHOT_IMAGE_FOLDER = './droneapp/static/img/snapshots/'


class ErrorNoImageDir(Exception):
    """Error no image directory"""


class ErrorNoFaceDetectXMLFile(Exception):
    """Error no face detect xml file"""


class DroneManager:
    __metaclass__ = Singleton

    def __init__(self, vehicle):

        self.vehicle = vehicle
        self.cap = cv.VideoCapture(0)
        self.yaw_val = 0
        self.Vx = 0
        self.Vy = 0
        self.Vz = 0

        if not os.path.exists(FACE_DETECT_XML_FILE):
            raise ErrorNoFaceDetectXMLFile(
                'No {}'.format(FACE_DETECT_XML_FILE))
        self.face_cascade = cv.CascadeClassifier(FACE_DETECT_XML_FILE)
        self.body_cascade = cv.CascadeClassifier(BODY_DETECT_XML_FILE)
        self._is_enable_face_detect = False
        self._is_enable_body_detect = False

        if not os.path.exists(SNAPSHOT_IMAGE_FOLDER):
            raise ErrorNoImageDir(
                '{} does not exists'.format(SNAPSHOT_IMAGE_FOLDER))

        self.is_snapshot = False
        self.fly = False
        # self._command_semaphore = threading.Semaphore(1)
        # self._command_thread = None
        # self._yaw_semaphore = threading.Semaphore(1)
        # self._yaw_thread = None

    def __del__(self):
        self.stop()
        self.cap.release()

    def stop(self):
        self.vehicle.close()

    def arm_and_takeOff(self):
        if not self.vehicle.location.global_relative_frame.alt >= 1:
            self.fly = True
            # switch vehicle to GUIDED mode and wait for change
            self.vehicle.mode = VehicleMode("GUIDED")
            while self.vehicle.mode != "GUIDED":
                print("Waiting for vehicle to enter GUIDED mode")
                time.sleep(1)
            print("Vehicle now in {}".format(self.vehicle.mode))
            # Arm vehicle once GUIDED mode is confirmed
            self.vehicle.armed = True

            while not self.vehicle.armed:

                if self.fly:
                    print("Waiting for vehicle to arm..")
                    print('self.fly={}'.format(self.fly))
                    time.sleep(1)
                else:
                    return None
            self.vehicle.simple_takeoff(DEFAULT_ALTITUDE)

            while True:
                if self.fly:
                    print("Current Altitude: %d" %
                          self.vehicle.location.global_relative_frame.alt)
                    if self.vehicle.location.global_relative_frame.alt >= (DEFAULT_ALTITUDE * .95):
                        break
                    time.sleep(1)
                else:
                    return None

            print("Default altitude reached")

            return None

        elif self.fly:
            print("Vehicle is already air bound")
            return None

    def land_and_disarm(self):
        if self.fly:
            self.fly = False
        print('Running the land function...')
        print('self.fly = {}'.format(self.fly))

        if self.vehicle.location.global_relative_frame.alt > 0.5:
            self.vehicle.mode = VehicleMode("LAND")
            print('Vehicle mode has been switched to {}'.format(self.vehicle.mode))
            while not self.vehicle.mode == VehicleMode("LAND"):
                print("Vehicle is preparing to land...")
            while True:
                print("Current Altitude: %d" %
                      self.vehicle.location.global_relative_frame.alt)
                if self.vehicle.location.global_relative_frame.alt <= 0.5:
                    break

        elif self.vehicle.armed:
            self.vehicle.armed = False
            while self.vehicle.armed:
                print('Disarming motors...')
        print("Vehicle currently grounded")
        return None

    # def set_velocity_body(self, Vx, Vy, Vz, dx, dy, dz, ff, blocking=True):
    #     self._command_thread = threading.Thread(target=self._set_velocity_body, args=(Vx, Vy, Vz, dx, dy, dz,
    #                                                                                   ff, blocking))
    #     self._command_thread.start()

    # def condition_yaw(self, heading, relative=False, blocking=True):
    #     self._yaw_thread = threading.Thread(target=self._condition_yaw, args=(heading, relative, blocking))
    #     self._yaw_thread.start()

    def set_velocity_body(self, Vx, Vy, Vz, dx, dy, dz, ff):
        logger.info({'action': 'send velocity command', 'Vx': Vx, 'Vy': Vy, 'Vz': Vz,
                    'diffx': dx, 'diffy': dy, 'percent': dz, 'frameA': FRAME_AREA, 'Face_frame': ff})

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
            0b0000111111000111,  # Bitmask-considering only the velocities
            0, 0, 0,  # Position
            Vx, Vy, Vz,  # Velocity
            0, 0, 0,  # Acceleration
            0, 0)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def condition_yaw(self, heading, relative=False):
        logger.info({'action': 'send yaw command', 'yaw_val': heading})
        if relative:
            is_relative = 1  # yaw relative to direction of travel
        else:
            is_relative = 0  # yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            0,  # param 1, yaw in degrees
            heading,  # param 2, yaw speed deg/s
            1,  # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)  # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def enable_face_detect(self):
        self._is_enable_face_detect = True

    def disable_face_detect(self):
        self._is_enable_face_detect = False

    def enable_body_detect(self):
        self._is_enable_body_detect = True

    def disable_body_detect(self):
        self._is_enable_body_detect = False

    def error_calc_yaw(self, info, w, pid, pError):
        error = info[0][0] - w // 2
        speed = (pid[0] * error) + (pid[1] * (error - pError))
        speed = int(np.clip(speed, -100, 100))

        if info[0][0] != 0:
            self.yaw_val = speed
        else:
            self.yaw_val = 0
            error = 0

        return error

    def error_calc_Vz(self, info, h, pid, pError):
        error = info[0][1] - h // 2
        speed = ((pid[0] * error) + (pid[1] * (error - pError)))
        speed = int(np.clip(speed, -5, 5))

        if info[0][1] != 0:
            self.Vz = speed
        else:
            self.Vz = 0
            error = 0

        return error

    def error_calc_Vx(self, info, pid, pError):
        error = float("{:.3f}".format(float((0.30 * FRAME_AREA) - info[1])))
        speed = (pid[0] * error) + (pid[1] * (error - pError))
        speed = int(np.clip(speed, -5, 5))

        if info[1] != 0:
            self.Vx = speed
        else:
            self.Vx = 0
            error = 0

        return error

    def error_calc_Vy(self, info, w, pid, pError):
        error = info[0][0] - w // 2
        speed = (pid[0] * error) + (pid[1] * (error - pError))
        speed = int(np.clip(speed, -5, 5))

        if info[0][0] != 0:
            self.Vy = speed
        else:
            self.Vy = 0
            error = 0

        return error

    def video_jpeg_generator(self):
        while True:
            img, frame = self.cap.read()
            frame = cv.resize(frame, (FRAME_X, FRAME_Y))

            if self._is_enable_face_detect:
                cascade = self.face_cascade
            elif self._is_enable_body_detect:
                cascade = self.body_cascade

            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

            # detects all the faces/bodies in a frame and stores to "faces"
            objects = cascade.detectMultiScale(gray, 1.3, 5)

            my_object_list_center = []
            my_object_list_area = []

            # loop through all the objects in the frame detected in each
            # frame and find the areas as well as the midpoints of the objects in the frame so far
            # we are only interested in the closest object, hence the greatest area of the objects detected
            for (x, y, w, h) in objects:
                cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

                object_center_x = x + (w / 2)

                object_center_y = y + (h / 2)

                # to scale up the face_area to the FRAME_AREA
                object_area = float("{:.3f}".format(float(w * h * 14.8145)))
                my_object_list_area.append(object_area)
                my_object_list_center.append(
                    (object_center_x, object_center_y))

            if len(my_object_list_area) != 0:
                i = my_object_list_area.index(max(my_object_list_area))
                info = [my_object_list_center[i], my_object_list_area[i]]

            # if no face was detected
            else:
                info = [[0, 0], 0]

            diff_x = FRAME_CENTER_X - info[0][0]
            diff_y = FRAME_CENTER_Y - info[0][1]

            raw_percent_object = float(info[1] / FRAME_AREA)
            percent_object = float("{:.3f}".format(raw_percent_object))

            global P_Error_yaw
            P_Error_yaw = self.error_calc_yaw(
                info, FRAME_X, PID, P_Error_yaw)

            global P_Error_Vx
            P_Error_Vx = self.error_calc_Vx(info, PID, P_Error_Vx)

            global P_Error_Vy
            P_Error_Vy = self.error_calc_Vy(info, FRAME_X, PID, P_Error_Vy)

            global P_Error_Vz
            P_Error_Vz = self.error_calc_Vz(info, FRAME_Y, PID, P_Error_Vz)

            self.set_velocity_body(
                self.Vx, self.Vy, self.Vz, diff_x, diff_y, percent_object, info[1])
            # time for the drone to respond to the initial commands
            time.sleep(0.05)

            self.condition_yaw(self.yaw_val, relative=True)
            # time for the drone to respond to the second commands
            time.sleep(0.05)

            _, jpeg = cv.imencode('.jpg', frame)
            jpeg_binary = jpeg.tobytes()

            if self.is_snapshot:
                backup_file = time.strftime("%Y%m%d-%H%M%S") + '.jpg'
                snapshot_file = 'snapshot.jpg'
                for filename in (backup_file, snapshot_file):
                    file_path = os.path.join(
                        SNAPSHOT_IMAGE_FOLDER, filename)
                    with open(file_path, 'wb') as f:
                        f.write(jpeg_binary)
            self.is_snapshot = False

            yield jpeg_binary

    def snapshot(self):
        self.is_snapshot = True
        retry = 0
        while retry < 3:
            if not self.is_snapshot:
                return True
            time.sleep(0.1)
            retry += 1
        return False

    def change_mode(self, mode):
        if not self.vehicle.mode.name == mode:
            self.vehicle.mode = VehicleMode(mode)
            while self.vehicle.mode != VehicleMode(mode):
                print('Waiting for vehicle to enter {} mode'.format(mode))
            print('Vehicle mode has been switched to {}'.format(self.vehicle.mode))
        else:
            print('Vehicle mode already {}'.format(self.vehicle.mode))
        return None

    # def change_mode_auto(self):
    #     if not self.vehicle.mode.name == "GUIDED":
    #         self.vehicle.mode = VehicleMode("GUIDED")
    #         while self.vehicle.mode != VehicleMode("GUIDED"):
    #             print("Waiting for vehicle to enter GUIDED mode")
    #         print('Vehicle mode has been switched to {}'.format(self.vehicle.mode))
    #     else:
    #         print('Vehicle mode already {}'.format(self.vehicle.mode))
    #     return None

    # def change_mode_RTL(self):
    #     if not self.vehicle.mode.name == "RTL":
    #         self.vehicle.mode = VehicleMode("RTL")
    #         while self.vehicle.mode != VehicleMode("RTL"):
    #             print("vehicle is initializing RTL mode...")
    #         print('Vehicle mode has been switched to {}'.format(self.vehicle.mode))
    #     else:
    #         print('Vehicle mode already {}'.format(self.vehicle.mode))
    #     return None
