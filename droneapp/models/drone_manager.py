from dronekit import connect, VehicleMode, LocationGlobalRelative
import logging
import os
import threading
import sys
import time
import exceptions
from pymavlink import mavutil

import cv2 as cv
import numpy as np
import contextlib


from droneapp.models.base import Singleton

logging.basicConfig(level=logging.INFO, stream=sys.stdout)
logger = logging.getLogger(__name__)

DEFAULT_ALTITUDE = 1.86

FRAME_X = int(960 / 3)
FRAME_Y = int(720 / 3)
FRAME_AREA = FRAME_X * FRAME_Y

FRAME_SIZE = FRAME_AREA * 3
FRAME_CENTER_X = FRAME_X / 2
FRAME_CENTER_Y = FRAME_Y / 2

FACE_DETECT_XML_FILE = './droneapp/models/haarcascade_frontalface_default.xml'

SNAPSHOT_IMAGE_FOLDER = './droneapp/static/img/snapshots/'


class ErrorNoFaceDetectXMLFile(Exception):
    """Error no face detect xml file"""


class ErrorNoImageDir(Exception):
    """Error no image directory"""


class DroneManager:
    __metaclass__ = Singleton

    def __init__(self, connection_string='/dev/ttyAMA0', wait_ready=True, baud=57600):
        self.connection_string = connection_string
        self.wait_ready = wait_ready
        self.baud = baud
        # Connect to the Vehicle (in this case a UDP endpoint)
        self.vehicle = connect(self.connection_string, wait_ready=self.wait_ready, baud=self.baud)

        self._play_video_thread = threading.Thread(target=self.video_jpeg_generator)
        self._play_video_thread.start()

        if not os.path.exists(FACE_DETECT_XML_FILE):
            raise ErrorNoFaceDetectXMLFile('No {}'.format(FACE_DETECT_XML_FILE))
        self.face_cascade = cv.CascadeClassifier(FACE_DETECT_XML_FILE)
        self._is_enable_face_detect = False

        if not os.path.exists(SNAPSHOT_IMAGE_FOLDER):
            raise ErrorNoImageDir('{} does not exists'.format(SNAPSHOT_IMAGE_FOLDER))
        self.is_snapshot = False

        self._command_semaphore = threading.Semaphore(1)
        self._command_thread = None

    def __del__(self):
        self.stop()

    def stop(self):
        self.vehicle.close()

    def arm_and_takeOff(self):
        while not self.vehicle.is_armable:
            print("Waiting for vehicle to become armable..")
            time.sleep(1)

        # switch vehicle to GUIDED mode and wait for change
        self.vehicle.mode = VehicleMode("GUIDED")
        while self.vehicle.mode != "GUIDED":
            print("Waiting for vehicle to enter GUIDED mode")
            time.sleep(1)

        # Arm vehicle once GUIDED mode is confirmed
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("Waiting for vehicle to arm..")
            time.sleep(1)

        self.vehicle.simple_takeoff(DEFAULT_ALTITUDE)

        while True:
            print("Current Altitude: %d" % self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= (DEFAULT_ALTITUDE * .95):
                break
            time.sleep(1)

        print("Default DEFAULT_ALTITUDE reached")
        return None

    def land_and_disarm(self):
        if self.vehicle.location.global_relative_frame.alt > 0:
            self.vehicle.mode = VehicleMode("LAND")
        elif self.vehicle.armed:
            self.vehicle.armed = False

    def set_velocity_body(self, Vx, Vy, Vz, blocking=True):
        self._command_thread = threading.Thread(target=self._set_velocity_body, args=(Vx, Vy, Vz, blocking))
        self._command_thread.start()

    def _set_velocity_body(self, Vx, Vy, Vz, blocking=True):
        is_acquire = self._command_semaphore.acquire(blocking=blocking)
        if is_acquire:
            with contextlib.ExitStack() as stack:
                stack.callback(self._command_semaphore.release)
                logger.info({'action': 'send velocity command', 'Vx': Vx, 'Vy': Vy, 'Vz': Vz})

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
        else:
            logger.warning({'action': 'send velocity command', 'Vx': Vx, 'Vy': Vy, 'Vz': Vz, 'status': 'not_acquire'})
        return None

    def enable_face_detect(self):
        self._is_enable_face_detect = True

    def disable_face_detect(self):
        self._is_enable_face_detect = False

    def video_jpeg_generator(self):
        cap = cv.VideoCapture(0)
        while cap.isOpened():
            img, frame = cap.read()
            if self._is_enable_face_detect:
                gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
                faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in faces:
                    cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

                    face_center_x = x + (w / 2)
                    face_center_y = x + (h / 2)
                    diff_x = FRAME_CENTER_X - face_center_x
                    diff_y = FRAME_CENTER_Y - face_center_y
                    face_area = w * h
                    percent_face = face_area / FRAME_AREA

                    drone_x, drone_y, drone_z = 0, 0, 0
                    if diff_x < -30:
                        drone_y = 0.3
                    if diff_x > 30:
                        drone_y = -0.3
                    if diff_y < -15:
                        drone_z = 0.3
                    if diff_y > 15:
                        drone_z = -0.3
                    if percent_face > 0.3:
                        drone_x = -0.3
                    if percent_face < 0.2:
                        drone_x = 0.3
                    self.set_velocity_body(drone_x, drone_y, drone_z, blocking=False)

                    break

            _, jpeg = cv.imencode('.jpg', frame)
            jpeg_binary = jpeg.tobytes()

            if self.is_snapshot:
                backup_file = time.strftime("%Y%m%d-%H%M%S") + '.jpg'
                snapshot_file = 'snapshot.jpeg'
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


if __name__ == '__main__ ':
    drone_manager = DroneManager()
    drone_manager.arm_and_takeOff()

    time.sleep(10)

    drone_manager.land_and_disarm()
    drone_manager.stop()
