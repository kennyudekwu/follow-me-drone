import cv2 as cv
from droneapp.models.base import Singleton
import os
import time

DS_FACTOR = 0.6
SNAPSHOT_IMAGE_FOLDER = './droneapp/static/img/snapshots/'


class ErrorNoImageDir(Exception):
    """Error no image directory"""


class Camera(object):
    __metaclass__ = Singleton

    def __init__(self):
        self.cap = cv.VideoCapture(0)

        if not os.path.exists(SNAPSHOT_IMAGE_FOLDER):
            raise ErrorNoImageDir('{} does not exists'.format(SNAPSHOT_IMAGE_FOLDER))

        self.is_snapshot = False

    def __del__(self):
        self.cap.release()

    def video_jpeg_generator(self):
        while True:
            img, frame = self.cap.read()
            frame = cv.resize(frame, None, fx=DS_FACTOR, fy=DS_FACTOR,
                              interpolation=cv.INTER_AREA)

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
