import logging

from flask import jsonify
from flask import render_template
from flask import request
from flask import Response

from droneapp.models.drone_manager import DroneManager
from droneapp.models.camera import Camera

import config

logger = logging.getLogger(__name__)
app = config.app


def get_drone():
    return DroneManager()


def get_camerafeed():
    return Camera()


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/controller/')
def controller():
    return render_template('controller.html')


@app.route('/api/command/', methods=['POST'])
def command():
    cmd = request.form.get('command')
    logger.info({'action': 'command', 'cmd': cmd})
    drone = get_drone()

    if cmd == 'takeOff':
        drone.arm_and_takeOff()
    if cmd == 'land':
        drone.land_and_disarm()
    if cmd == 'faceDetectandTrack':
        drone.enable_face_detect()
    if cmd == 'stopfaceDetectandTrack':
        drone.disable_face_detect()

    return jsonify(status='success'), 200


@app.route('/api/picture/', methods=['POST'])
def image_capture():
    cmd = request.form.get('command')
    logger.info({'action': 'command', 'cmd': cmd})
    video = get_camerafeed()

    if cmd == 'snapshot':
        if video.snapshot():
            return jsonify(status='success'), 200
        else:
            return jsonify(status='fail'), 400

    return jsonify(status='success'), 200


def video_generator():
    video = get_camerafeed()
    for jpeg in video.video_jpeg_generator():
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' +
               jpeg +
               b'\r\n\r\n')


@app.route('/video/streaming')
def video_feed():
    return Response(video_generator(), mimetype='multipart/x-mixed-replace; boundary=frame')


def run():
    app.run(host=config.WEB_ADDRESS, port=config.WEB_PORT, debug=True, threaded=True)
