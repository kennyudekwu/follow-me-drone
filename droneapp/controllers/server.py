import logging

from flask import jsonify
from flask import render_template
from flask import request
from flask import Response

from droneapp.models.drone_manager import DroneManager

import config
from dronekit import connect

logger = logging.getLogger(__name__)
app = config.app
vehicle = None


def get_drone(arg):
    return DroneManager(arg)


def get_connection():
    global vehicle
    if vehicle is None:
        vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)
    return vehicle


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
    get_connection()
    drone = get_drone(get_connection())

    if cmd == 'takeOff':
        drone.arm_and_takeOff()
    if cmd == 'land_now':
        drone.land_and_disarm()
    if cmd == 'up':
        drone.set_velocity_body(0, 0, -0.2, 0, 0, 0, 0)
    if cmd == 'clockwise':
        drone.condition_yaw(15, relative=True)
    if cmd == 'counterClockwise':
        drone.condition_yaw(-15, relative=True)
    if cmd == 'down':
        drone.set_velocity_body(0, 0, 0.2, 0, 0, 0, 0)
    if cmd == 'forward':
        drone.set_velocity_body(0.15, 0, 0, 0, 0, 0, 0)
    if cmd == 'left':
        drone.set_velocity_body(0, -0.15, 0, 0, 0, 0, 0)
    if cmd == 'right':
        drone.set_velocity_body(0, 0.15, 0, 0, 0, 0, 0)
    if cmd == 'backward':
        drone.set_velocity_body(-0.15, 0, 0, 0, 0, 0, 0)
    if cmd == 'faceDetectandTrack':
        drone.enable_face_detect()
    if cmd == 'stopfaceDetectandTrack':
        drone.disable_face_detect()
    if cmd == 'bodyDetectandTrack':
        drone.enable_body_detect()
    if cmd == 'stopbodyDetectandTrack':
        drone.disable_body_detect()
    if cmd == 'loiter':
        drone.change_mode('LOITER')
    if cmd == 'guided':
        drone.change_mode('GUIDED')
    if cmd == 'rtl':
        drone.change_mode('RTL')
    if cmd == 'snapshot':
        if drone.snapshot():
            return jsonify(status='success'), 200
        else:
            return jsonify(status='fail'), 400

    return jsonify(status='success'), 200


def video_generator():
    get_connection()
    drone = get_drone(get_connection())

    for jpeg in drone.video_jpeg_generator():
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' +
               jpeg +
               b'\r\n\r\n')


@app.route('/video/streaming')
def video_feed():
    return Response(video_generator(), mimetype='multipart/x-mixed-replace; boundary=frame')


def run():
    app.run(host=config.WEB_ADDRESS, port=config.WEB_PORT,
            debug=True, threaded=True)
