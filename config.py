import os
import socket
from flask import Flask

# WEB_ADDRESS = socket.gethostbyname(socket.gethostname())
WEB_ADDRESS = '0.0.0.0'
WEB_PORT = 5000
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
TEMPLATES = os.path.join(PROJECT_ROOT, 'droneapp/templates')
STATIC_FOLDER = os.path.join(PROJECT_ROOT, 'droneapp/static')
DEBUG = False
LOG_FILE = 'raspi.log'

app = Flask(__name__, template_folder=TEMPLATES, static_folder=STATIC_FOLDER)

if DEBUG:
    app.debug = DEBUG
