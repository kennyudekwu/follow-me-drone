import logging
import sys

import config
import droneapp.controllers.server

logging.basicConfig(level=logging.INFO, stream=sys.stdout)

if __name__ == '__main__':
    droneapp.controllers.server.run()