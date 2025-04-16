#!/usr/local/bin/env python3.6
import asyncio
import logging
import os
import socket
import time

import evolver_server
import robotics_server
import socketio
import yaml
from multi_server import MultiServer

EVOLVER_CONF_FILENAME = "conf.yml"
ROBOTICS_CONF_FILENAME = "robotics_server_conf.yml"

logger = logging.getLogger("ht_evolver")
logging.basicConfig(
    format="%(asctime)s - %(name)s - [%(levelname)s] - %(message)s\n",
    datefmt="%Y-%m-%d %H:%M:%S",
    filename="/home/pi/ht_evolver.log",
    level=logging.INFO,
)
logging.getLogger("engineio.client").setLevel(logging.ERROR)
logging.getLogger("socketio.client").setLevel(logging.ERROR)
logging.getLogger("aiohttp").setLevel(logging.ERROR)
logging.getLogger("urllib3").setLevel(logging.ERROR)
logging.getLogger("asyncio").setLevel(logging.ERROR)
evolver_conf = {}
robotics_conf = {}

if __name__ == "__main__":
    with open(
        os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__), EVOLVER_CONF_FILENAME)),
        "r",
    ) as ymlfile:
        evolver_conf = yaml.safe_load(ymlfile)
    with open(
        os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__), ROBOTICS_CONF_FILENAME)),
        "r",
    ) as ymlfile:
        robotics_conf = yaml.safe_load(ymlfile)

    # need to get our IP
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    evolver_ip = s.getsockname()[0]
    s.close()

    # Set up evolver_server before adding to MultiServer
    evolver_server = evolver_server.EvolverServer(evolver_ip)
    evolver_server.setup_event_handlers()

    # Set up the robotics_server before adding to MultiServer
    robotics_server = robotics_server.RoboticsServer()
    robotics_server.register_callback()
    robotics_server.setup_event_handlers()

    server_loop = asyncio.new_event_loop()
    ms = MultiServer(loop=server_loop)
    app1 = ms.add_app(port=evolver_conf["evolver_port"])
    app2 = ms.add_app(port=robotics_conf["robotics_port"])
    evolver_server.attach(app1)
    robotics_server.attach(app2)
    ms.run_all()

    # Set up the robotics_server as an eVOLVER client
    socketIO_eVOLVER = socketio.Client(logger=True, engineio_logger=True)
    robotics_server.setup_client(socketIO_eVOLVER)
    connected = False
    while not connected:
        try:
            socketIO_eVOLVER.connect(
                "http://{0}:{1}".format(evolver_ip, evolver_conf["evolver_port"]),
                namespaces=["/default_evolver"],
            )
            connected = True
            logger.info("Connected to eVOLVER server")
        except Exception:
            logger.info("Failed to connect to eVOLVER server. Retrying in 0.5 seconds.")
            time.sleep(0.5)

    # Set up data broadcasting
    bloop = asyncio.new_event_loop()
    last_time = None
    while True:
        current_time = time.time()
        evolver_status = evolver_server.get_evolver_status()

        if (
            (last_time is None or current_time - last_time >= evolver_conf["broadcast_timing"])
            and not evolver_status["running_immediate"]
            and not evolver_status["running_broadcast"]
        ):
            start_time = time.time()
            bloop.run_until_complete(robotics_server.broadcast())
            result = bloop.run_until_complete(evolver_server.broadcast(0))
            if not result:
                continue
            time.sleep(5)

            result = bloop.run_until_complete(evolver_server.broadcast(1))
            if not result:
                continue

            result = bloop.run_until_complete(evolver_server.broadcast(2))
            if not result:
                continue
            end_time = time.time()
            elapsed_time = end_time - start_time
            logger.info("total broadcast processing time: %s" % elapsed_time)

            last_time = time.time()
