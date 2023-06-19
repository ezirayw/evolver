#!/usr/local/bin/env python3.6
import yaml
import time
import asyncio
from multi_server import MultiServer
import socket
import evolver_server
import robotics_server
import socketio
import os
import logging

logger = logging.getLogger('ht_evolver')
logging.basicConfig(format='%(asctime)s - %(name)s - [%(levelname)s] ''- %(message)s\n', datefmt='%Y-%m-%d %H:%M:%S', filename='/home/pi/ht_evolver.log', level=logging.INFO)
logging.getLogger('engineio.client').setLevel(logging.ERROR)
logging.getLogger('socketio.client').setLevel(logging.ERROR)
logging.getLogger('aiohttp').setLevel(logging.ERROR)
logging.getLogger('urllib3').setLevel(logging.ERROR)
logging.getLogger('asyncio').setLevel(logging.ERROR)

evolver_conf = {}
robotics_conf = {}
EVOLVER_CONF_FILENAME = 'conf.yml'
ROBOTICS_CONF_FILENAME = 'robotics_server_conf.yml'

def start_background_loop(loop):
    asyncio.set_event_loop(loop)
    loop.run_forever()

if __name__ == '__main__':
    # need to get our IP
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    evolver_ip = s.getsockname()[0]
    s.close()

    with open(os.path.realpath(os.path.join(os.getcwd(),os.path.dirname(__file__), EVOLVER_CONF_FILENAME)), 'r') as ymlfile:
        evolver_conf = yaml.safe_load(ymlfile)
    
    with open(os.path.realpath(os.path.join(os.getcwd(),os.path.dirname(__file__), ROBOTICS_CONF_FILENAME)), 'r') as ymlfile:
        robotics_conf = yaml.safe_load(ymlfile)

    evolver_conf['evolver_ip'] = evolver_ip
    robotics_conf['evolver_ip'] = evolver_ip
    robotics_conf['evolver_port'] = evolver_conf['evolver_port']

    # Set up the server
    server_loop = asyncio.new_event_loop()
    ms = MultiServer(loop=server_loop)
    app1 = ms.add_app(port = evolver_conf['evolver_port'])
    app2 = ms.add_app(port = robotics_conf['robotics_port'])
    evolver_server.attach(app1, evolver_conf)
    robotics_server.attach(app2)
    ms.run_all()

    # Set up client
    socketIO_eVOLVER = socketio.Client(logger=True, engineio_logger=True)
    robotics_server.setup_client(socketIO_eVOLVER)
    socketIO_eVOLVER.connect("http://{0}:{1}".format(evolver_conf['evolver_ip'], evolver_conf['evolver_port']), namespaces=['/dpu-evolver'])


    # Set up data broadcasting
    bloop = asyncio.new_event_loop()
    last_time = None
    running = False
    while True:
        current_time = time.time()
        commands_in_queue = evolver_server.get_num_commands() > 0
        commands_blank = False

        if (last_time is None or current_time - last_time > evolver_conf['broadcast_timing'] or commands_in_queue) and not running:
            if last_time is None or current_time - last_time > evolver_conf['broadcast_timing']:
                last_time = time.time()
            try:
                running = True
                tag = 'pre_reading'
                logger.debug("Running Pre-Data Broadcast!")
                bloop.run_until_complete(robotics_server.broadcast())
                bloop.run_until_complete(evolver_server.broadcast(commands_in_queue, tag))
                logger.debug("Pre-Data Broadcast Sent!")
                time.sleep(5)

                logger.debug("Running True Broadcast!")
                bloop.run_until_complete(evolver_server.broadcast(commands_blank, ''))

                tag = 'post_reading'
                logger.debug("Running Post-Data Broadcast!")
                bloop.run_until_complete(evolver_server.broadcast(commands_blank, tag))
                logger.debug("Post-Data Broadcast Sent!")
                running = False
            except:
                pass
