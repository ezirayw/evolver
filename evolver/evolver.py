#!/usr/local/bin/env python3.6
import yaml
import time
import asyncio
from multi_server import MultiServer
from threading import Thread
import socket
import evolver_server
import os

conf = {}
CONF_FILENAME = 'conf.yml'

def start_background_loop(loop):
    asyncio.set_event_loop(loop)
    loop.run_forever()

if __name__ == '__main__':
    # need to get our IP
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    evolver_ip = s.getsockname()[0]
    s.close()
    with open(os.path.realpath(os.path.join(os.getcwd(),os.path.dirname(__file__), CONF_FILENAME)), 'r') as ymlfile:
        conf = yaml.load(ymlfile)

    conf['evolver_ip'] = evolver_ip

    # Set up the server
    server_loop = asyncio.new_event_loop()
    ms = MultiServer(loop=server_loop)
    app1 = ms.add_app(port = conf['port'])
    evolver_server.attach(app1, conf)
    ms.run_all()

    # Set up data broadcasting
    bloop = asyncio.new_event_loop()
    last_time = None
    running = False
    arduino_coordination = ''
    while True:
        current_time = time.time()
        commands_in_queue = evolver_server.get_num_commands() > 0

        if (last_time is None or current_time - last_time > conf['broadcast_timing'] or commands_in_queue) and not running:
            try:
                running = True
                arduino_coordination = 'pre_reading'
                print("Running Pre_Reading Broadcast!")
                bloop.run_until_complete(evolver_server.broadcast(commands_in_queue, arduino_coordination))
                print("Pre_Reading Parameters Set!")
                print(time.time())
                time.sleep(7)
                print(time.time())

                arduino_coordination = ''
                print("Running True Broadcast!")
                bloop.run_until_complete(evolver_server.broadcast(commands_in_queue, arduino_coordination))
                print("Data Broadcasted!")

                arduino_coordination = 'post_reading'
                print("Resetting Parameters Until Next Reading!")
                bloop.run_until_complete(evolver_server.broadcast(commands_in_queue, arduino_coordination))
                print("Post_Reading Broadcast Set!")
                running = False
            except:
                pass
            if last_time is None or current_time - last_time > conf['broadcast_timing']:
                last_time = time.time()
