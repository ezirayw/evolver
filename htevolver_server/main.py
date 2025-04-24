import asyncio
import logging
import os
import socket
import time

import socketio
import yaml
from aiohttp import web
from aiohttp.web_app import Application
from htevolver.htevolver_namespace import EvolverNamespace
from htevolver_client.interfaces.htevolver_interface import HTEvolverNamespace
from robotics.robotics_namespace import RoboticsNamespace

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


async def shutdown(app):
    """Cancel the broadcast task when shutting down"""
    if app.get("broadcast_task"):
        app["broadcast_task"].cancel()
        try:
            await app["broadcast_task"]
        except asyncio.CancelledError:
            pass


async def broadcast_loop(app: Application):
    """Background task for periodic broadcasting"""

    last_time = None
    while True:
        current_time = time.time()
        evolver_status = app["evolver_namespace"].get_evolver_status()

        if (
            (last_time is None or current_time - last_time >= app["evolver_conf"]["broadcast_timing"])
            and not evolver_status["running_immediate"]
            and not evolver_status["running_broadcast"]
        ):
            start_time = time.time()
            await app["robotics_namespace"].broadcast()
            result = await app["evolver_namespace"].broadcast(0)
            if not result:
                await asyncio.sleep(0.1)
                continue

            await asyncio.sleep(5)

            result = await app["evolver_namespace"].broadcast(1)
            if not result:
                await asyncio.sleep(0.1)
                continue

            result = await app["evolver_namespace"].broadcast(2)
            if not result:
                await asyncio.sleep(0.1)
                continue

            end_time = time.time()
            elapsed_time = end_time - start_time
            logger.info(f"total broadcast processing time: {elapsed_time}")

            last_time = time.time()

        # Non-blocking sleep to let the event loop handle other tasks
        await asyncio.sleep(0.1)


async def start_background_tasks(app):
    """Start background tasks after app startup"""
    app["broadcast_task"] = asyncio.create_task(broadcast_loop(app))


async def init_app():
    """Initialize the web application with all required components"""
    # Load configs
    evolver_conf_path = os.path.realpath(os.path.join(os.getcwd(), "htevolver", EVOLVER_CONF_FILENAME))
    robotics_conf_path = os.path.realpath(os.path.join(os.getcwd(), "robotics", ROBOTICS_CONF_FILENAME))
    evolver_conf = {}
    robotics_conf = {}
    with open(evolver_conf_path, "r") as ymlfile:
        evolver_conf = yaml.safe_load(ymlfile)

    with open(robotics_conf_path, "r") as ymlfile:
        robotics_conf = yaml.safe_load(ymlfile)

    app = web.Application()
    app["port"] = evolver_conf["port"]
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip = s.getsockname()[0]
    app["ip"] = ip
    s.close()

    sio = socketio.AsyncServer()
    sio.attach(app)

    htevolver_client = HTEvolverNamespace(namespace="/evolver", connect=False)
    app["evolver_namespace"] = EvolverNamespace(evolver_conf, ip)
    app["robotics_namespace"] = RoboticsNamespace(robotics_conf, htevolver_client)
    sio.register_namespace(app["evolver_namespace"])
    sio.register_namespace(app["robotics_namespace"])

    # Set up startup and shutdown handlers
    app.on_startup.append(start_background_tasks)
    app.on_shutdown.append(shutdown)

    return app


async def main():
    """Main entry point for the application"""
    app = await init_app()

    # Setup and start the web server
    port = app["evolver_conf"].get("evolver_port", 8081)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", port)

    logger.info(f"Starting HT-eVOLVER server on port {port}")
    await site.start()

    # Keep the server running indefinitely
    try:
        while True:
            await asyncio.sleep(3600)  # Sleep for an hour
    except (KeyboardInterrupt, asyncio.CancelledError):
        logger.info("Shutting down HT-eVOLVER server")


if __name__ == "__main__":
    asyncio.run(main())
