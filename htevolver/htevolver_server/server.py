import asyncio
import contextlib
import logging
import os
import time

import socketio
import yaml
from aiohttp import web
from aiohttp.web_app import Application

from htevolver.htevolver_server.evolver_namespace_server import EvolverServerNamespace
from htevolver.htevolver_server.robotics_namespace_server import RoboticsServerNamespace

EVOLVER_CONF_FILENAME: str = "evolver_conf.yml"
ROBOTICS_CONF_FILENAME: str = "robotics_conf.yml"
LOGGING_DIR: str = "/home/pi/logs"

logger = logging.getLogger("htevolver")
logger.setLevel(logging.INFO)


def setup_logging(log_dir: str = LOGGING_DIR):
    os.makedirs(log_dir, exist_ok=True)

    file_handler = logging.FileHandler(os.path.join(log_dir, "htevolver.log"))
    file_handler.setLevel(logging.INFO)
    logger.addHandler(file_handler)
    file_formatter = logging.Formatter(fmt="%(asctime)s - %(name)s - [%(levelname)s] - %(message)s", datefmt="%Y-%m-%d %H:%M:%S")
    file_handler.setFormatter(file_formatter)

    logging.getLogger("engineio.client").setLevel(logging.ERROR)
    logging.getLogger("socketio.client").setLevel(logging.ERROR)
    logging.getLogger("aiohttp").setLevel(logging.ERROR)
    logging.getLogger("urllib3").setLevel(logging.ERROR)
    logging.getLogger("asyncio").setLevel(logging.ERROR)
    logging.getLogger("tecancavro").setLevel(logging.ERROR)


async def shutdown(app):
    """Cancel the broadcast task when shutting down.

    Gracefully cancels and waits for the broadcast task to complete when
    the application is shutting down.

    Args:
        app (Application): The AIOHTTP web application instance.
    """
    if app.get("broadcast_task"):
        app["broadcast_task"].cancel()
        try:
            await app["broadcast_task"]
        except asyncio.CancelledError:
            pass


async def broadcast_loop(app: Application):
    """Background task for periodic broadcasting of system status.

    Runs in the background to periodically broadcast the system state to all
    connected clients. Coordinates broadcasts between the robotics and eVOLVER
    namespaces, ensuring they don't interfere with each other or with immediate
    command execution.

    Args:
        app (Application): The AIOHTTP web application instance containing
            the namespace objects and broadcast timing configuration.

    Examples:
        This function is typically started as a background task:
        ```
        app["broadcast_task"] = asyncio.create_task(broadcast_loop(app))
        ```
    """
    last_time = 0.0
    while True:
        current_time = time.time()

        if (
            (last_time == 0.0 or (current_time - last_time >= app["broadcast_timing"]))
            and not app["evolver_namespace"].running_immediate
            and not app["evolver_namespace"].running_broadcast
        ):
            logger.info("Starting Broadcast Loop")
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
            last_time = start_time
            elapsed_time = end_time - start_time
            logger.info(f"Total Broadcast Processing Time: {elapsed_time}")

        # Non-blocking sleep to let the event loop handle other tasks
        await asyncio.sleep(0.1)


async def background_tasks(app):
    """Start background tasks after app startup and clean them up on shutdown.

    Used as a cleanup context for the web application. Creates and manages
    the broadcast task lifecycle, ensuring it's properly created on startup
    and cleaned up on shutdown.

    Args:
        app (Application): The AIOHTTP web application instance.

    Examples:
        This function is typically added to the application's cleanup context:
        ```
        app.cleanup_ctx.append(background_tasks)
        ```
    """
    app["broadcast_task"] = asyncio.create_task(broadcast_loop(app))
    logger.debug("starting broadcast task")

    yield
    logger.debug("closing background broadcast loop")
    app["broadcast_task"].cancel()
    with contextlib.suppress(asyncio.CancelledError):
        logger.debug("handling broadcast task close")
        await app["broadcast_task"]


def init() -> web.Application:
    """Initialize the web application with all required components.

    Creates and configures the AIOHTTP web application, Socket.IO server,
    namespace handlers, and loads configuration files. Sets up logging
    and background tasks.

    Returns:
        web.Application: The initialized AIOHTTP web application ready to be run.

    Examples:
        ```
        app = init()
        web.run_app(app, port=app["port"])
        ```
    """
    # Load configs
    evolver_conf_path = os.path.realpath(os.path.join("/home/pi/evolver", EVOLVER_CONF_FILENAME))
    robotics_conf_path = os.path.realpath(os.path.join("/home/pi/evolver", ROBOTICS_CONF_FILENAME))
    evolver_conf = {}
    robotics_conf = {}
    with open(evolver_conf_path, "r") as ymlfile:
        evolver_conf = yaml.safe_load(ymlfile)

    with open(robotics_conf_path, "r") as ymlfile:
        robotics_conf = yaml.safe_load(ymlfile)

    log_level = evolver_conf.get("log_level", "INFO").upper()
    logger.setLevel(getattr(logging, log_level))

    app = web.Application()
    app["port"] = evolver_conf["port"]
    app["broadcast_timing"] = evolver_conf["broadcast_timing"]

    sio = socketio.AsyncServer(always_connect=True)
    sio.attach(app)

    app["evolver_namespace"] = EvolverServerNamespace(evolver_conf, evolver_conf_path)
    app["robotics_namespace"] = RoboticsServerNamespace(robotics_conf, robotics_conf_path)
    sio.register_namespace(app["evolver_namespace"])
    sio.register_namespace(app["robotics_namespace"])

    app.cleanup_ctx.append(background_tasks)

    return app


def main():
    setup_logging()
    app = init()
    logger.info(f"Starting HT-eVOLVER server on port {app['port']}")
    web.run_app(app, port=app["port"])


if __name__ == "__main__":
    main()
