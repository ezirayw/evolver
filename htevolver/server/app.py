import logging
from contextlib import asynccontextmanager
from pathlib import Path

import yaml
from fastapi import FastAPI

from htevolver.server import evolver, robotics
from htevolver.server.robotics import Robotics

EVOLVER_CONF_FILENAME: str = "evolver_conf.yml"
ROBOTICS_CONF_FILENAME: str = "robotics_conf.yml"
LOGGING_DIR: Path = Path.joinpath(Path.home(), "pi", "logs")
logger = logging.getLogger("htevolver")
logger.setLevel(logging.INFO)


def setup_logging():
    if not LOGGING_DIR.is_dir():
        LOGGING_DIR.mkdir(parents=True)

    file_handler_path: Path = Path.joinpath(LOGGING_DIR, "htevolver.log")
    file_handler = logging.FileHandler(str(file_handler_path))
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


setup_logging()


@asynccontextmanager
async def lifespan(app: FastAPI):
    # create Evolver and Robotics instances, which manage state information for eVOLVER and Robotics hardware
    project_directory: Path = Path.cwd().parent.parent
    evolver_config_path: Path = project_directory.joinpath(EVOLVER_CONF_FILENAME)
    robotics_config_path: Path = project_directory.joinpath(ROBOTICS_CONF_FILENAME)

    with evolver_config_path.open("r") as conf_file:
        evolver_config: dict = yaml.safe_load(conf_file)
    with robotics_config_path.open("r") as conf_file:
        robotics_config: dict = yaml.safe_load(conf_file)

    yield {
        "evolver": evolver.Evolver(evolver_config, evolver_config_path),
        "robotics": Robotics(robotics_config, robotics_config_path),
    }
    ...


app = FastAPI(lifespan=lifespan)
app.include_router(evolver.router)
app.include_router(robotics.router)
