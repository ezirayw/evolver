import os
import numpy as np
import yaml
import aiohttp
import requests
import socketio
import asyncio
import shutil
import logging
import time
from dataclasses import dataclass, field, asdict
import skimage as ski
from xarm.wrapper import XArmAPI

logger = logging.getLogger(__name__)
ROBOTICS_CONF_FILENAME = "robotics_server_conf.yml"


#### CLASS DEFINITIONS ####
# define custom Exception that catches robotics errors, logs them, and stops robotics
class RoboticsError(Exception):
    def __init__(self, message: str):
        self.message = message
        logger.error("RoboticsError Found: %s" % message)


class FluidicEventError(Exception):
    def __init__(self, message: str):
        self.message = message
        logger.error("FluidicEventError Found: %s" % message)


class OctoPrintError(Exception):
    def __init__(self, message: str):
        self.message = message
        logger.error("OctoPrintError Found: %s" % message)


class HelperEventError(Exception):
    def __init__(self, message: str, robotics_server: "RoboticsServer"):
        self.message = message
        logger.error("HelperEventError Found: %s" % message)


class ExitRobotics(Exception):
    def __init__(self):
        logger.info("ExitRobotics raised in response to detected exit command")


# define namespace events to communicate with eVOLVER server
class EvolverNamespace(socketio.ClientNamespace):
    # first index for vial, second index for quad
    # overflow_data = {"left": [], "right": []}

    def on_connect(self, *args):
        logger.info("robotics_eVOLVER connected to base_eVOLVER server")

    def on_disconnect(self, *args):
        logger.info("robotics_eVOLVER disconnected from base_eVOLVER server")

    def on_reconnect(self, *args):
        logger.info("robotics_eVOLVER reconnected to base_eVOLVER as server")

    # def on_broadcast(self, data):
    #     if data["phase"] == 1:
    #         self.overflow_data["left"] = (
    #             data["config"].get("overflow_left", {}).get("value", None)
    #         )
    #         self.overflow_data["left"] = [
    #             x / 100 for x in self.overflow_data["left"]
    #         ]  # convert back to proper voltage

    #         self.overflow_data["right"] = (
    #             data["config"].get("overflow_right", {}).get("value", None)
    #         )
    #         self.overflow_data["right"] = [
    #             x / 100 for x in self.overflow_data["right"]
    #         ]  # convert back to proper voltage

    def fluid_command(self, MESSAGE):
        logger.info("fluid command: %s" % MESSAGE)
        command = {
            "param": "pump",
            "value": MESSAGE,
            "recurring": False,
            "immediate": True,
        }
        self.emit("command", command, namespace="/default_evolver")


@dataclass
class OctoPrintInterface:
    name: str
    evolver_ip: str
    octoprint_conf: dict
    connected: bool = field(default=False)
    post_gcode_timeout: int = field(init=False)
    api_key: str = field(init=False)
    gcode_dir: str = field(init=False)
    instance_dir: str = field(init=False)
    base_url: str = field(init=False)
    id: str = field(init=False)

    def __post_init__(self):
        self.post_gcode_timeout = self.octoprint_conf["post_gcode_timeout"]
        self.gcode_dir = self.octoprint_conf["gcode_dir"]
        self.instance_dir = self.octoprint_conf["octoprint_dir"]
        self.base_url = (
            "http://" + self.evolver_ip + ":" + str(self.octoprint_conf["port"])
        )
        self.id = self.octoprint_conf["octoprint_id"]

        # load in user.yaml file to get api_key
        user_conf_filepath = os.path.join(self.instance_dir, "users.yaml")
        with open(user_conf_filepath, "r") as conf:
            user_conf = yaml.safe_load(conf)
            self.api_key = user_conf["ht_evolver"]["apikey"]

    def connect(self):
        """Connect OctoPrint instance to pumps on smoothieboard using OctoPrint API."""

        # send POST request to OctoPrint server
        payload = {"command": "connect"}
        header = {"X-Api-Key": self.api_key}
        temp_url = self.base_url + "/api/connection"

        try:
            response = requests.post(temp_url, headers=header, json=payload)
            if response.status_code == 204:
                logger.debug("connection to pumps for %s successful" % self.name)
                self.connected = True
            if response.status_code == 400:
                logger.warning("cannot connect to pumps on %s" % self.name)
        except Exception as e:
            raise OctoPrintError(
                "could not connect to pumps on %s due to error: %s" % (self.name, e)
            )

    def disconnect(self):
        """Disconnect OctoPrint instance from pumps on smoothieboard using OctoPrint API."""

        # send POST request to OctoPrint server
        payload = {"command": "disconnect"}
        header = {"X-Api-Key": self.api_key}
        temp_url = self.base_url + "/api/connection"

        try:
            response = requests.post(temp_url, headers=header, json=payload)
            if response.status_code == 204:
                logger.debug("disconnect from pumps on %s successful" % self.name)
                self.connected = False
            if response.status_code == 400:
                logger.warning("cannot disconnect from pumps on %s" % self.name)
        except Exception as e:
            raise OctoPrintError(
                "could not disconnect to pumps on %s due to error: %s" % (self.name, e)
            )

    def cancel(self):
        """Cancel any current pump jobs on smoothieboard using OctoPrint API.
        Raises:
            OctoPrintError: If there is an error while canceling the pump jobs.
        """

        if self.connected == False:
            raise OctoPrintError(
                "cannot cancel current job on %s, not connected to syringe pump"
                % self.name
            )

        # send POST request to OctoPrint server
        payload = {"command": "cancel"}
        header = {"X-Api-Key": self.api_key}
        temp_url = self.base_url + "/api/job"

        try:
            response = requests.post(temp_url, headers=header, json=payload)
            if response.status_code == 204:
                logger.debug(
                    "cancellation of active pump jobs on %s successful" % self.name
                )
            if response.status_code == 409:
                logger.warning("no active pump jobs found to cancel on %s" % self.name)
        except Exception as e:
            raise OctoPrintError(
                "could not cancel active jobs on %s due to error: %s" % (self.name, e)
            )

    async def get_status(self, session):
        """Check status of OctoPrint instance, including any active jobs, using the OctoPrint API.

        Args:
            session (aiohttp.ClientSession): The client session used to make HTTP requests.
        Returns:
            response.json (dict): A dictionary containing the result of the job status check.
        Raises:
            OctoPrintError: If there is an error while checking the job status."""

        url = self.base_url + "/api/job"
        header = {"X-Api-Key": self.api_key}
        try:
            async with session.get(url, headers=header) as response:
                return await response.json()
        except Exception as e:
            raise OctoPrintError(
                "could not check job successfully for %s: %s" % (self.name, e)
            )

    def write_gcode(
        self, mode: str, pump_instructions: dict, pump_conf: dict, primed_status: bool
    ):
        """Writes G-code instructions to local file based on the given mode and pump steps.

        Args:
            mode (str): The mode of operation. Can be 'aspirate', 'dispense', or 'prime_pumps'.
            pump_instructions (dict): stores syringe pump actuation steps under pump name key. e.g. {'pump_name': steps}
            pump_conf (dict): contains pump configuration.
            primed_status (bool): A boolean indicating whether the pumps are primed."""

        plunger_commands = []
        prime_commands = []
        valve_commands = {"on": [], "off": [], "steps": 0}
        gcode = ""

        # mode specific modifications to gcode to account for valve actuation and/or priming
        if mode == "aspirate":
            for pump in pump_instructions:
                # get pump port settings
                plunger_motor = pump_conf["pumps"][pump]["motor_connections"]["plunger"]
                plunger_commands.append(
                    "{0}-{1}".format(plunger_motor, pump_instructions[pump])
                )

            # combine gcode commands
            # command = plunger_commands[0] + ' ' + plunger_commands[1]
            combined_plunger_commands = " ".join(plunger_commands)
            gcode = "G91\nG1 {0} F{1}\nM18".format(
                combined_plunger_commands, pump_conf["plunger_speed_in"]
            )

        elif mode == "dispense":
            for pump in pump_instructions:
                plunger_motor = pump_conf["pumps"][pump]["motor_connections"]["plunger"]
                valve_motor = pump_conf["pumps"][pump]["motor_connections"]["valve"]
                valve_steps_on = pump_conf["pumps"][pump]["motor_connections"][
                    "valve_steps"
                ]
                valve_steps_off = (
                    pump_conf["pumps"][pump]["motor_connections"]["valve_steps"] * -1
                )

                if primed_status and pump_instructions[pump] != 0:
                    plunger_commands.append(
                        "{0}{1}".format(
                            plunger_motor,
                            pump_instructions[pump] + pump_conf["priming_steps"],
                        )
                    )
                    prime_commands.append(
                        "{0}-{1}".format(plunger_motor, pump_conf["priming_steps"])
                    )
                else:
                    plunger_commands.append(
                        "{0}{1}".format(plunger, pump_instructions[pump])
                    )
                    prime_commands.append("")

                if pump_instructions[pump] == 0:
                    valve_commands["on"].append("{0}{1}".format(valve_motor, 0))
                    valve_commands["off"].append("{0}{1}".format(valve_motor, 0))
                else:
                    valve_commands["on"].append(
                        "{0}{1}".format(valve_motor, valve_steps_on)
                    )
                    valve_commands["off"].append(
                        "{0}{1}".format(valve_motor, valve_steps_off)
                    )

            # combine gcode commands
            # valve_on = valve_commands['on'][0] + ' ' + valve_commands['on'][1]
            combined_valve_on_commands = " ".join(valve_commands["on"])
            combined_valve_off_commands = " ".join(valve_commands["off"])
            combined_plunger_commands = " ".join(plunger_commands)
            combined_prime_commands = " ".join(prime_commands)

            if primed_status and pump_instructions[pump] != 0:
                gcode = "G91\nG1 {0} F25000\nG4 P25\nG1 {1} F{2}\nG4 P25\nG1 {3} F15000\nG4 P25\nG1 {4} F25000\nM18".format(
                    combined_valve_on_commands,
                    combined_plunger_commands,
                    pump_conf["plunger_speed_out"],
                    combined_prime_commands,
                    combined_valve_off_commands,
                )
            else:
                gcode = "G91\nG1 {0} F25000\nG4 P25\nG1 {1} F{2}\nG4 P25\nG1 {3} F25000\nM18".format(
                    combined_valve_on_commands,
                    combined_plunger_commands,
                    pump_conf["plunger_speed_out"],
                    combined_valve_off_commands,
                )

        elif mode == "prime_pumps":
            for pump in pump_instructions:
                plunger_motor = pump_conf["pumps"][pump]["motor_connections"]["plunger"]
                valve_motor = pump_conf["pumps"][pump]["motor_connections"]["valve"]
                valve_steps_on = pump_conf["pumps"][pump]["motor_connections"][
                    "valve_steps"
                ]
                valve_steps_off = (
                    pump_conf["pumps"][pump]["motor_connections"]["valve_steps"] * -1
                )

                plunger_commands.append(
                    "{0}{1}".format(plunger_motor, pump_instructions[pump])
                )
                valve_commands["on"].append(
                    "{0}{1}".format(valve_motor, valve_steps_on)
                )
                valve_commands["off"].append(
                    "{0}{1}".format(valve_motor, valve_steps_off)
                )

            # combine gcode commands
            combined_valve_on_commands = " ".join(valve_commands["on"])
            combined_valve_off_commands = " ".join(valve_commands["off"])
            combined_plunger_commands = " ".join(plunger_commands)
            gcode = "G91\nG1 {0} F25000\nG4 P25\nG1 {1} F{2}\nG4 P25\nG1 {3} F25000\nM18".format(
                combined_valve_on_commands,
                combined_plunger_commands,
                pump_conf["plunger_speed_in"],
                combined_valve_off_commands,
            )

        else:
            gcode = "M18"

        # write command to gcode file
        filename = mode + ".gcode"
        gcode_path = os.path.join(self.gcode_dir, filename)
        f = open(gcode_path, "w")
        f.write(gcode)
        f.close()

    async def post_gcode_async(self, session, gcode_path: str):
        """Upload a G-code file to OctoPrint instance to actuate syringe pump using OctoPrint API.

        Args:
            session (aiohttp.ClientSession()): The session object used for making HTTP requests.
            gcode_path (str): The path to the G-code file to be sent.

        Raises:
            OctoPrintError: If there is an error while sending the G-code file."""

        if self.connected == False:
            raise OctoPrintError(
                "cannot post gcode to %s, not connected to pumps" % self.name
            )

        url = self.base_url + "/api/files/local"
        header = {"X-Api-Key": self.api_key}
        request_attempts = 0

        while True:
            try:
                with open(gcode_path, "rb") as f:
                    # send POST request to OctoPrint server
                    payload = {"file": f, "print": "true"}
                    async with session.post(
                        url, headers=header, data=payload
                    ) as response:
                        result = await response.json()
                        logger.debug(result)
                        if result["done"]:
                            break

            except KeyError as e:
                logger.debug(
                    "could not recognize successful gcode POST event to %s, trying again: %s"
                    % (self.name, e)
                )
                request_attempts = request_attempts + 1
                if request_attempts > self.post_gcode_timeout:
                    raise OctoPrintError(
                        "could not post gcode to %s after %s attempts"
                        % (self.name, self.robotics_conf["post_request_timeout"])
                    )
                await asyncio.sleep(0.25)  # wait 0.25 second before trying again

            except Exception as e:
                raise


@dataclass
class RoboticsStatus:
    mode: str = "idle"
    active_quad: str = None
    active_pumps: list = field(default_factory=list)
    vial_window: list = field(default_factory=list)
    overflow_status: dict = field(
        default_factory=lambda: {"quads": [False, False, False, False], "vial": None}
    )
    xArm: dict = field(
        default_factory=lambda: {
            "warning_code": 0,
            "error_code": 0,
            "arm_state": None,
            "connected": None,
        }
    )
    OctoPrint: dict = field(default_factory=dict)
    prime_status: dict = field(
        default_factory=lambda: {"influx": False, "efflux": False}
    )


@dataclass
class RoboticsServer(EvolverNamespace):
    sio: socketio.AsyncServer = field(
        default_factory=lambda: socketio.AsyncServer(always_connect=True)
    )
    evolver_ns: EvolverNamespace = None
    robotics_status: RoboticsStatus = field(default_factory=RoboticsStatus)
    server_path: str = field(init=False)
    robotics_conf_path: str = field(init=False)
    robotics_conf: dict = field(init=False)
    pump_conf: dict = field(init=False)
    dilutions_path: str = field(init=False)
    xarm_ip: str = field(init=False)
    octoprint_instances: dict = field(default_factory=dict)
    arm: XArmAPI = field(init=False)

    def __post_init__(self):
        self.server_path = os.path.dirname(os.path.abspath(__file__))
        self.robotics_conf_path = os.path.join(
            self.server_path, "robotics_server_conf.yml"
        )
        with open(self.robotics_conf_path, "r") as conf:
            self.robotics_conf = yaml.safe_load(conf)
        self.pump_conf = self.robotics_conf["pump_conf"]
        self.dilutions_path = os.path.join(self.server_path)
        self.xarm_ip = self.robotics_conf["xarm_ip"]
        for octoprint_name in self.robotics_conf["octoprint_instances"]:
            self.octoprint_instances[octoprint_name] = OctoPrintInterface(
                octoprint_name,
                self.robotics_conf["evolver_ip"],
                self.robotics_conf["octoprint_instances"][octoprint_name],
            )
            self.robotics_status.OctoPrint[octoprint_name] = {
                "connection_status": self.octoprint_instances[octoprint_name].connected
            }
            gcode_dir_exist = os.path.exists(
                self.robotics_conf["octoprint_instances"][octoprint_name]["gcode_dir"]
            )
            if gcode_dir_exist:
                shutil.rmtree(
                    self.robotics_conf["octoprint_instances"][octoprint_name][
                        "gcode_dir"
                    ]
                )
            os.makedirs(
                self.robotics_conf["octoprint_instances"][octoprint_name]["gcode_dir"]
            )

        self.arm = XArmAPI(self.xarm_ip, enable_report=True)
        self.arm.clean_warn()
        self.arm.clean_error()
        self.arm.motion_enable(enable=True)
        self.arm.set_state(state=0)
        self.arm.set_mode(0)
        self.arm.set_collision_sensitivity(2)
        self.arm.set_self_collision_detection(True)
        # handle potential C21 kinematic errors (align end effector to be parallel to ground)
        code, angles = self.arm.get_servo_angle()
        if code == 0:
            angles[3] = -(angles[1] + angles[2])
            self.arm.set_servo_angle(angle=angles, wait=True)
        self.robotics_status.xArm["connected"] = self.arm.connected
        logger.info("robotics_evolver server initialized")

    def update_conf(self):
        """Updates the robotics + pump configuration by loading the contents of the robotics_conf file."""

        with open(self.robotics_conf_path, "r") as conf:
            self.robotics_conf = yaml.safe_load(conf)
        self.pump_conf = self.robotics_conf["pump_conf"]

    def register_callback(self):
        """Register the error_warn_changed_callback and state_changed_callback for the xArm 5."""

        self.arm.register_error_warn_changed_callback(
            callback=self.error_warn_change_callback
        )
        self.arm.register_state_changed_callback(callback=self.state_changed_callback)
        self.arm.register_connect_changed_callback(
            callback=self.connect_changed_callback
        )

    def error_warn_change_callback(self, data: dict):
        """Update the error and warning codes in the robotics_status class attribute.

        Args:
            data (dict): A dictionary containing the error and warning codes."""

        self.robotics_status.xArm["error_code"] = data["error_code"]
        self.robotics_status.xArm["warn_code"] = data["warn_code"]
        if data["error_code"] != 0:
            logger.error("xArm error_code %s encountered." % data["error_code"])
            self.stop_robotics()
        if data["warn_code"] != 0:
            logger.warning("xArm warning_code %s encountered." % data["warn_code"])

    def state_changed_callback(self, data: dict):
        """Update the arm state in the robotics_status class attribute based on the received data.

        Args:
            data (dict): contains the xArm state."""

        logger.debug(data)
        self.robotics_status.xArm["arm_state"] = data["state"]
        if data["state"] == 4:
            logger.error("xArm entered error state")
            self.stop_robotics()

    def connect_changed_callback(self, data: dict):
        """Update the arm connection status in the robotics_status class attribute based on the received data.

        Args:
            data (dict): A dictionary containing the connection status."""

        logger.info("xArm connection status changed to %s" % data["connected"])
        self.robotics_status.xArm["connected"] = data["connected"]

    def attach(self, app):
        """Attach the the robotics_server to the given Flask app.

        Args:
            app (Flask): The Flask app to attach the robotics_server to."""

        self.sio.attach(app)
        logger.debug("robotics server attached")

    def setup_client(self, socketIO_client: socketio.Client):
        """Set up the robotics_server as an eVOLVER client by registering the EvolverNamespace.

        Args:
            socketIO_client (socketio.Client): The socketIO client to set up."""

        self.evolver_ns = EvolverNamespace("/default_evolver")
        socketIO_client.register_namespace(self.evolver_ns)

    def stop_robotics(self):
        """Stop all robotics and syringe pump operations, including canceling current jobs and disconnecting from OctoPrint servers. Used in cases of emergencies or unforseen errors. Requires manual intervention to restart."""

        self.robotics_status.mode = "emergency_stop"
        logger.error("stop_robotics() called - check logs to identify cause of error")

        # emergency stop xArm, cancel current syringe job commands, and disconnect from OctoPrint servers
        try:
            for octoprint_name in self.octoprint_instances:
                self.octoprint_instances[octoprint_name].cancel()
                self.octoprint_instances[octoprint_name].disconnect()
            self.arm.emergency_stop()
            self.arm.disconnect()
        except OctoPrintError as e:
            logger.exception(e)

    def reset_xArm(self):
        """Clear potential warnings/errors and align end effector."""
        self.arm.clean_warn()
        self.arm.clean_error()
        self.arm.motion_enable(True)
        self.arm.set_state(0)
        code, angles = self.arm.get_servo_angle()
        if code == 0:
            angles[3] = -(angles[1] + angles[2])
            self.arm.set_servo_angle(angle=angles, wait=True)

    def map_gcode_commands(self, instructions: dict):
        """Maps pump instructions to cogante OctoPrint instances.

        Args:
            instructions (dict): contains unmapped pump instructions. e.g. {'pump_name': steps, 'pump_name': steps, ...}
        Returns:
            octoprint_instructions_map (dict): contains pump instructions mapped to cognate OctoPrint instance.
            e.g. {octoprint_0: {'pump_name': steps, 'pump_name': steps}, octoprint_1: {'pump_name': volume, 'pump_name': volume}, ...}"""

        # use pump_config to map pump instructions to cognate OctoPrint instances
        # should look like following: {octoprint_0: {pump_target: volume, pump_target: volume}, octoprint_1: {pump_target: volume, pump_target: volume}, ...}
        octoprint_instructions_map = {}
        for octoprint_name in self.octoprint_instances:
            octoprint_instructions = {}
            for pump_target in instructions:
                if (
                    pump_target in self.pump_conf["pumps"]
                    and pump_target in self.octoprint_instances[octoprint_name]["pumps"]
                ):
                    octoprint_instructions[pump_target] = instructions[pump_target]
            octoprint_instructions_map[octoprint_name] = octoprint_instructions

        return octoprint_instructions_map

    def make_ipp_command(
        self, duration: int, frequency: int, ipp_address_key: str, polarity: int
    ):
        """Create a custom, eVOLVER-formatted ipp command.

        Args:
            duration (int): Time duration in seconds to actuate the target IPP.
            frequency (int): Actuation frequency (Hz) to actuate target IPP.
            ipp_address_key (str): The IPP address key, used to map to external solenoids.
            polarity (int): The polarity of IPP actuation, dictates flow direction.

        Returns:
            ipp_command (list): A list containing IPP commands."""

        # load in current config
        self.update_conf()

        # build ipp efflux command
        ipp_command = ["--"] * 48  # empty command
        ipp_index = 1

        for ipp_address in self.robotics_conf["ipp_mapping"][ipp_address_key][
            "solenoid_numbers"
        ]:
            ipp_command[ipp_address] = "{0}|{1}|{2}|{3}".format(
                frequency,
                self.robotics_conf["ipp_mapping"][ipp_address_key]["ipp_number"],
                ipp_index,
                duration,
            )
            ipp_index = ipp_index + 1

        return ipp_command

    async def check_status(self, session, gcode_paths: list[str] = None):
        """Check the status of the OctoPrint servers, and if G-code paths are provided, the completion status of certain syringe pumps.

        Args:
            session (aiohttp.Client()): The session object for making HTTP requests.
            gcode_paths (list): A list of G-code file paths to check the status

        Returns:
            result (dict): contains the states of the OctoPrint servers and the completion status of pump jobs. Check OctoPrint docs for more information on the response format."""

        status_tasks = []
        all_jobs_complete = 0
        if gcode_paths == None:
            gcode_paths = []
        result = {"accept_new_jobs": False, "octoprint_statuses": {}}

        # create get_status() tasks for each OctoPrint instance
        for octoprint_name in self.octoprint_instances:
            status_tasks.append(
                self.octoprint_instances[octoprint_name].get_status(session)
            )
            result["octoprint_statuses"][octoprint_name] = {}

        try:
            status_results = await asyncio.gather(*status_tasks, return_exceptions=True)

            # parse through status results to check OctoPrint state and pump job completion, if gcode paths are given
            for octoprint_name in self.octoprint_instances:
                index = self.octoprint_instances[octoprint_name].id
                logger.debug("check status results")
                logger.debug(status_results[index])
                if isinstance(status_results[index], OctoPrintError):
                    raise status_results[index]
                else:
                    result["octoprint_statuses"][octoprint_name] = status_results[index]

                if gcode_paths:
                    filename = os.path.split(gcode_paths[index])[1]
                    if (
                        (status_results[index]["progress"]["completion"] >= 100)
                        and (status_results[index]["state"] == "Operational")
                        and (status_results[index]["job"]["file"]["name"] == filename)
                    ):
                        all_jobs_complete += 1

            if all_jobs_complete == len(gcode_paths):
                result["accept_new_jobs"] = True

        except Exception as e:
            logger.debug("check status issue occured: %s" % e)
        finally:
            return result

    async def fluidic_event(
        self,
        session,
        gcode_files: list[str],
        mode: str,
        arm_settings: dict = None,
        print_string: str = None,
    ):
        """Perform aspiration and dispense actions with syringe pumps that can be coordinated with the xArm.

        Args:
            session (aiohttp.Client()): The session object for the HTTP request.
            gcode_files (list): A list of two filepaths the aspirate and dispense G-code files.
            mode (string): Can be one of the following ('[global_mode]_influx', '[global_mode]_wash', '[global_mode]_pipette')
            arm_settings (dict): Contains the necessary information to control desired xArm path.
            print_string (string): A string representing additional information about the fluidic event for logging.

        Raises:
        - FluidicEventError: If there is an error running the fluidic event."""

        try:
            await (
                self.check_for_interrupt()
            )  # hang here if pause is called or exit routine if exit command received
        except ExitRobotics:
            raise
        self.robotics_status.mode = mode

        # create aspiration commands
        logger.info("running aspiration during %s for: %s" % (mode, print_string))
        check_files = []
        try:
            async with asyncio.TaskGroup() as aspiration_tasks:
                for octoprint_name in self.octoprint_instances:
                    gcode_path = os.path.join(
                        self.octoprint_instances[octoprint_name].gcode_dir,
                        gcode_files[0],
                    )
                    check_files.append(gcode_path)
                    aspiration_tasks.create_task(
                        self.octoprint_instances[octoprint_name].post_gcode_async(
                            session, gcode_path
                        )
                    )

                if arm_settings:
                    aspiration_tasks.create_task(self.arm_path(arm_settings))
        except* RoboticsError:
            # reset xArm to clear potential errors and try again
            self.reset_xArm()
            try:
                self.arm_path(arm_settings)
            except* RoboticsError:
                raise FluidicEventError(
                    "Successive xArm errors detected after retyring post-reset during aspiration tasks"
                )
        except* OctoPrintError:
            # try to reset OctoPrint and try again
            raise FluidicEventError(
                "OctoPrint error detected when trying to execute aspiration tasks"
            )
        except* ExitRobotics:
            raise
        except* Exception:
            raise FluidicEventError(
                "Unforseen error encountered when trying to execute aspiration tasks, check logs for traceback"
            )

        try:
            await (
                self.check_for_interrupt()
            )  # hang here if pause is called or exit routine if exit command received
        except ExitRobotics:
            raise
        self.robotics_status.mode = mode

        # verify that syringe_pumps are ready to receive dispense commands
        check_attempts = 0
        while True:
            check = await self.check_status(session, check_files)
            logger.debug("checking status of aspiration commands")
            logger.debug(check)
            if check["accept_new_jobs"]:
                break
            else:
                if check_attempts > self.robotics_conf["check_status_timout"]:
                    raise FluidicEventError(
                        "check attempts for aspiration fluidic event exceeded max attempts, check OctoPrint server logs"
                    )
                check_attempts = check_attempts + 1
                await asyncio.sleep(0.25)

        try:
            await (
                self.check_for_interrupt()
            )  # hang here if pause is called or exit routine if exit command received
        except ExitRobotics:
            raise
        self.robotics_status.mode = mode

        # create dispense commands
        logger.info("running dispense during %s for: %s" % (mode, print_string))
        check_files = []
        try:
            async with asyncio.TaskGroup() as dispense_tasks:
                for octoprint_name in self.octoprint_instances:
                    gcode_path = os.path.join(
                        self.octoprint_instances[octoprint_name].gcode_dir,
                        gcode_files[1],
                    )
                    dispense_tasks.create_task(
                        self.octoprint_instances[octoprint_name].post_gcode_async(
                            session, gcode_path
                        )
                    )
                    check_files.append(gcode_path)
        except* OctoPrintError:
            # try to reset OctoPrint and try again
            raise FluidicEventError(
                "OctoPrint error detected when trying to execute dispense tasks"
            )
        except* Exception:
            raise FluidicEventError(
                "Unforseen error encountered when trying to execute dispense tasks, check logs for traceback"
            )

        try:
            await (
                self.check_for_interrupt()
            )  # hang here if pause is called or exit routine if exit command received
        except ExitRobotics:
            raise
        self.robotics_status.mode = mode

        # verify that syringe pumps are ready to receive future commands
        check_attempts = 0
        while True:
            check = await self.check_status(session, check_files)
            logger.debug("checking status of dispense commands")
            logger.debug(check)
            if check["accept_new_jobs"]:
                break
            else:
                if check_attempts > self.robotics_conf["check_status_timout"]:
                    raise FluidicEventError(
                        "check attempts for dispense fluidic event exceeded max attempts, check OctoPrint server logs"
                    )
                check_attempts = check_attempts + 1
                await asyncio.sleep(0.25)

        logger.info("finished %s for: %s" % (mode, print_string))

    async def prime_influx_helper(self):
        """Helper method to prime the syringe pumps by createing air gap in pippette

        Returns:
            result (dict): A dictionary with the result of the priming operation."""

        # update robotics status
        self.robotics_status.mode = "priming_influx"

        # start asyncio Client Session
        session = aiohttp.ClientSession()

        # load in current config
        self.update_conf()

        # get fluid pumps from server_conf and write prime pumps instructions
        pump_instructions = {}
        print_string = ""

        for pump in self.pump_conf["pumps"]:
            print_string = print_string + "{0} pump".format(pump)
            pump_instructions[pump] = self.pump_conf["priming_steps"]

        mapped_pump_instructions = self.map_gcode_commands(pump_instructions)
        for octoprint_name in mapped_pump_instructions:
            self.octoprint_instances[octoprint_name].write_gcode(
                "prime_pumps",
                mapped_pump_instructions[octoprint_name],
                self.pump_conf,
                self.robotics_status.prime_status["influx"],
            )

        # create syringe_pump priming commands
        try:
            prime_pumps_tasks = []
            check_files = []

            async with asyncio.TaskGroup() as prime_pumps_tasks:
                for octoprint_name in self.octoprint_instances:
                    gcode_path = os.path.join(
                        self.octoprint_instances[octoprint_name].gcode_dir,
                        "prime_pumps.gcode",
                    )
                    prime_pumps_tasks.create_task(
                        self.octoprint_instances[octoprint_name].post_gcode_async(
                            session, gcode_path
                        )
                    )
                    check_files.append(gcode_path)

            # verify that syringe pumps are ready to receive future commands
            check_attempts = 0
            while True:
                check = await self.check_status(session, check_files)
                logger.debug(
                    "checking status of OctoPrint commands for prime_influx_helper"
                )
                logger.debug(check)
                if check["accept_new_jobs"]:
                    break
                else:
                    if check_attempts > self.robotics_conf["check_status_timout"]:
                        raise FluidicEventError(
                            "check attempts for dispense fluidic event exceeded max attempts, check OctoPrint server logs"
                        )
                    check_attempts = check_attempts + 1
                    await asyncio.sleep(0.25)

            # update robotics status upon successful priming
            logger.info("syringe pumps successfully primed")
            await session.close()
        except Exception:
            await session.close()
            raise HelperEventError(
                "error running prime_influx_helper, check logs for traceback"
            )

    async def efflux_ipp_helper(self, data: dict):
        """Helper method to use efflux IPPs in polarity for efflux during influx routines or to add media/fluids into vials.

        Args:
            data (dict):
                polarity (int): The polarity of the efflux IPP actuation. Can be 0 (towards waste) or 1 (into vials).
                duration (int): The duration of the efflux IPP actuation, in seconds.
                frequency (int): Number of actuation events per second, Hz. Dictates flow rate.
        Returns:
            result (dict): A dictionary with the result of the priming operation."""

        # load in current config
        self.update_conf()

        # use parameters found in data, otherwise pull from defaults found in robotics_conf
        efflux_commands = []
        for quad in data:
            ipp_address_key = quad + "_efflux"
            if "duration" in data[quad]:
                duration = data[quad]["duration"]
            else:
                duration = self.robotics_conf["ipp_efflux_settings"]["duration"]

            if "frequency" in data[quad]:
                frequency = data[quad]["frequency"]
            else:
                frequency = self.robotics_conf["ipp_efflux_settings"]["frequency"]

            if "polarity" in data[quad]:
                polarity = data[quad]["polarity"]
            else:
                polarity = self.robotics_conf["ipp_efflux_settings"]["polarity"]
            efflux_commands.append(
                self.make_ipp_command(duration, frequency, ipp_address_key, polarity)
            )

        ipp_efflux_command = ["--"] * 48  # empty command
        # collapse generated commands to a single command
        for index in range(len(efflux_commands[0])):
            for ipp_command in efflux_commands:
                if ipp_command[index] != "--":
                    ipp_efflux_command[index] = ipp_command[index]

        # send efflux command
        self.evolver_ns.fluid_command(ipp_efflux_command)

    async def pipette_helper(self, data: dict):
        """Helper method to run a single pipette action using syringe pumps.

        Returns:
            result (dict): A dictionary with the result of the pipette_helper operation"""

        # start asyncio Client Session
        session = aiohttp.ClientSession()

        # load in current config
        self.update_conf()

        # build gcode instructions for OctoPrint instances
        pipette_instructions = {}
        print_string = "pipetting with syringe pumps"

        for pump in data:
            if pump in self.pump_conf["pumps"]:
                print_string = print_string + " {0} pump ".format(pump)
                pipette_instructions[pump] = data[pump]
            else:
                raise HelperEventError(
                    "pump {0} not found in pump configuration".format(pump)
                )
        mapped_pipette_instructions = self.map_gcode_commands(pipette_instructions)

        for octoprint_name in mapped_pipette_instructions:
            self.octoprint_instances[octoprint_name].write_gcode(
                "aspirate",
                mapped_pipette_instructions[octoprint_name],
                self.pump_conf,
                self.robotics_status.prime_status["influx"],
            )
            self.octoprint_instances[octoprint_name].write_gcode(
                "dispense",
                mapped_pipette_instructions[octoprint_name],
                self.pump_conf,
                self.robotics_status.prime_status["influx"],
            )
        gcode_files = ["aspirate.gcode", "dispense.gcode"]

        try:
            await self.fluidic_event(
                session, gcode_files, "pipette", print_string=print_string
            )
            await session.close()
        except ExitRobotics:
            await session.close()
            raise
        except Exception:
            await session.close()
            raise HelperEventError(
                "Error running pipette_helper - stopping robotics. Check logs for traceback"
            )

    async def influx_snake_helper(self, data: dict):
        """Helper function for executing sequential fluidic_event(s) in a snake-like pattern across Smart Quads. Use for dilution and fill_vial routines events if xArm is desired.

        Args:
            data (dict):
                'command': (dict) contains syringe pump and ipp commands.
                'target_quads': (list) list of target smart quads for influx commands
                'mode': (str) mode of operation. Can be 'dilution' or 'filling_vials'.

        Raises:
            HelperEventError: Raised in the event a FluidicEvent Exception is caught to coordinate experiment management."""

        syringe_pump_commands = data["commands"]["syringe_pump_command"]
        # start asyncio Client Session
        session = aiohttp.ClientSession()

        # load in current config
        self.update_conf()

        # loop through sets of vials (vial_window) and execute fluid dispension events
        vial_map = [[0, 1, 2, 3, 4, 5], [11, 10, 9, 8, 7, 6], [12, 13, 14, 15, 16, 17]]
        for quad_name in data["target_quads"]:
            self.robotics_status.active_quad = quad_name
            vial_dilution_coordinates = [-18, 36]
            wash_station_coordinates = [72, -29]
            change_row = False

            # calculate euclidean transformation matrix to convert vial_coordinates into arm coordinates using homing calibration
            # home based on vial_0 and vial_17 for each smart quad
            vial_0_out = np.array(
                [
                    self.robotics_conf["homing_coordinates"][quad_name]["vial_0"][
                        "x_out"
                    ],
                    self.robotics_conf["homing_coordinates"][quad_name]["vial_0"]["y"],
                ]
            )
            vial_0_in = np.array(
                [
                    self.robotics_conf["homing_coordinates"][quad_name]["vial_0"][
                        "x_in"
                    ],
                    self.robotics_conf["homing_coordinates"][quad_name]["vial_0"]["y"],
                ]
            )

            vial_17_out = np.array(
                [
                    self.robotics_conf["homing_coordinates"][quad_name]["vial_17"][
                        "x_out"
                    ],
                    self.robotics_conf["homing_coordinates"][quad_name]["vial_17"]["y"],
                ]
            )
            vial_17_in = np.array(
                [
                    self.robotics_conf["homing_coordinates"][quad_name]["vial_17"][
                        "x_in"
                    ],
                    self.robotics_conf["homing_coordinates"][quad_name]["vial_17"]["y"],
                ]
            )

            z_vial_dilution = np.array(
                [
                    self.robotics_conf["homing_coordinates"][quad_name]["vial_0"][
                        "z_out"
                    ],
                    self.robotics_conf["homing_coordinates"][quad_name]["vial_0"][
                        "z_in"
                    ],
                ]
            )
            z_wash_station = np.array(
                [
                    self.robotics_conf["homing_coordinates"][quad_name]["wash_station"][
                        "z_out"
                    ],
                    self.robotics_conf["homing_coordinates"][quad_name]["wash_station"][
                        "z_in"
                    ],
                ]
            )

            vial_coordinates = np.array([[0, 36], [90, 0]])
            calibrated_coordinates_out = np.array([vial_0_out, vial_17_out])
            calibrated_coordinates_in = np.array([vial_0_in, vial_17_in])

            transform_matrix_out = self.rigid_transform(
                vial_coordinates, calibrated_coordinates_out
            )
            transform_matrix_in = self.rigid_transform(
                vial_coordinates, calibrated_coordinates_in
            )
            transform_matrices = np.stack((transform_matrix_out, transform_matrix_in))

            for row_num in range(np.size(vial_map, 0)):
                current_vial_row = vial_map[row_num]

                # get list of pumps from config
                pump_map = []
                pump_types = []
                pump_num = len(self.pump_conf["pumps"])
                for pump_position in range(pump_num):
                    for pump in self.pump_conf["pumps"]:
                        if self.pump_conf["pumps"][pump]["position"] == pump_position:
                            pump_map.append(pump)
                            pump_types.append(self.pump_conf["pumps"][pump]["type"])
                            break

                # flip pump map to account for leading pump position changing in middle row of quad
                if row_num == 1:
                    pump_map.reverse()

                # check if pumps have the same fluid type to configure number of vial windows and arm path
                num_vial_windows = None
                uniform_pump_types = None
                if len(set(pump_types)) > 1:
                    num_vial_windows = 6 + (
                        pump_num - 1
                    )  # overhang vial window for pumps with different fluid types
                    uniform_pump_types = False
                if len(set(pump_types)) == 1:
                    num_vial_windows = int(
                        6 / len(pump_map)
                    )  # no overhang vial window for pumps with same fluid types
                    uniform_pump_types = True

                # calculate number of active vial sets
                vial_window = []
                active_pumps = []

                # update vial window (set of vials in which dispense needles are physically above).
                # Vial window essentially behaves like a queue data structure, where vials are first in, first out as arm moves along snake dilution path
                for x in range(num_vial_windows):
                    if uniform_pump_types:
                        vial_window = current_vial_row[
                            x * len(pump_map) : x * len(pump_map) + len(pump_map)
                        ]
                        active_pumps = pump_map

                    if not uniform_pump_types:
                        if x < pump_num:
                            vial_window.append(current_vial_row[x])
                            active_pumps.append(pump_map[x])
                        if x >= pump_num:
                            vial_window.pop(0)
                            if x < len(current_vial_row):
                                vial_window.append(current_vial_row[x])
                            if x >= len(current_vial_row):
                                active_pumps.pop(0)

                    # check if this is the start of influx_snake_helper (vial_0)
                    initial_vial = False
                    if vial_window[0] == 0:
                        initial_vial = True

                    print_string = ""
                    for i in range(len(vial_window)):
                        print_string = print_string + "vial_{0} ".format(vial_window[i])
                    print_string = print_string + "in {0}".format(quad_name)

                    logger.info("current vial window is: %s" % vial_window)
                    logger.info(
                        "active pumps for current vial window is: %s" % active_pumps
                    )

                    # execute wash step for current vial_window
                    z = {"current": z_vial_dilution, "target": z_wash_station}
                    arm_settings = {
                        "current_coordinates": vial_dilution_coordinates,
                        "target_coordinates": wash_station_coordinates,
                        "transform_matrices": transform_matrices,
                        "z": z,
                        "initial_vial": initial_vial,
                        "post_wash": False,
                    }
                    wash_pump_instructions = {}
                    for pump in active_pumps:
                        wash_pump_instructions[pump] = 0

                    mapped_wash_pump_instructions = self.map_gcode_commands(
                        wash_pump_instructions
                    )
                    for octoprint_name in mapped_wash_pump_instructions:
                        self.octoprint_instances[octoprint_name].write_gcode(
                            "aspirate",
                            mapped_wash_pump_instructions[octoprint_name],
                            self.pump_conf,
                            self.robotics_status.prime_status["influx"],
                        )
                        self.octoprint_instances[octoprint_name].write_gcode(
                            "dispense",
                            mapped_wash_pump_instructions[octoprint_name],
                            self.pump_conf,
                            self.robotics_status.prime_status["influx"],
                        )
                    gcode_files = ["aspirate.gcode", "dispense.gcode"]

                    try:
                        await self.check_for_interrupt()  # hang here if pause is called or exit routine if exit command received
                    except ExitRobotics:
                        await session.close()
                        raise

                    try:
                        # influx routine resumed
                        self.robotics_status.mode = data["mode"]
                        await self.fluidic_event(
                            session,
                            gcode_files,
                            data["mode"],
                            arm_settings,
                            print_string + " (wash)",
                        )
                    except ExitRobotics:
                        await session.close()
                        raise
                    except Exception:
                        # emergency stop, error encountered and manual intervention required
                        await session.close()
                        raise HelperEventError(
                            "Error running wash fluidic event in influx_snake_helper - stopping robotics. Check logs for traceback"
                        )

                    # calculate pump volume (in steps) from syringe pump command and write gcode files to handle dilution events for current vial window
                    pump_instructions = {}
                    fractional_pump_instructions = {}
                    max_pump_counter = {}
                    self.robotics_status.vial_window = vial_window
                    self.robotics_status.active_pumps = active_pumps

                    # for each vial in the current vial window extract how many maximum syringe volumes and fractional volumes will be pumped
                    for i in range(len(vial_window)):
                        active_vial_name = "vial_{0}".format(vial_window[i])
                        active_pump = active_pumps[i]
                        pump_step_fraction = (
                            syringe_pump_commands[active_pump][quad_name][
                                active_vial_name
                            ]
                            / self.pump_conf["pumps"][active_pump]["max_steps"]
                        )
                        max = int(pump_step_fraction)
                        fractional_pump_instructions[active_pump] = int(
                            (pump_step_fraction - max)
                            * self.pump_conf["pumps"][active_pump]["max_steps"]
                        )
                        max_pump_counter[active_pump] = max

                    # move arm only during first pump event to position arm for current vial_window
                    arm_moved = False
                    while True:
                        # if desired volume is above maximum syringe volume, continously pump maximum syringe volumes, otherwise pump fractional syringe volume
                        for pump in active_pumps:
                            if max_pump_counter[pump] > 0:
                                pump_instructions[pump] = self.pump_conf["pumps"][pump][
                                    "max_steps"
                                ]
                                max_pump_counter[pump] = max_pump_counter[pump] - 1

                            if max_pump_counter[pump] == 0:
                                pump_instructions[pump] = fractional_pump_instructions[
                                    pump
                                ]
                                fractional_pump_instructions[pump] = "done"

                        mapped_pump_instructions = self.map_gcode_commands(
                            pump_instructions
                        )
                        for octoprint_name in mapped_wash_pump_instructions:
                            self.octoprint_instances[octoprint_name].write_gcode(
                                "aspirate",
                                mapped_pump_instructions[octoprint_name],
                                self.pump_conf,
                                self.robotics_status.prime_status["influx"],
                            )
                            self.octoprint_instances[octoprint_name].write_gcode(
                                "dispense",
                                mapped_pump_instructions[octoprint_name],
                                self.pump_conf,
                                self.robotics_status.prime_status["influx"],
                            )
                        arm_settings = {}
                        gcode_files = ["aspirate.gcode", "dispense.gcode"]

                        # calculate target coordinates for next vial_window
                        # if at end of row, next vial_window will be the next row
                        if not arm_moved:
                            if change_row:
                                vial_dilution_coordinates[1] = (
                                    vial_dilution_coordinates[1] - 18
                                )

                            # subtract if in middle row to move left, add if in first or third
                            else:
                                row_logic = 1
                                if vial_dilution_coordinates[1] == 18:
                                    row_logic = -1
                                vial_dilution_coordinates[0] = (
                                    vial_dilution_coordinates[0] + row_logic * 18
                                )

                            z = {"current": z_wash_station, "target": z_vial_dilution}
                            arm_settings = {
                                "current_coordinates": wash_station_coordinates,
                                "target_coordinates": vial_dilution_coordinates,
                                "transform_matrices": transform_matrices,
                                "z": z,
                                "initial_vial": False,
                                "post_wash": True,
                            }
                            arm_moved = True

                        try:
                            await self.check_for_interrupt()  # hang here if pause is called or exit routine if exit command received
                        except ExitRobotics:
                            await session.close()
                            raise

                        try:
                            # influx routine resumed
                            self.robotics_status.mode = data["mode"]
                            await self.fluidic_event(
                                session,
                                gcode_files,
                                data["mode"],
                                arm_settings,
                                print_string + " (influx)",
                            )
                        except ExitRobotics:
                            await session.close()
                            raise
                        except Exception:
                            # emergency stop, error encountered and manual intervention required
                            await session.close()
                            raise HelperEventError(
                                "Error running vial_set fluidic event in influx_snake_helper - stopping robotics. Check logs for traceback"
                            )

                        # check if volume for all vials in current vial window has been dispensed
                        # break loop if complete, otherwise continue pumping
                        if list(fractional_pump_instructions.values()).count(
                            "done"
                        ) >= len(list(fractional_pump_instructions.values())):
                            break

                    # finished dilutions for current vial_window, moving to next set of vials
                    change_row = False

                # change row
                change_row = True

            # reached end of dilution events for quad, move arm up before moving to next quad
            arm_coordinates_out = np.dot(
                transform_matrices[0],
                np.array(
                    [
                        [vial_dilution_coordinates[0]],
                        [vial_dilution_coordinates[1]],
                        [1],
                    ]
                ),
            )

            try:
                await self.check_for_interrupt()  # hang here if pause is called or exit routine if exit command received
            except ExitRobotics:
                await session.close()
                raise

            try:
                self.robotics_status.mode = data[
                    "mode"
                ]  # influx_routine resumed, change robotics_status mode
                await self.move_arm(
                    {
                        "x": arm_coordinates_out[0][0],
                        "y": arm_coordinates_out[1][0],
                        "z": z_vial_dilution[0],
                    }
                )
            except Exception:
                await session.close()
                raise HelperEventError(
                    "Error moving arm up after finishing snake influx path for %s - stopping robotics. Check logs for traceback"
                    % quad_name
                )

        self.robotics_status.vial_window = None
        self.robotics_status.active_quad = None
        await session.close()

    def rigid_transform(self, quad_coordinates, arm_coordinates):
        """Calculate the rigid transformation matrix between two sets of coordinates. Used to convert vial coordinates into xArm coordinates."""

        tform = ski.transform.EuclideanTransform()
        tform.estimate(quad_coordinates, arm_coordinates)
        return tform

    async def move_arm(self, coordinates):
        """Move xARM to specified coordinates."""
        x = coordinates["x"]
        y = coordinates["y"]
        z = coordinates["z"]

        self.update_conf()
        xarm_params = self.robotics_conf["xarm_params"]

        # move xARM to specified coordinates
        if self.robotics_status.xArm["arm_state"] == 4:
            raise RoboticsError("xArm in stop state, requires reset")
        else:
            result = self.arm.set_position(
                x=x,
                y=y,
                z=z,
                roll=xarm_params["roll"],
                pitch=xarm_params["pitch"],
                yaw=xarm_params["yaw"],
                speed=xarm_params["speed"],
                mvacc=xarm_params["mvacc"],
                wait=True,
            )
            if result < 0:
                raise RoboticsError(
                    "Unforseen failure detected when trying to move xARM during move_arm(), error code {0} given".format(
                        result
                    )
                )

    async def arm_path(self, arm_settings: dict):
        """Moves the xArm to a specified path using the given arm settings.

        Args:
            arm_settings (dict):
                initial_vial (bool): if set to True, xArm will begin path at phase 2 since it is already above the vials. False will start at phase 1.
                current_coordinates (list): [x,y] cooridantes of current arm coordinates based on smart quad plane
                target_coordinates (list): [x,y] cooridantes of target arm coordinates based on smart quad plane
                transform_matrices (list): numpy transformation matrix used to convert smart quad coordinates to xArm coordinates
                z (dict): [z1, z2] contains current and target z locations in xArm coordinates

        Raises:
            Exception: If there is an error while moving the arm.

        Returns:
            None"""

        x1 = None
        y1 = None
        x2 = None
        y2 = None
        x3 = None
        y3 = None
        next_coordinates_out = None
        next_coordinates_in = None

        # phase 1 coordinates (move arm up - z stays same)
        x1 = arm_settings["current_coordinates"][0]
        y1 = arm_settings["current_coordinates"][1]

        # phase 2 coordinates (move arm to horizontally to next vial window)
        x2 = arm_settings["target_coordinates"][0]
        y2 = arm_settings["target_coordinates"][1]

        # phase 3 coordinates (move arm down into next vial window)
        x3 = arm_settings["target_coordinates"][0]
        y3 = arm_settings["target_coordinates"][1]

        # set phase 1 coordinates equal to phase 2 to eliminate verticial movement if called at the begining of a smart quad sweep routine
        if arm_settings["initial_vial"]:
            x1 = x2
            y1 = y2

        next_coordinates_out = np.array([[x1, x2], [y1, y2], [1, 1]])
        next_coordinates_in = np.array([[x3], [y3], [1]])

        # transform vial coordinates into arm coordinates for each phase
        arm_coordinates_out = np.dot(
            arm_settings["transform_matrices"][0], next_coordinates_out
        )
        arm_coordinates_in = np.dot(
            arm_settings["transform_matrices"][1], next_coordinates_in
        )

        # move arm to next vial window using transformed vial coordinates
        try:
            if arm_settings["post_wash"]:
                await asyncio.sleep(
                    self.robotics_conf["wash_settings"]["submersed_time"]
                )
            await self.move_arm(
                {
                    "x": arm_coordinates_out[0][0],
                    "y": arm_coordinates_out[1][0],
                    "z": arm_settings["z"]["current"][0],
                }
            )
            await (
                self.check_for_interrupt()
            )  # hang here if pause is called or exit routine if exit command received
            # add delay to allow ethanol to dry from influx needle
            if arm_settings["post_wash"]:
                await asyncio.sleep(self.robotics_conf["wash_settings"]["dry_time"])
            await self.move_arm(
                {
                    "x": arm_coordinates_out[0][1],
                    "y": arm_coordinates_out[1][1],
                    "z": arm_settings["z"]["target"][0],
                }
            )
            await (
                self.check_for_interrupt()
            )  # hang here if pause is called or exit routine if exit command received
            await self.move_arm(
                {
                    "x": arm_coordinates_in[0][0],
                    "y": arm_coordinates_in[1][0],
                    "z": arm_settings["z"]["target"][1],
                }
            )

        except Exception:
            raise

    # Server event handlers. Must be registered using self.register_event_handlers() before usage
    async def broadcast(self):
        """Broadcasts the current robotics status to all connected clients. Also check if robotics server is connected to OctoPrint servers."""
        # check for potential overflow based on sensitivity threshold
        # overflow_trigger_map = {'left': [], 'right': []}
        # overflow_trigger_map['left'] =  [x * self.robotics_conf['overflow_voltage_step'] for x in self.robotics_conf['overflow_trigger_map']['left']]
        # overflow_trigger_map['right'] =  [x * self.robotics_conf['overflow_voltage_step'] for x in self.robotics_conf['overflow_trigger_map']['right']]
        # for quad_index in range(4):
        #    if (self.evolver_ns.overflow_data['left'][quad_index] > self.robotics_conf['overflow_voltage_threshold']) or (self.evolver_ns.overflow_data['right'][quad_index] > self.robotics_conf['overflow_voltage_threshold']):
        #        self.robotics_status.overflow_status['quads'][quad_index] = True
        #        self.stop_robotics()

        # identify the vial(s) that overflowed
        #        for index in range(18):
        #            if self.evolver_ns.overflow_data['right'][quad_index] >= overflow_trigger_map['right'][index] - self.robotics_conf['overflow_voltage_threshold'] or self.evolver_ns.overflow_data['right'][quad_index] <= overflow_trigger_map['right'][index] + self.robotics_conf['overflow_voltage_threshold']:
        #                for vial_index in range(index, index + 5):
        #                    if self.evolver_ns.overflow_data['left'][quad_index] >= overflow_trigger_map['left'][index] - self.robotics_conf['overflow_voltage_threshold'] or self.evolver_ns.overflow_data['left'][quad_index] <= overflow_trigger_map['left'][index] + self.robotics_conf['overflow_voltage_threshold']:
        #                        self.robotics_status.overflow_status['vial'] = vial_index

        # update connection statuses of OctoPrint instances
        session = aiohttp.ClientSession()
        check = await self.check_status(session)
        for octoprint_name in self.octoprint_instances:
            if not check["octoprint_statuses"][octoprint_name]:
                self.octoprint_instances[octoprint_name].connected = False
            if check["octoprint_statuses"][octoprint_name]:
                if "Offline" in check["octoprint_statuses"][octoprint_name]["state"]:
                    self.octoprint_instances[octoprint_name].connected = False
                else:
                    self.octoprint_instances[octoprint_name].connected = True
            self.robotics_status.OctoPrint[octoprint_name]["connection_status"] = (
                self.octoprint_instances[octoprint_name].connected
            )
        await session.close()

        # emit robotics status to all connected clients
        logging.info("robotics status broadcast %s" % asdict(self.robotics_status))
        await self.sio.emit(
            "broadcast", asdict(self.robotics_status), namespace="/robotics_evolver"
        )

    async def on_connect(self, sid, environ, auth):
        """Called when client connects to server."""
        logger.info("Client connected to robotics_eVOLVER server")

    async def on_disconnect(self, sid):
        """Called when client disconnects from server."""
        logger.info("Client disconnected to robotics_eVOLVER server")

    async def on_pause_robotics(self, sid, data):
        """Pause all robotics."""

        # if self.robotics_status.mode != 'exit' and self.robotics_status.mode != 'idle' and self.robotics_status.mode != 'emergency_stop':
        if self.robotics_status.mode in self.robotics_conf["modes"]["routines"]:
            self.robotics_status.mode = "pause"
            logger.info("Received pause request, pauing active routines.")

    async def on_resume_robotics(self, sid, data):
        """Resume all robotics."""

        if self.robotics_status.mode == "pause":
            self.robotics_status.mode = "resume"
            self.arm.set_state(0)
            logger.info("Resuming active routines.")

    async def on_exit_robotics(self, sid, data):
        """Exit the active robotics routine."""

        # if self.robotics_status.mode != 'idle' and self.robotics_status.mode != 'emergency_stop':
        if (
            self.robotics_status.mode in self.robotics_conf["modes"]["routines"]
            or self.robotics_status.mode == "pause"
        ):
            self.robotics_status.mode = "exit"
            logger.info("Received exit request, exiting active routines.")

    async def on_request_robotics_status(self, sid, data):
        """Request the current robotics status."""

        logger.info("Request for current robotics status received.")
        return {"type": "robotics", "data": asdict(self.robotics_status)}

    async def on_request_pump_conf(self, sid, data):
        """Request the current pump configuration."""

        logger.info("Request for pump settings received.")
        return {"type": "pump", "data": self.pump_conf}

    async def on_override_robotics_status(self, sid, data: dict):
        """Override self.robotics_status on the client.

        This method is called when a request is received to override `self.robotics_status` based on the provided `data` parameter.

        Args:
            sid (str): session ID.
            data (dict): contains new robotics status information."""

        logger.info("Received request to override robotics status")
        logger.info(data)
        # if data['mode'] in ['idle', 'dilution', 'pipetting', 'filling_vials', 'priming_influx', 'priming_efflux', 'pause', 'resume']:
        if "mode" in data:
            if (
                data["mode"] in self.robotics_conf["modes"]["routines"]
                or data["mode"] in self.robotics_conf["modes"]["states"]
            ):
                self.robotics_status.mode = data["mode"]

        if "prime_status" in data:
            if "influx" in data["prime_status"]:
                self.robotics_status.prime_status["influx"] = data["prime_status"][
                    "influx"
                ]
            if "efflux" in data["prime_status"]:
                self.robotics_status.prime_status["efflux"] = data["prime_status"][
                    "efflux"
                ]

        if "reset_xArm" in data:
            if not self.robotics_status.xArm.connected:
                self.arm.connect()
            self.reset_xArm()

    def on_stop_robotics(self, sid, data):
        """Emergency stop all robotics"""
        self.stop_robotics()

    async def on_reconnect_robotics(self, sid, data: dict):
        """Reconnect to OctoPrint servers and/or xArm.

        Args:
            sid (str): session ID.
            data (dict): contains the modules for reconnection. {'octoprint': True or False, 'xarm': True or False}"""

        if data["OctoPrint"]:
            for octoprint_name in self.octoprint_instances:
                self.octoprint_instances[octoprint_name].connect()
        if data["xArm"]:
            self.arm.connect()
        logger.info("Reconnecting to xArm and OctoPrint instances.")

    async def check_for_interrupt(self):
        """Called during routines to catch pause or exit signals from client. Hangs if pause is detected and/or raises ExitRobotics exception if exit is detected"""

        while self.robotics_status.mode == "pause":
            # pause detected, halt influx routine until pause is lifted or influx routine is stopped
            await self.sio.sleep(0.1)

        if (
            self.robotics_status.mode == "exit"
            or self.robotics_status.mode == "emergency_stop"
        ):
            raise ExitRobotics()

    async def on_pipette_routine(self, sid, data: dict):
        """Fill tubing lines with connected syringe pumps.

        Args:
            sid (str): session ID.
            data (dict): empty data strcuture to run the helper function. Currently not used"""

        start_time = time.time()  # get start time of routine to later calculate total elapsed time, useful for clients to gauge routine duration
        # execute routine using pipette_helper function
        if self.robotics_status.mode == "idle":
            try:
                logger.info("Pipetting liquids using syringe pumps")
                self.robotics_status.mode = "pipetting"
                await self.pipette_helper(data)
                end_time = time.time()
                self.robotics_status.mode = "idle"
                return {
                    "done": True,
                    "routine": "pipette",
                    "robotics_status": asdict(self.robotics_status),
                    "elapsed_time": end_time - start_time,
                    "message": "pipette_helper called successfully",
                }

            except HelperEventError as e:
                end_time = time.time()
                logger.exception(e)
                return {
                    "done": False,
                    "routine": "pipette",
                    "robotics_status": asdict(self.robotics_status),
                    "elapsed_time": end_time - start_time,
                    "message": "HelperEventError encountered, check HT_eVOLVER logs for traceback",
                }
            except ExitRobotics as e:
                end_time = time.time()
                logger.info("Exiting pipette routine")
                return {
                    "done": False,
                    "routine": "pipette",
                    "robotics_status": asdict(self.robotics_status),
                    "elapsed_time": end_time - start_time,
                    "message": "Exiting pipette routine",
                }

        else:
            return {"done": False, "message": "robotics_status mode is not idle"}

    async def on_fill_vials_routine(self, sid, data: dict):
        """Fill vials with media during experiment setup. Can use either robotic arm via influx_snake_helper() or efflux IPPs in reverse polarity via efflux_ipp_helper().

        Args:
            self: The reference to the current instance of the class.
            sid (str): The session ID.
            data (dict): contains parameters to run helper functions
                'commands' (dict): contains list of commands under 'syringe_pump_commands' and 'ipp_efflux_command' keys.
                'target_quads' (list): list of target smart quads e.g. ['quad_0', 'quad_1', 'quad_2', 'quad_3'].
                'wash' (bool): flag indicating whether to perform a wash step.
                'hardware' (str): hardware to use for routine, 'syringe_pumps' or 'ipp'"""

        start_time = time.time()  # get start time of influx routine to later calculate total elapsed time, useful for clients to gauge routine duration
        # execute prime_efflux_helper function
        if self.robotics_status.mode == "idle":
            try:
                logger.info("Filling vials with using the following command: %s" % data)
                self.robotics_status.mode = "filling_vials"
                helper_function = ""

                if data["hardware"] == "syringe_pumps":
                    helper_function = "influx_snake_helper"
                    data["mode"] = "filling_vials"
                    await self.influx_snake_helper(data)
                if data["hardware"] == "ipp":
                    helper_function = "efflux_ipp_helper"
                    ipp_commands = {}
                    for quad in data["target_quads"]:
                        ipp_commands[quad] = {"polarity": 1}
                    await self.efflux_ipp_helper(ipp_commands)

                self.robotics_status.mode = "idle"
                end_time = time.time()
                return {
                    "done": True,
                    "routine": "fill_vials",
                    "robotics_status": asdict(self.robotics_status),
                    "elapsed_time": end_time - start_time,
                    "message": "{0} successfully called".format(helper_function),
                }

            except HelperEventError as e:
                end_time = time.time()
                logger.exception(e)
                return {
                    "done": False,
                    "routine": "fill_vials",
                    "robotics_status": asdict(self.robotics_status),
                    "elapsed_time": end_time - start_time,
                    "message": "HelperEventError encountered, check HT_eVOLVER logs for traceback",
                }
            except ExitRobotics as e:
                end_time = time.time()
                logger.info("Exiting fill_vials routine")
                return {
                    "done": False,
                    "routine": "fill_vials",
                    "robotics_status": asdict(self.robotics_status),
                    "elapsed_time": end_time - start_time,
                    "message": "Exiting fill_vials routine",
                }
        else:
            return {
                "done": False,
                "routine": "fill_vials",
                "robotics_status": asdict(self.robotics_status),
                "message": "robotics_status mode not idle",
            }

    async def on_prime_influx_routine(self, sid, data: dict):
        """Prime syringe pumps. Tubes must be filled first.

        Args:
            sid (str): session ID.
            data (dict): empty data structure to run the helper function. Currently not used.

        Returns:
            dict: contains the result of the prime_influx routine."""

        start_time = time.time()  # get start time of influx routine to later calculate total elapsed time, useful for clients to gauge routine duration
        # execute routine using prime_influx_helper function
        if (
            self.robotics_status.mode == "idle"
            and not self.robotics_status.prime_status["influx"]
        ):
            try:
                logger.info("Priming syringe pumps")
                self.robotics_status.mode = "priming_influx"
                await self.prime_influx_helper()
                end_time = time.time()
                self.robotics_status.mode = "idle"
                self.robotics_status.prime_status["influx"] = True
                return {
                    "done": True,
                    "routine": "prime_influx",
                    "robotics_status": asdict(self.robotics_status),
                    "elapsed_time": end_time - start_time,
                    "message": "prime_influx_helper called successfully",
                }

            except HelperEventError as e:
                end_time = time.time()
                logger.exception(e)
                return {
                    "done": False,
                    "routine": "prime_influx",
                    "robotics_status": asdict(self.robotics_status),
                    "elapsed_time": end_time - start_time,
                    "message": "HelperEventError encountered, check HT_eVOLVER logs for traceback",
                }
        else:
            message = ""
            if self.robotics_status.mode != "idle":
                message += "robotics_status mode is not idle "
            if self.robotics_status.prime_status["influx"]:
                message += "influx syringe pumps are already primed"
            return {
                "done": False,
                "routine": "prime_influx",
                "robotics_status": asdict(self.robotics_status),
                "message": message,
            }

    async def on_prime_efflux_routine(self, sid, data):
        """Prime efflux board by reverse actuating efflux IPPs.

        Args:
            sid (str): session ID.
            data (dict): contains the parameters to run the helper function.

        Returns:
            dict: contains the result of the prime_efflux routine."""

        start_time = time.time()  # get start time of influx routine to later calculate total elapsed time, useful for clients to gauge routine duration
        # execute prime_efflux_helper function
        if self.robotics_status.mode == "idle":
            try:
                logger.info("Priming syringe pumps")
                self.robotics_status.mode = "priming_efflux"
                ipp_commands = {}
                for quad in data["target_quads"]:
                    ipp_commands[quad] = {"polarity": 1}
                await self.efflux_ipp_helper(ipp_commands)
                self.robotics_status.mode = "idle"
                end_time = time.time()
                return {
                    "done": True,
                    "routine": "prime_efflux",
                    "robotics_status": asdict(self.robotics_status),
                    "elapsed_time": end_time - start_time,
                    "message": "reverse_ipp_helper called successfully",
                }

            except HelperEventError as e:
                end_time = time.time()
                logger.exception(e)
                return {
                    "done": False,
                    "routine": "prime_efflux",
                    "robotics_status": asdict(self.robotics_status),
                    "elapsed_time": end_time - start_time,
                    "message": "HelperEventError encountered, check HT_eVOLVER logs for traceback",
                }
        else:
            return {
                "done": False,
                "routine": "prime_efflux",
                "robotics_status": asdict(self.robotics_status),
                "message": "robotics_status mode not idle",
            }

    async def on_dilution_routine(self, sid, data: dict):
        """Perform cooridinated syringe pump, xArm, and IPP hardware to dilute cultures for target vials.

        Args:
            sid (str): session ID.
            self: The reference to the current instance of the class.
            data (dict): The dilution commands to be executed.
                'commands' (dict): contains list of commands under 'syringe_pump_commands' and 'ipp_efflux_command' keys.
                'target_quads': (list) list of target smart quads for influx commands
                'mode': (str) mode of operation. Can be 'dilution' or 'fill_vials',
                'wash': (bool) flag indicating whether to perform a wash step.

        Returns:
            dict: contains the result of the dilution process.

        Raises:
            HelperEventError: If an error occurs during the influx routine process."""

        # get start time of influx routine to later calculate total elapsed time, useful for clients to gauge routine duration
        start_time = time.time()

        # execute influx routine using influx_snake_helper function
        if self.robotics_status.mode == "idle":
            try:
                logger.info("Executing the following influx routine command: %s" % data)
                self.robotics_status.mode = "dilution"
                data["mode"] = "dilution"
                await self.influx_snake_helper(data)
                await asyncio.sleep(
                    3
                )  # add delay to give time for culture mixing prior to efflux
                await self.efflux_ipp_helper(data["commands"]["ipp_efflux_command"])
                self.robotics_status.mode = "idle"
                end_time = time.time()
                return {
                    "done": True,
                    "routine": "influx",
                    "robotics_status": asdict(self.robotics_status),
                    "elapsed_time": end_time - start_time,
                    "message": "influx_snake_helper called successfully",
                }

            except HelperEventError as e:
                end_time = time.time()
                logger.exception(e)
                return {
                    "done": False,
                    "routine": "influx",
                    "robotics_status": asdict(self.robotics_status),
                    "elapsed_time": end_time - start_time,
                    "message": "HelperEventError encountered, check HT_eVOLVER logs for traceback",
                }
            except ExitRobotics as e:
                end_time = time.time()
                logger.info("Exiting influx routine")
                return {
                    "done": False,
                    "routine": "influx",
                    "robotics_status": asdict(self.robotics_status),
                    "elapsed_time": end_time - start_time,
                    "message": "Exiting influx routine",
                }
        else:
            return {
                "done": False,
                "routine": "influx",
                "robotics_status": asdict(self.robotics_status),
                "message": "robotics_status mode not idle",
            }

    def setup_event_handlers(self):
        """Attach event handlers to the server. Must be called before starting the server."""

        # connection handling
        self.sio.on("disconnect", self.on_disconnect, namespace="/robotics_evolver")
        self.sio.on("connect", self.on_connect, namespace="/robotics_evolver")

        # get information regarding robotics server
        self.sio.on(
            "request_robotics_status",
            self.on_request_robotics_status,
            namespace="/robotics_evolver",
        )
        self.sio.on(
            "request_pump_conf",
            self.on_request_pump_conf,
            namespace="/robotics_evolver",
        )

        # control state of robotics server and/or active routines
        self.sio.on(
            "override_robotics_status",
            self.on_override_robotics_status,
            namespace="/robotics_evolver",
        )
        self.sio.on(
            "stop_robotics", self.on_stop_robotics, namespace="/robotics_evolver"
        )
        self.sio.on(
            "reconnect_robotics",
            self.on_reconnect_robotics,
            namespace="/robotics_evolver",
        )
        self.sio.on(
            "pause_robotics", self.on_pause_robotics, namespace="/robotics_evolver"
        )
        self.sio.on(
            "resume_robotics", self.on_resume_robotics, namespace="/robotics_evolver"
        )
        self.sio.on(
            "exit_robotics", self.on_exit_robotics, namespace="/robotics_evolver"
        )

        # user defined robotics routines
        self.sio.on(
            "pipette_routine", self.on_pipette_routine, namespace="/robotics_evolver"
        )
        self.sio.on(
            "fill_vials_routine",
            self.on_fill_vials_routine,
            namespace="/robotics_evolver",
        )
        self.sio.on(
            "prime_influx_routine",
            self.on_prime_influx_routine,
            namespace="/robotics_evolver",
        )
        self.sio.on(
            "prime_efflux_routine",
            self.on_prime_efflux_routine,
            namespace="/robotics_evolver",
        )
        self.sio.on(
            "dilution_routine", self.on_dilution_routine, namespace="/robotics_evolver"
        )
