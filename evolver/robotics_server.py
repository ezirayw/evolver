import os
import numpy as np
import yaml
import aiohttp
import requests
import socketio
import asyncio
import shutil
import logging
import skimage as ski
from xarm.wrapper import XArmAPI

logger = logging.getLogger(__name__)

#### CLASS DEFINITIONS ####
# define custom Exception that catches robotics errors, logs them, and stops robotics
class RoboticsError(Exception):
    def __init__(self, message):
        self.message = message
        logger.error('RoboticsError Found: %s' % message)

class FluidicEventError(Exception):
    def __init__(self, message):
        self.message = message
        logger.error('FluidicEventError Found: %s' % message)

class OctoPrintError(Exception):
    def __init__(self, message):
        self.message = message
        logger.error('OctoPrintError Found: %s' % message)

class HelperEventError(Exception):
    def __init__(self, message):
        self.message = message
        logger.error('HelperEventError Found: %s' % message)

# define namespace events to communicate with eVOLVER server
class EvolverNamespace(socketio.ClientNamespace):
    def on_connect(self, *args):
        logger.info('robotics_eVOLVER connected to base_eVOLVER server')

    def on_disconnect(self, *args):
        logger.info('robotics_eVOLVER disconnected from base_eVOLVER server')

    def on_reconnect(self, *args):
        logger.info("robotics_eVOLVER reconnected to base_eVOLVER as server")

    def fluid_command(self, MESSAGE):
        logger.info('fluid command: %s', MESSAGE)
        command = {'param': 'pump', 'value': MESSAGE,
                    'recurring': False ,'immediate': True}
        self.emit('command', command, namespace='/dpu-evolver')

class RoboticsServer(EvolverNamespace):
    def __init__(self):

        # define class attributes
        self.sio = socketio.AsyncServer(always_connect=True)
        self.evolver_ns = None
        self.robotics_status = {
            'mode': 'idle',
            'active_quad': None,
            'active_pumps': None,
            'vial_window': None,
            'xArm': {
                'warning_code': 0,
                'error_code': 0,
                'arm_state': None
            },
            'primed': False
        }
        
        self.server_path = os.path.dirname(os.path.abspath(__file__))
        self.robotics_conf_path = os.path.join(self.server_path, 'robotics_server_conf.yml')

        with open(self.robotics_conf_path, 'r') as conf:
            self.robotics_conf = yaml.safe_load(conf)

        self.dilutions_path = os.path.join(self.server_path)
        self.xarm_ip = self.robotics_conf['xarm_ip']
        self.api_key = self.robotics_conf['octoprint_api_key']
        self.pump_conf = self.robotics_conf['pump_conf']
        self.octoprint_urls = {}
        for smoothie in self.pump_conf['smoothies']:
            url = "http://" + '192.168.1.15:' + str(self.pump_conf['smoothies'][smoothie]['port'])
            self.octoprint_urls[smoothie] = url
            gcode_dir_exist = os.path.exists(self.pump_conf['smoothies'][smoothie]['gcode_path'])
            if gcode_dir_exist:
                shutil.rmtree(self.pump_conf['smoothies'][smoothie]['gcode_path'])
            os.makedirs(self.pump_conf['smoothies'][smoothie]['gcode_path'])
        
        self.arm = XArmAPI(self.xarm_ip, enable_report=True)
        self.arm.clean_warn()
        self.arm.clean_error()
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_collision_sensitivity(2)
        self.arm.set_self_collision_detection(True)
        self.arm.set_state(state=0)
            
    def update_conf(self):
        """ Updates the robotics + pump configuration by loading the contents of the robotics_conf file.
        
        Reads the contents of the robotics_conf file specified by the `robotics_conf_path` attribute, and stores the parsed YAML data in the `robotics_conf` attribute. 
        Additionally, it updates the `pump_conf` attribute with the corresponding value from the `robotics_conf` dictionary. """
        
        with open(self.robotics_conf_path, 'r') as conf:
            self.robotics_conf = yaml.safe_load(conf)
        self.pump_conf = self.robotics_conf['pump_conf']
    
    def register_callback(self):
        """Register the error_warn_changed_callback and state_changed_callback for the xArm 5."""
        
        self.arm.register_error_warn_changed_callback(callback=self.error_warn_change_callback)
        self.arm.register_state_changed_callback(callback=self.state_changed_callback)

    def error_warn_change_callback(self, data):
        """ Update the error and warning codes in the robotics_status class attribute. """
        
        self.robotics_status['xArm']['error_code'] = data['error_code']
        self.robotics_status['xArm']['warn_code'] = data['warn_code']
        if data['error_code'] != 0:
            logger.error('xArm error_code %s encountered.' % data['error_code'])
            self.robotics_status['mode'] = 'emergency_stop'
        if data['warn_code'] != 0:
            logger.warn('xArm warning_code %s encountered.'.format(data['warn_code']))

    def state_changed_callback(self, data):
        """Update the arm state in the robotics_status class attribute based on the received data."""

        logger.debug(data)
        self.robotics_status['xArm']['arm_state'] = data['state']
        if data['state'] == 4:
            logger.error('xArm entered error state')
            self.robotics_status['mode'] = 'emergency_stop'
    
    def attach(self, app):
        """Attach the the robotics_server to the given Flask app."""
        
        self.sio.attach(app)
    
    def setup_client(self, socketIO_client):
        """Set up the robotics_server as an eVOLVER client by registering the EvolverNamespace."""
        
        self.evolver_ns = EvolverNamespace('/dpu-evolver')
        socketIO_client.register_namespace(self.evolver_ns)

    def stop_robotics(self):
        """Stop all robotics and syringe pump operations, including canceling current jobs and disconnecting from OctoPrint servers. Used in cases of emergencies or unforseen errors. Requires manual intervention to restart."""
        
        # emergency stop xArm and cancel current syringe job commands and disconnect from OctoPrint servers
        payload_1 = {'command': 'cancel'}
        payload_2 = {'command': 'disconnect'}
        for url in self.octoprint_urls:
            temp_url_1 = self.octoprint_urls[url] + '/api/job'
            temp_url_2 = self.octoprint_urls[url] + '/api/connection'
            header = {'X-Api-Key': self.api_key, 'Content-Type': 'application/json'}
            self.arm.emergency_stop()
            requests.post(temp_url_1, headers=header, params=payload_1)
            requests.post(temp_url_2, headers=header, params=payload_2)

        logger.info('stop_robotics() called - check logs to identify cause of error')
        self.robotics_status['mode'] = 'emergency_stop'

    def write_gcode(self, mode, instructions, gcode='', gcode_path=''):
        """ Writes G-code instructions to a file based on the given mode and instructions.

        Args:
            mode (str): The mode of operation. Can be 'aspirate', 'dispense', or 'prime_pumps'.
            instructions (dict): A dictionary containing the pump instructions.
            gcode (str, optional): The existing G-code string. Defaults to an empty string.
            gcode_path (str, optional): The path to the G-code file. Defaults to an empty string.

        Returns:
            None

        Raises:
            None
        """

        # use motor connection settings from config to map pump instructions to proper smoothieboards
        smoothie_instructions_map = {}
        for smoothie in self.pump_conf['smoothies']:
            smoothie_instructions = {}
            for pump_target in instructions:
                if pump_target in self.pump_conf['pumps'] and self.pump_conf['pumps'][pump_target]['smoothie'] == smoothie:
                    smoothie_instructions[pump_target] = instructions[pump_target]
            smoothie_instructions_map[smoothie] = smoothie_instructions

        # create gcode instructions for each pumps on each smoothieboard
        for smoothie in smoothie_instructions_map:
            plunger_in_commands = ['' , '']
            plunger_out_commands = ['' , '']
            prime_commands = ['' , '']
            valve_commands = {'on': ['' , ''] , 'off': ['', ''], 'steps': 0 }
            count = 0

            # mode specific modifications to gcode to account for valve actuation and/or priming
            if mode == 'aspirate':
                for pump in smoothie_instructions_map[smoothie]:
                    # get pump port settings
                    plunger = self.pump_conf['pumps'][pump]['motor_connections']['plunger']
                    plunger_in_commands[count] = '{0}{1}'.format(plunger, smoothie_instructions_map[smoothie][pump])
                    count = count + 1

                # combine gcode commands
                command = plunger_in_commands[0] + ' ' + plunger_in_commands[1]
                gcode = "G91\nG1 {0} F{1}\nM18".format(command, self.pump_conf['plunger_speed_in'])

            elif mode =='dispense':
                for pump in smoothie_instructions_map[smoothie]:
                    plunger = self.pump_conf['pumps'][pump]['motor_connections']['plunger']
                    valve = self.pump_conf['pumps'][pump]['motor_connections']['valve']
                    valve_steps_on = self.pump_conf['pumps'][pump]['motor_connections']['valve_steps']
                    valve_steps_off = self.pump_conf['pumps'][pump]['motor_connections']['valve_steps'] * -1

                    plunger_out_commands[count] = '{0}-{1}'.format(plunger, smoothie_instructions_map[smoothie][pump])
                    if self.robotics_status['primed'] and smoothie_instructions_map[smoothie][pump] != 0:
                        plunger_out_commands[count] = '{0}-{1}'.format(plunger, smoothie_instructions_map[smoothie][pump] + self.pump_conf['priming_steps'])
                        prime_commands[count] = '{0}{1}'.format(plunger, self.pump_conf['priming_steps'])
                    
                    if smoothie_instructions_map[smoothie][pump] == 0:
                        valve_commands['on'][count] = '{0}{1}'.format(valve, 0)
                        valve_commands['off'][count] = '{0}{1}'.format(valve, 0)    
                    valve_commands['on'][count] = '{0}{1}'.format(valve, valve_steps_on)
                    valve_commands['off'][count] = '{0}{1}'.format(valve, valve_steps_off)
                    count = count + 1

                # combine gcode commands
                valve_on = valve_commands['on'][0] + ' ' + valve_commands['on'][1]
                valve_off = valve_commands['off'][0] + ' ' + valve_commands['off'][1]
                plunger_out = plunger_out_commands[0] + ' ' + plunger_out_commands[1]
                prime = prime_commands[0] + ' ' + prime_commands[1]

                gcode = "G91\nG1 {0} F20000\nG4 P150\nG1 {1} F{2}\nG4 P150\nG1 {3} F20000\nM18".format(valve_on, plunger_out, self.pump_conf['plunger_speed_out'], valve_off)
                if self.robotics_status['primed']:
                    gcode = "G91\nG1 {0} F20000\nG4 P150\nG1 {1} F{2}\nG4 P150\nG1 {3} F15000\nG4 P150\nG1 {4} F20000\nM18".format(valve_on, plunger_out, self.pump_conf['plunger_speed_out'], prime, valve_off)

            elif mode == "prime_pumps":
                for pump in smoothie_instructions_map[smoothie]:
                    plunger = self.pump_conf['pumps'][pump]['motor_connections']['plunger']
                    valve = self.pump_conf['pumps'][pump]['motor_connections']['valve']
                    valve_steps_on = self.pump_conf['pumps'][pump]['motor_connections']['valve_steps']
                    valve_steps_off = self.pump_conf['pumps'][pump]['motor_connections']['valve_steps'] * -1

                    plunger_in_commands[count] = '{0}{1}'.format(plunger, smoothie_instructions_map[smoothie][pump])
                    valve_commands['on'][count] = '{0}{1}'.format(valve, valve_steps_on)
                    valve_commands['off'][count] = '{0}{1}'.format(valve, valve_steps_off)
                    count = count + 1

                # combine gcode commands
                valve_on = valve_commands['on'][0] + ' ' + valve_commands['on'][1]
                valve_off = valve_commands['off'][0] + ' ' + valve_commands['off'][1]

                gcode = 'G91\nG1 {0} F20000\nG4 P150\nG1 {1} F{2}\nG4 P150\nG1 {3} F20000\nM18'.format(valve_on, plunger_in_commands, self.pump_conf['plunger_speed_in'], valve_off)
            else:
                gcode = 'M18'

            # write command to gcode file
            filename = mode + '.gcode'
            gcode_path = os.path.join(self.pump_conf['smoothies'][smoothie]['gcode_path'], filename)
            f = open(gcode_path, 'w')
            f.write(gcode)
            f.close()

    async def post_gcode_async(self, session, gcode_path, smoothie):
        """ Sends a G-code file to the target smoothieboard using the OctoPrint API.

        Args:
            session (object): The session object used for making HTTP requests.
            gcode_path (str): The path to the G-code file to be sent.
            smoothie (str): The target smoothieboard to send the G-code file to.

        Raises:
            RoboticsError: If the function is called while in 'emergency_stop' mode.
            OctoPrintError: If there is an error while sending the G-code file. """
        
        payload = {'file': open(gcode_path, 'rb'), 'print':'true'}
        url = self.octoprint_urls[smoothie] + '/api/files/local'
        header={'X-Api-Key': self.api_key }
        
        if self.robotics_status['mode'] == 'emergency_stop':
            raise RoboticsError('cannot post gcode because in emergency_stop mode')
        else:  
            try:
                async with session.post(url, headers=header, data=payload) as response:
                    await response.json()
            except Exception as e:
                raise OctoPrintError('could not post gcode successfully to %s smoothie: %s' % (smoothie, e))

    async def check_job(self, session, smoothie):
        """ Check the status of target smoothieboard using the OctoPrint API. If an exception occurs during the HTTP request, the function sets the result to False.

        Args:
            session (aiohttp.ClientSession): The client session used to make HTTP requests.
            smoothie (str): The identifier for the robotics server.

        Returns:
            dict: A dictionary containing the result of the job status check. """
        
        url = self.octoprint_urls[smoothie] + '/api/job'
        header={'X-Api-Key': self.api_key }
        result = None
        try:
            async with session.get(url, headers=header) as response:
                result = await response.json()
        except Exception:
            result = False
        finally:
            return {'result': result}

    async def check_status(self, session, gcode_paths):
        """ Check the status of the OctoPrint servers that map to the given G-code files.

        Args:
            session (object): The session object for making HTTP requests.
            gcode_paths (list): A list of file paths for the G-code files.

        Returns:
            dict: A dictionary containing the result of the status check. The 'result' key
                    will be set to True if all G-code files are complete and the OctoPrint
                    server is operational. Otherwise, it will be set to False. """
        
        # map gcode file to cognate smoothie
        status_tasks = []
        status_complete = 0
        result = None

        for gcode_path in gcode_paths:
            gcode_parent_path = os.path.split(gcode_path)[0]
            for smoothie in self.pump_conf['smoothies']:
                if self.pump_conf['smoothies'][smoothie]['gcode_path'] == gcode_parent_path:
                    status_tasks.append(self.check_job(session, smoothie))
                    break

        # use check_job() to get OctoPrint server status for desired smoothieboard(s)
        try:
            status_results = await asyncio.gather(*status_tasks)
            for i in range(len(gcode_paths)):
                filename = os.path.split(gcode_paths[i])[1]
                if (status_results[i]['result']['progress']['completion'] >= 100) and (status_results[i]['result']['state'] == 'Operational') and (status_results[i]['result']['job']['file']['name'] == filename):
                    status_complete = status_complete + 1
            if status_complete == len(gcode_paths):
                result = True
            else:
                result = False
        except Exception as e:
            logger.debug('check status issue occured: %s', e)
            result = False
        finally:
            return {'result': result}

    async def fluidic_event(self, session, gcode_files, mode, arm_settings={}, print_string=''):
        """ Perform a fluidic event.

        This function runs a fluidic event, which consists of an aspiration phase and a dispense phase.
        The aspiration phase involves sending G-code commands to the syringe pumps to aspirate fluids. The dispense phase involves sending G-code commands to the syringe pumps to dispense fluids.

        Parameters:
        - session: The session object for the HTTP request.
        - gcode_files: A list of two strings representing the paths to the G-code files for aspiration and dispense.
        - mode: A string indicating the mode of the fluidic event.
        - arm_settings: A dictionary containing arm settings (optional). arm_settings{"current_coordinates": current_coordinates, "transform_matrices": transform_matrices, "z": z}
        - print_string: A string representing additional information about the fluidic event (optional).

        Raises:
        - FluidicEventError: If there is an error running the fluidic event.

        Returns: None """

        # create aspiration commands
        check_files = []
        await self.check_for_pause()
        self.robotics_status['mode'] = mode

        logger.info('running aspiration during %s for: %s' % (mode, print_string))
        async with asyncio.TaskGroup() as aspiration_tasks:
            for smoothie in range(self.pump_conf['smoothie_num']):
                gcode_path = os.path.join(self.pump_conf['smoothies'][smoothie]['gcode_path'], gcode_files[0])
                check_files.append(gcode_path)
                aspiration_tasks.create_task(self.post_gcode_async(session, gcode_path, smoothie))

            if arm_settings:
                aspiration_tasks.create_task(self.arm_path(arm_settings))

        # verify that syringe_pumps are ready to receive dispense commands
        while True:
            check = await self.check_status(session, check_files)
            if check['result']:
                break
            else:
                await asyncio.sleep(0.5)

        # create dispense commands
        check_files = []
        await self.check_for_pause()
        self.robotics_status['mode'] = mode

        logger.info('running dispense during %s for: %s' % (mode, print_string))
        async with asyncio.TaskGroup() as dispense_tasks:
            for smoothie in range(self.pump_conf['smoothie_num']):
                gcode_path = os.path.join(self.pump_conf['smoothies'][smoothie]['gcode_path'], gcode_files[1])
                dispense_tasks.create_task(self.post_gcode_async(session, gcode_path, smoothie))
                check_files.append(gcode_path)
        
        # verify that syringe pumps are ready to receive future commands
        while True:
            check = await self.check_status(session, check_files)
            if check['result']:
                break
            else:
                await asyncio.sleep(0.5)
        logger.info('finished %s for: %s' % (mode, print_string))

    async def prime_pumps_helper(self):
        """ Helper method to prime the fluid pumps.

        This method primes the fluid pumps by sending priming instructions to the syringe pumps. After priming, robotics_status is updated
    
        Returns:
            A dictionary with the result of the priming operation. If the priming is successful,
            the dictionary will contain {'result': {'done': True}}. If there is an error during
            the priming process, the dictionary will contain {'result': {'done': False}}. """

        if self.robotics_status['mode'] == 'idle' and not self.robotics_status['primed']:

            self.robotics_status['mode'] = 'priming'
            # start asyncio Client Session
            session = aiohttp.ClientSession()
        
            # get fluid pumps from server_conf and write prime pumps instructions
            instructions = {}
            print_string = ''

            # load in current config
            self.update_conf()

            for pump in self.pump_conf['pumps']:
                print_string = print_string + '{0} pump'.format(pump)
                instructions[pump] = self.pump_conf['priming_steps']
            self.write_gcode('prime_pumps', instructions)

            # create syringe_pump priming commands
            try:
                prime_pumps_tasks = [None] * self.pump_conf['smoothie_num']
                check_files = [None] * self.pump_conf['smoothie_num']
                prime_pumps_results = []
                while True:
                    for smoothie in range(self.pump_conf['smoothie_num']):
                        gcode_path = os.path.join(self.pump_conf['smoothies'][smoothie]['gcode_path'], 'prime_pumps.gcode')
                        prime_pumps_tasks[smoothie] = self.post_gcode_async(session, gcode_path, smoothie)
                        check_files[smoothie] = gcode_path

                    prime_pumps_results = await asyncio.gather(*prime_pumps_tasks)
                    if any([ ('done', False) in dict['result'].items() for dict in prime_pumps_results ]):
                        continue
                    else:
                        break
            
            except Exception as e:
                await session.close()
                logger.error('error priming pumps: %s' % e)
                logger.exception(e)
                return {'result':{'done': False}}

            else:
                # verify that syringe pumps executed priming commands
                commands_complete = 0
                for dict in prime_pumps_results:
                    if dict['result']['done']:
                        commands_complete = commands_complete + 1

                # verify status of syringe_pumps to receive future commands
                if commands_complete == len(prime_pumps_tasks):
                    self.robotics_status['primed'] = True
                    while True:
                        check = await self.check_status(session, check_files)
                        if check['result']:
                            break
                        else:
                            await asyncio.sleep(0.5)
                    await session.close()
                    self.robotics_status['mode'] = 'idle'
                    logger.info('syringe pumps successfully primed for future dispensing')
                    return {'result':{'done': True}}
                else:
                    await session.close()
                    self.robotics_status['mode'] = 'idle'
                    logger.error('error priming syringe pumps, commands not completed')
                    return {'result':{'done': False}}

    async def fill_tubing_helper(self):
        """ Helper method to fill tubing using syringe pumps.

        Returns:
            dict: {'result': {'done': True}} if the filling is successful, {'result': {'done': False}} otherwise. """
        
        # check status before executing robotic function
        if self.robotics_status['mode'] == 'idle':
            
            self.robotics_status['mode'] = 'fill_tubing'
            # start asyncio Client Session
            session = aiohttp.ClientSession()

            # get fluid pumps from server_conf
            instructions = {}
            print_string = ''

            # load in current config
            self.update_conf()

            for pump in self.pump_conf['pumps']:
                print_string = print_string + '{0} pump '.format(pump)
                instructions[pump] = 350
            self.write_gcode('aspirate', instructions)
            self.write_gcode('dispense', instructions)
            gcode_files = ['aspirate.gcode', 'dispense.gcode']

            print_string = 'filling syringe pump tubing'

            await self.fluidic_event(session, gcode_files, 'fill_tubing', print_string=print_string)
            
            self.robotics_status['mode'] = 'idle'
            await session.close()
            return {'result':{'done': True}}

 
    async def influx_snake_helper(self, data):
        """Helper function for executing sequential fluidic_event(s) in a snake-like pattern across Smart Quads. Use for dilution and vial setup events.

        Args:
            data (dict): A dictionary containing the following keys:
                - 'message' (dict): A dictionary containing fluidic commands for syringe pump and ipp efflux.
                - 'active_quads' (list): A list of active quadrants.
                - 'mode' (str): The mode of operation ('dilution' or 'vial_setup').
                - 'wash' (bool): A flag indicating whether to perform a wash step.

        Returns:
            None

        Raises:
            HelperEventError: Raised in the event a FluidicEvent Exception is caught to coordinate experiment management. """

        if self.robotics_status['mode'] == 'idle' or self.robotics_status['mode'] == 'pause':
            syringe_pump_commands = data['message']['syringe_pump_message']
            ipp_effux_command = data['message']['ipp_efflux_message']
            
            # update status (either dilution or vial_setup)
            self.robotics_status['mode'] = data['mode']
            
            # start asyncio Client Session
            session = aiohttp.ClientSession()

            # get calibrated smart quad coordinates from config
            self.update_conf()
            
            self.pump_conf = self.robotics_conf['pump_conf']
            vial_map = [[0,1,2,3,4,5], [11,10,9,8,7,6], [12,13,14,15,16,17]]

            # loop through sets of vials (vial_window) and execute fluid dispension events
            for quad_name in data['active_quads']:
                self.robotics_status['active_quad'] = quad_name
                vial_dilution_coordinates = [-18,36]
                wash_station_coordinates = [0,-24]
                change_row = False

                # calculate euclidean transformation matrix to convert vial_coordinates into arm coordinates using homing calibration
                # home based on first two and last two vials
                vial_0_out = np.array([self.robotics_conf['homing_coordinates'][quad_name]['vial_0']['x_out'], self.robotics_conf['homing_coordinates'][quad_name]['vial_0']['y']])
                vial_0_in = np.array([self.robotics_conf['homing_coordinates'][quad_name]['vial_0']['x_in'], self.robotics_conf['homing_coordinates'][quad_name]['vial_0']['y']])

                vial_17_out = np.array([self.robotics_conf['homing_coordinates'][quad_name]['vial_17']['x_out'], self.robotics_conf['homing_coordinates'][quad_name]['vial_17']['y']])
                vial_17_in = np.array([self.robotics_conf['homing_coordinates'][quad_name]['vial_17']['x_in'], self.robotics_conf['homing_coordinates'][quad_name]['vial_17']['y']])            
                
                z_vial_dilution = np.array([self.robotics_conf['homing_coordinates'][quad_name]['vial_0']['z_out'], self.robotics_conf['homing_coordinates'][quad_name]['vial_0']['z_in']])
                z_wash_station = np.array([self.robotics_conf['homing_coordinates'][quad_name]['wash_station']['z_out'], self.robotics_conf['homing_coordinates'][quad_name]['wash_station']['z_in']])
            
                vial_coordinates = np.array([[0, 36], [90, 0]])
                calibrated_coordinates_out = np.array([vial_0_out, vial_17_out])
                calibrated_coordinates_in = np.array([vial_0_in, vial_17_in])
            
                transform_matrix_out = self.rigid_transform(vial_coordinates, calibrated_coordinates_out)
                transform_matrix_in = self.rigid_transform(vial_coordinates, calibrated_coordinates_in)
                transform_matrices = np.stack((transform_matrix_out, transform_matrix_in))

                for row_num in range(np.size(vial_map, 0)):
                    current_vial_row = vial_map[row_num]

                    # get list of pumps from server_conf.yml
                    pump_map = []
                    for pump_id in range(self.pump_conf['pump_num']):
                        for pump in self.pump_conf['pumps']:
                            if self.pump_conf['pumps'][pump]['id'] == pump_id:
                                pump_map.append(pump)
                                break

                    # flip pump map to account for leading pump position changing in middle row of quad
                    if row_num == 1:
                        pump_map.reverse()

                    # number of active vial sets
                    num_vial_sets = 6 + (self.pump_conf['pump_num'] - 1)
                    vial_window = []
                    active_pumps = []

                    # update vial window (set of vials in which dispense needles are physically above). 
                    # Vial window essentially behaves like a queue data structure, where vials are first in, first out as arm moves along snake dilution path
                    for x in range(num_vial_sets):
                        if x < self.pump_conf['pump_num']:
                            vial_window.append(current_vial_row[x])
                            active_pumps.append(pump_map[x])
                        if x >= self.pump_conf['pump_num']:
                            vial_window.pop(0)
                            if x < len(current_vial_row):
                                vial_window.append(current_vial_row[x])
                            if x >= len(current_vial_row):
                                active_pumps.pop(0)
                        
                        print_string = ''
                        for i in range(len(vial_window)):
                            print_string = print_string + 'vial_{0} '.format(vial_window[i])
                        print_string = print_string + 'in {0}'.format(quad_name)

                        logger.debug('current vial window is: %s', vial_window)
                        logger.debug('active pumps for current vial window is: %s', active_pumps)

                        # execute wash step for current vial_window
                        if data['wash']:
                            z = {'current': z_vial_dilution, 'target': z_wash_station}        
                            arm_settings = {"current_coordinates": vial_dilution_coordinates, "target_coordinates": wash_station_coordinates, "transform_matrices": transform_matrices, "z": z}
                            wash_pump_instructions = {}
                            for pump in self.pump_conf['pumps']:
                                wash_pump_instructions[pump] = 0
                            self.write_gcode('aspirate', wash_pump_instructions)
                            self.write_gcode('dispense', wash_pump_instructions)
                            gcode_files = ['aspirate.gcode', 'dispense.gcode']
                            
                            await self.check_for_pause()
                            try:
                                # influx routine resumed
                                self.robotics_status['mode'] = data['mode']
                                await self.fluidic_event(session, gcode_files, 'wash', arm_settings, print_string)
                            except Exception:
                                # emergency stop, error encountered and manual intervention required
                                await session.close()
                                self.stop_robotics()
                                raise HelperEventError('error running wash fluidic event in influx_snake_helper, check logs for traceback')

                        # extract pump volume (in steps) from syringe pump command and write gcode files to handle dilution events for current vial window
                        fractional_pump_instructions = {}
                        max_pump_instructions = {}
                        max_pump_num = {}
                        
                        self.robotics_status['vial_window'] = vial_window
                        self.robotics_status['active_pumps'] = active_pumps
                        # for each vial in the current vial window extract how many maximum syringe volumes and fractional volumes will be pumped
                        for i in range(len(vial_window)):
                            active_vial_name = 'vial_{0}'.format(vial_window[i])
                            active_pump = active_pumps[i]
                            pump_step_fraction = syringe_pump_commands[active_pump][quad_name][active_vial_name] / self.pump_conf["pumps"][active_pump]["max_steps"]
                            max = int(pump_step_fraction)
                            fractional_pump_instructions[active_pump] = int((pump_step_fraction - max) * self.pump_conf["pumps"][active_pump]["max_steps"])
                            max_pump_num[active_pump] = max

                        # move arm only during first pump event to position arm for current vial_window
                        arm_moved = False
                        while True:
                            # if desired volume is above maximum syringe volume, continously pump maximum syringe volumes, otherwise pump fractional syringe volume
                            for pump in active_pumps:
                                if max_pump_num[pump] > 0:
                                    max_pump_instructions[pump] = self.pump_conf["pumps"][pump]["max_steps"]
                                    self.write_gcode('aspirate', max_pump_instructions)
                                    self.write_gcode('dispense', max_pump_instructions)
                                    max_pump_num[pump] = max_pump_num[pump] - 1

                                if max_pump_num[pump] == 0:
                                    self.write_gcode('aspirate', fractional_pump_instructions)
                                    self.write_gcode('dispense', fractional_pump_instructions)
                                    fractional_pump_instructions[pump] = 'done'
                                                
                            # calculate target coordinates for next vial_window
                            # if at end of row, next vial_window will be the next row 
                            arm_settings = {}
                            gcode_files = ['aspirate.gcode', 'dispense.gcode']                           

                            if not arm_moved:
                                if change_row:
                                    vial_dilution_coordinates[1] = vial_dilution_coordinates[1] - 18

                                # subtract if in middle row to move left, add if in first or third
                                else:
                                    row_logic = 1
                                    if vial_dilution_coordinates[1] == 18:
                                        row_logic = -1
                                    vial_dilution_coordinates[0] = vial_dilution_coordinates[0] + row_logic*18

                                z = {'current': z_wash_station, 'target': z_vial_dilution}        
                                arm_settings = {"current_coordinates": wash_station_coordinates, "target_coordinates":vial_dilution_coordinates, "transform_matrices": transform_matrices, "z": z, "wash_dry_delay": 15}
                                arm_moved = True

                            await self.check_for_pause()
                            try:
                                # influx routine resumed
                                self.robotics_status['mode'] = data['mode']
                                await self.fluidic_event(session, gcode_files, 'influx', arm_settings, print_string)
                            
                            except Exception:
                                # emergency stop, error encountered and manual intervention required
                                await session.close()
                                self.stop_robotics()
                                raise HelperEventError('error running vial_set fluidic event in influx_snake_helper, check logs for traceback')

                            
                            # check if volume for all vials in current vial window has been dispensing
                            # break loop if complete, otherwise continue pumping
                            if list(fractional_pump_instructions.values()).count('done') >= len(list(fractional_pump_instructions.values())):
                                break
                        
                        # finished dilutions for current vial_window, moving to next set of vials
                        change_row = False

                    # change row
                    change_row = True

                # reached end of dilution events for quad, move arm up before moving to next quad
                arm_coordinates_out = np.dot(transform_matrices[0], np.array([[vial_dilution_coordinates[0]], [vial_dilution_coordinates[1]], [1]]))
                await self.check_for_pause()      
                try:
                    # influx routine resumed
                    self.robotics_status['mode'] = data['mode']
                    await self.move_arm({'x': arm_coordinates_out[0][0], 'y': arm_coordinates_out[1][0], 'z': z_vial_dilution[0]}, True)
                except Exception:
                    await session.close()
                    self.stop_robotics()
                    raise HelperEventError('error moving arm to up after finishing quad influx_snake function, check logs')
                
                # send efflux command to continue IPPs for x amount of time if influx mode is set to dilution
                if self.robotics_status['mode'] == 'dilution':
                    self.evolver_ns.fluid_command(ipp_effux_command)
                    await asyncio.sleep(0.5)
                    # send secondary command in case first not registered
                    self.evolver_ns.fluid_command(ipp_effux_command) 

            self.robotics_status['mode'] = 'idle'
            self.robotics_status['vial_window'] = None
            self.robotics_status['active_quad'] = None
            
            await session.close()
            return {'done': True}

    def rigid_transform(self, quad_coordinates, arm_coordinates):
        """ Calculate the rigid transformation matrix between two sets of coordinates. Used to convert vial coordinates into arm coordinates. """

        tform = ski.transform.EuclideanTransform()
        tform.estimate(quad_coordinates, arm_coordinates)
        return tform

    async def move_arm(self, coordinates, wait):
        """ Move xARM to specified coordinates."""
        x = coordinates['x']
        y = coordinates['y']
        z = coordinates['z']

        self.update_conf()
        xarm_params = self.robotics_conf['xarm_params']

        # move xARM to specified coordinates
        if self.robotics_status['mode'] == 'emergency_stop' or self.robotics_status['xArm']['arm_state'] == 4:
            raise RoboticsError('cannot move arm due to incompatible mode or arm state')
        else:
            result = self.arm.set_position(x=x, y=y, z=z, roll=xarm_params['roll'], pitch=xarm_params['pitch'], yaw=xarm_params['yaw'], speed=xarm_params['speed'], mvacc=xarm_params['mvacc'], wait=wait)
            if result < 0:
                raise RoboticsError('failure moving xARM during move_arm(), error code {0} given'.format(result))
            return result

    async def arm_path(self, arm_settings):
        """ Moves the xArm to a specified path using the given arm settings.

        Args:
            arm_settings (dict): A dictionary containing the arm settings.
                It should include the following keys:
                - 'current_coordinates': A list of current arm coordinates [x, y].
                - 'target_coordinates': A list of target arm coordinates [x, y].
                - 'transform_matrices': A list of transformation matrices for each phase.
                - 'z': A dictionary containing the current and target z coordinates.

        Raises:
            Exception: If there is an error while moving the arm.

        Returns:
            None """

        # phase 1 coordinates (move arm up - z stays same)
        x1 = arm_settings['current_coordinates'][0]
        y1 = arm_settings['current_coordinates'][1]

        # phase 2 & 3 coordinates (move arm to next vial window)
        x2 = arm_settings['target_coordinates'][0]
        x3 = arm_settings['target_coordinates'][0]

        y2 = arm_settings['target_coordinates'][1]
        y3 = arm_settings['target_coordinates'][1]

        next_coordinates_out = np.array([[x1, x2], [y1, y2], [1,1]])
        next_coordinates_in = np.array([[x3], [y3], [1]])

        # transform vial coordinates into arm coordinates for each phase
        arm_coordinates_out = np.dot(arm_settings['transform_matrices'][0], next_coordinates_out)
        arm_coordinates_in = np.dot(arm_settings['transform_matrices'][1], next_coordinates_in)

        # move arm to next vial window using transformed vial coordinates
        try:
            if 'wash_dry_delay' in arm_settings:
                await asyncio.sleep(15)
            await self.move_arm({'x': arm_coordinates_out[0][0], 'y': arm_coordinates_out[1][0], 'z': arm_settings['z']['current'][0]}, True)
            # add delay to allow ethanol to dry from influx needle
            if 'wash_dry_delay' in arm_settings:
                await asyncio.sleep(arm_settings['wash_dry_delay'])
            await self.move_arm({'x': arm_coordinates_out[0][1], 'y': arm_coordinates_out[1][1], 'z': arm_settings['z']['target'][0]}, True)
            await self.move_arm({'x': arm_coordinates_in[0][0], 'y': arm_coordinates_in[1][0], 'z': arm_settings['z']['target'][1]}, True)
        
        except Exception as e:
            raise e


    # Server event handlers. Must be registered using self.register_event_handlers() before usage
    async def broadcast(self):
        """ Broadcasts the current robotics status to all connected clients."""
        
        logging.info('Robotics status broadcast %s' % self.robotics_status)
        await self.sio.emit('broadcast', self.robotics_status, namespace = '/robotics')

    async def on_connect(self, sid, environ, auth):
        """ Called when client connects to server."""

        logger.info('dpu connected to robotics_eVOLVER server')

    async def on_disconnect(self, sid):
        """ Called when client disconnects from server."""
        
        logger.info('dpu disconnected to robotics_eVOLVER server')

    async def on_pause_robotics(self, sid, data):
        """ Pause all robotics. """

        self.robotics_status['mode'] = 'pause'        
        logger.info('Received pause request, pauing active routines.')

    async def on_resume_robotics(self, sid, data):
        """ Resume all robotics. """

        self.robotics_status['mode'] = 'idle'
        logger.info('Resuming active routines.')
    
    async def on_get_pump_conf(self, sid, data):
        """ Request pump configuration. """
        
        data = {'data': self.pump_conf}
        logger.info("Retreiving pump settings...")
        await self.sio.emit('active_pump_conf', data, namespace = '/robotics')
    
    async def on_fill_tubing(self, sid, data):
        """ Fill tubing lines with connected syringe pumps. """
        
        await self.fill_tubing_helper()
        logger.info('Filling syringe pump tubing')

    async def on_prime_pump(self, sid, data):
        """ Prime syringe pumps. Tubes must be filled first. """
        
        self.sio.start_background_task(self.prime_pumps_helper)
        logger.info('Priming syringe pumps')

    async def on_influx_routine(self, sid, data):
        """  Perform dilutions and media setup for target vials. 
        
        This method executes the dilution commands specified in the `data` parameter.
        It logs the commands being executed and returns the result of the dilution process.
        
        Args:
            sid (str): The session ID.
            self: The reference to the current instance of the class.
            data: The dilution commands to be executed.
        
        Returns:
            dict: A dictionary containing the result of the dilution process.
        
        Raises:
            HelperEventError: If an error occurs during the influx routine process. """
        
        try:
            logger.info('Executing the following influx routine command: %s', data)
            await self.influx_snake_helper(data)
            return {'done': True, 'robotics_status': self.robotics_status}
        
        except HelperEventError as e:
            result = {'done': False, 'robotics_status': self.robotics_status, 'message': 'HelperEventError encountered, check HT_eVOLVER logs for traceback'}
            logger.exception(e)
            self.stop_robotics()
            return result
        
    async def on_request_robotics_status(self, sid, data):
        """ Request the current robotics status. """

        logger.info('Retreiving active robotics status...')
        await self.sio.emit('active_robotics_status', self.robotics_status, namespace = '/robotics')

    async def on_request_robotics_status(self, sid, data):
        """ Request the current robotics status. """

        logger.info('Retreiving active robotics status...')
        await self.sio.emit('active_robotics_status', self.robotics_status, namespace = '/robotics')

    async def on_override_robotics_status(self, sid, data):
        """ Override self.robotics_status on the client.
        
        This method is called when a request is received to override `self.robotics_status` based on the provided `data` parameter.
        
        Parameters:
            sid (str): The session ID.
            data (dict): The data containing the new robotics status information. """
        
        logger.info('Received request to override robotics status')
        logger.info(data)
        if data['mode'] in ['idle', 'fill_tubing', 'priming', 'influx', 'pause', 'resume']:
            self.robotics_status['mode'] = data['mode']
        if 'primed' in data:
            self.robotics_status['primed'] = data['primed']
        if data['reset_arm']:
            self.arm.clean_warn()
            self.arm.clean_error()
            self.arm.set_state(state=0)
        logger.info('Overriding robotics status.')
    
    async def on_stop_robotics(self, sid, data):
        """ Emergency stop all robotics """
        
        self.stop_robotics()

    async def check_for_pause(self):
        while self.robotics_status['mode'] == 'pause':
            # pause detected, halt influx routine until pause is lifted or influx routine is stopped
            await self.sio.sleep(0.1)

    def setup_event_handlers(self):
        """ Attach event handlers to the server. Must be called before starting the server."""
        self.sio.on('disconnect', self.on_disconnect, namespace='/robotics')
        self.sio.on('connect', self.on_connect, namespace='/robotics')
        self.sio.on('get_pump_conf', self.on_get_pump_conf, namespace='/robotics')
        self.sio.on('fill_tubing', self.on_fill_tubing, namespace='/robotics')
        self.sio.on('prime_pump', self.on_prime_pump, namespace='/robotics')
        self.sio.on('influx_routine', self.on_influx_routine, namespace='/robotics')
        self.sio.on('request_robotics_status', self.on_request_robotics_status, namespace='/robotics')
        self.sio.on('override_robotics_status', self.on_override_robotics_status, namespace='/robotics')
        self.sio.on('stop_robotics', self.on_stop_robotics, namespace='/robotics')
        self.sio.on('pause_robotics', self.on_pause_robotics, namespace='/robotics')
        self.sio.on('resume_robotics', self.on_resume_robotics, namespace='/robotics')