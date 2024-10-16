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
        logger.info('fluid command: %s' % MESSAGE)
        command = {'param': 'pump', 'value': MESSAGE,
                    'recurring': False ,'immediate': True}
        self.emit('command', command, namespace='/dpu-evolver')

class OctoPrintInterface():
    def __init__(self, evolver_ip, api_key, octoprint_conf, name, post_gcode_timeout):
        self.evolver_ip = evolver_ip
        self.api_key = api_key
        self.gcode_folder = octoprint_conf['gcode_folder']
        self.base_url = 'http://' + self.evolver_ip + ':' + str(octoprint_conf['port'])
        self.id = octoprint_conf['octoprint_id']
        self.name = name
        self.post_gcode_timeout = post_gcode_timeout
        self.broadcast_count = 0
        self.connected = None

    def connect(self):
        """Connect OctoPrint instance to pumps on smoothieboard using OctoPrint API.
        
        Returns:
            dict: A dictionary containing the result of the connect command."""
        
        # send POST request to OctoPrint server
        payload = {'command': 'connect'}
        header = {'X-Api-Key': self.api_key}
        temp_url = self.base_url + '/api/connection'

        try:
            response = requests.post(temp_url, headers=header, json=payload)
            if response.status_code == 204:
                logger.debug('connection to pumps for %s successful' % self.name)
                self.connected = True
            if response.status_code == 400:
                logger.warn('cannot connect to pumps on %s' % self.name)
        except Exception as e:
            raise OctoPrintError('could not connect to pumps on %s due to error: %s' % (self.name, e))
    
    def disconnect(self):
        """ Disconnect OctoPrint instance from pumps on smoothieboard using OctoPrint API.
        
        Returns:
            dict: A dictionary containing the result of the disconnect command. """
        
        # send POST request to OctoPrint server
        payload = {'command': 'disconnect'}
        header = {'X-Api-Key': self.api_key}
        temp_url = self.base_url + '/api/connection'

        try:
            response = requests.post(temp_url, headers=header, json=payload)
            if response.status_code == 204:
                logger.debug('disconnect from pumps on %s successful' % self.name)
                self.connected = False
            if response.status_code == 400:
                logger.warn('cannot disconnect from pumps on %s' % self.name)   
        except Exception as e:
            raise OctoPrintError('could not disconnect to pumps on %s due to error: %s' % (self.name, e))

    def cancel(self):
        """Cancel any current pump jobs on smoothieboard using OctoPrint API.
        
        Returns:
            dict: A dictionary containing the result of the cancel command. """
        
        # send POST request to OctoPrint server
        payload = {'command': 'cancel'}
        header = {'X-Api-Key': self.api_key}
        temp_url = self.base_url + '/api/job'

        try:
            response = requests.post(temp_url, headers=header, json=payload)
            if response.status_code == 204:
                logger.debug('cancellation of active pump jobs on %s successful' % self.name)
            if response.status_code == 409:
                logger.warn('no active pump jobs found to cancel on %s' % self.name)
        except Exception as e:
            raise OctoPrintError('could not cancel pump jobs on %s due to error: %s' % (self.name, e))
        
    async def get_status(self, session):
        """ Check status of OctoPrint instance, including any active jobs, using the OctoPrint API.

        Args:
            session (aiohttp.ClientSession): The client session used to make HTTP requests.

        Returns:
            dict: A dictionary containing the result of the job status check. """
        
        url = self.base_url + '/api/job'
        header={'X-Api-Key': self.api_key }
        try:
            async with session.get(url, headers=header) as response:
                return await response.json()
        except Exception as e:
            raise OctoPrintError('could not check job successfully for %s: %s' % (self.name, e))

    def write_gcode(self, mode, pump_instructions, pump_conf, primed_status, gcode=''):
        """ Writes G-code instructions to a file based on the given mode and instructions.

        Args:
            mode (str): The mode of operation. Can be 'aspirate', 'dispense', or 'prime_pumps'.
            instructions (dict): A dictionary containing the pump instructions.

        Returns:
            None

        Raises:
            None
        """

        plunger_in_commands = ['' , '']
        plunger_out_commands = ['' , '']
        prime_commands = ['' , '']
        valve_commands = {'on': ['' , ''] , 'off': ['', ''], 'steps': 0 }
        count = 0
        gcode = ''

        # mode specific modifications to gcode to account for valve actuation and/or priming
        if mode == 'aspirate':
            for pump in pump_instructions:
                # get pump port settings
                plunger = pump_conf['pumps'][pump]['motor_connections']['plunger']
                plunger_in_commands[count] = '{0}{1}'.format(plunger, pump_instructions[pump])
                count = count + 1

            # combine gcode commands
            #command = plunger_in_commands[0] + ' ' + plunger_in_commands[1]
            command = plunger_in_commands[0]
            gcode = "G91\nG1 {0} F{1}\nM18".format(command, pump_conf['plunger_speed_in'])

        elif mode =='dispense':
            for pump in pump_instructions:
                plunger = pump_conf['pumps'][pump]['motor_connections']['plunger']
                valve = pump_conf['pumps'][pump]['motor_connections']['valve']
                valve_steps_on = pump_conf['pumps'][pump]['motor_connections']['valve_steps']
                valve_steps_off = pump_conf['pumps'][pump]['motor_connections']['valve_steps'] * -1

                plunger_out_commands[count] = '{0}-{1}'.format(plunger, pump_instructions[pump])
                if primed_status and pump_instructions[pump] != 0:
                    plunger_out_commands[count] = '{0}-{1}'.format(plunger, pump_instructions[pump] + pump_conf['priming_steps'])
                    prime_commands[count] = '{0}{1}'.format(plunger, pump_conf['priming_steps'])
                
                if pump_instructions[pump] == 0:
                    valve_commands['on'][count] = '{0}{1}'.format(valve, 0)
                    valve_commands['off'][count] = '{0}{1}'.format(valve, 0)    
                valve_commands['on'][count] = '{0}{1}'.format(valve, valve_steps_on)
                valve_commands['off'][count] = '{0}{1}'.format(valve, valve_steps_off)
                count = count + 1

            # combine gcode commands
            #valve_on = valve_commands['on'][0] + ' ' + valve_commands['on'][1]
            valve_on = valve_commands['on'][0]
            #valve_off = valve_commands['off'][0] + ' ' + valve_commands['off'][1]
            valve_off = valve_commands['off'][0]
            #plunger_out = plunger_out_commands[0] + ' ' + plunger_out_commands[1]
            plunger_out = plunger_out_commands[0]
            #prime = prime_commands[0] + ' ' + prime_commands[1]
            prime = prime_commands[0]
            
            gcode = "G91\nG1 {0} F20000\nG4 P100\nG1 {1} F{2}\nG4 P100\nG1 {3} F20000\nM18".format(valve_on, plunger_out, pump_conf['plunger_speed_out'], valve_off)
            if primed_status and pump_instructions[pump] != 0:
                gcode = "G91\nG1 {0} F20000\nG4 P100\nG1 {1} F{2}\nG4 P100\nG1 {3} F15000\nG4 P100\nG1 {4} F20000\nM18".format(valve_on, plunger_out, pump_conf['plunger_speed_out'], prime, valve_off)

        elif mode == "prime_pumps":
            for pump in pump_instructions:
                plunger = pump_conf['pumps'][pump]['motor_connections']['plunger']
                valve = pump_conf['pumps'][pump]['motor_connections']['valve']
                valve_steps_on = pump_conf['pumps'][pump]['motor_connections']['valve_steps']
                valve_steps_off = pump_conf['pumps'][pump]['motor_connections']['valve_steps'] * -1

                plunger_in_commands[count] = '{0}{1}'.format(plunger, pump_instructions[pump])
                valve_commands['on'][count] = '{0}{1}'.format(valve, valve_steps_on)
                valve_commands['off'][count] = '{0}{1}'.format(valve, valve_steps_off)
                count = count + 1

            # combine gcode commands
            valve_on = valve_commands['on'][0] + ' ' + valve_commands['on'][1]
            valve_off = valve_commands['off'][0] + ' ' + valve_commands['off'][1]
            gcode = 'G91\nG1 {0} F20000\nG4 P100\nG1 {1} F{2}\nG4 P100\nG1 {3} F20000\nM18'.format(valve_on, plunger_in_commands, pump_conf['plunger_speed_in'], valve_off)
        
        else:
            gcode = 'M18'

        # write command to gcode file
        filename = mode + '.gcode'
        gcode_path = os.path.join(self.gcode_folder, filename)
        f = open(gcode_path, 'w')
        f.write(gcode)
        f.close()

    async def post_gcode_async(self, session, gcode_path):
        """ Upload a G-code file and execute pump job to OctoPrint instance using OctoPrint API.

        Args:
            session (object): The session object used for making HTTP requests.
            gcode_path (str): The path to the G-code file to be sent.

        Raises:
            RoboticsError: If the function is called while in 'emergency_stop' mode.
            OctoPrintError: If there is an error while sending the G-code file. """
    
        url = self.base_url + '/api/files/local'
        header={'X-Api-Key': self.api_key }
        request_attempts = 0

        while True:
            try:
                with open(gcode_path, 'rb') as f:
                    # send POST request to OctoPrint server
                    payload = {'file': f, 'print':'true'}
                    async with session.post(url, headers=header, data=payload) as response:
                        result = await response.json()
                        logger.info(result)
                        if result['done']:
                            break
            
            except KeyError as e:
                logger.warn('could not recognize successful gcode POST event to %s, trying again: %s' % (self.name, e))
                request_attempts = request_attempts + 1
                if request_attempts > self.post_gcode_timeout:
                    raise OctoPrintError('could not post gcode to %s after %s attempts' % (self.name, self.robotics_conf['post_request_timeout']))
                await asyncio.sleep(1) # wait 1 second before trying again

            except Exception as e:
                raise OctoPrintError('unforseen error trying to POST gcode')

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
                'arm_state': None,
                'connected': None,
            },
            'OctoPrint': {},
            'prime_status': {
                'influx': False,
                'efflux': False
            }
        }
        
        # setup Robotics server instance
        self.server_path = os.path.dirname(os.path.abspath(__file__))
        self.robotics_conf_path = os.path.join(self.server_path, 'robotics_server_conf.yml')
        with open(self.robotics_conf_path, 'r') as conf:
            self.robotics_conf = yaml.safe_load(conf)
        self.pump_conf = self.robotics_conf['pump_conf']
        self.dilutions_path = os.path.join(self.server_path)
        self.xarm_ip = self.robotics_conf['xarm_ip']
        self.api_key = self.robotics_conf['octoprint_api_key']
        self.octoprint_instances = {}
        for octoprint_name in self.robotics_conf['octoprint_instances']:
            self.octoprint_instances[octoprint_name] = OctoPrintInterface(self.robotics_conf['evolver_ip'], self.api_key, self.robotics_conf['octoprint_instances'][octoprint_name], octoprint_name, self.robotics_conf['post_request_timeout'])
            self.robotics_status['OctoPrint'].update({octoprint_name: {'connection_status': self.octoprint_instances[octoprint_name].connected}})
            gcode_dir_exist = os.path.exists(self.robotics_conf['octoprint_instances'][octoprint_name]['gcode_folder'])
            if gcode_dir_exist:
                shutil.rmtree(self.robotics_conf['octoprint_instances'][octoprint_name]['gcode_folder'])
            os.makedirs(self.robotics_conf['octoprint_instances'][octoprint_name]['gcode_folder'])
        
        # setup xArm
        self.arm = XArmAPI(self.xarm_ip, enable_report=True)
        self.arm.clean_warn()
        self.arm.clean_error()
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_collision_sensitivity(2)
        self.arm.set_self_collision_detection(True)
        self.arm.set_state(state=0)
        self.robotics_status['xArm']['connected'] = self.arm.connected
        logger.info('robotics server initialized')
            
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
        self.arm.register_connect_changed_callback(callback=self.connect_changed_callback)

    def error_warn_change_callback(self, data):
        """ Update the error and warning codes in the robotics_status class attribute. """
        
        self.robotics_status['xArm']['error_code'] = data['error_code']
        self.robotics_status['xArm']['warn_code'] = data['warn_code']
        if data['error_code'] != 0:
            logger.error('xArm error_code %s encountered.' % data['error_code'])
            self.stop_robotics()
        if data['warn_code'] != 0:
            logger.warn('xArm warning_code %s encountered.' % data['warn_code'])

    def state_changed_callback(self, data):
        """Update the arm state in the robotics_status class attribute based on the received data."""

        logger.debug(data)
        self.robotics_status['xArm']['arm_state'] = data['state']
        if data['state'] == 4:
            logger.error('xArm entered error state')
            self.stop_robotics()

    def connect_changed_callback(self, data):
        """Update the arm connection status in the robotics_status class attribute based on the received data."""
        
        logger.info('xArm connection status changed to %s' % data['connected'])
        self.robotics_status['xArm']['connected'] = data['connected']
    
    def attach(self, app):
        """Attach the the robotics_server to the given Flask app."""
        
        self.sio.attach(app)
    
    def setup_client(self, socketIO_client):
        """Set up the robotics_server as an eVOLVER client by registering the EvolverNamespace."""
        
        self.evolver_ns = EvolverNamespace('/dpu-evolver')
        socketIO_client.register_namespace(self.evolver_ns)

    def stop_robotics(self):
        """Stop all robotics and syringe pump operations, including canceling current jobs and disconnecting from OctoPrint servers. Used in cases of emergencies or unforseen errors. Requires manual intervention to restart."""        

        self.robotics_status['mode'] = 'emergency_stop'
        logger.error('stop_robotics() called - check logs to identify cause of error')

        # emergency stop xArm, cancel current syringe job commands, and disconnect from OctoPrint servers
        try:
            for octoprint_name in self.octoprint_instances:
                self.octoprint_instances[octoprint_name].cancel()
                self.octoprint_instances[octoprint_name].disconnect()
            self.arm.disconnect()
        except OctoPrintError as e:
            logger.exception(e)
    
    def map_gcode_commands(self, instructions):
        """ Maps pump instructions to cogante OctoPrint instances.

        Args:
            mode (str): The mode of operation. Can be 'aspirate', 'dispense', or 'prime_pumps'.
            instructions (dict): A dictionary containing pump instructions. Key is pump target, value is volume to pump in G-code.
        Returns:
            octoprint_instructioins_map (dict): A nested dictionary containing pump instructions mapped to cognate OctoPrint instance.
            format is {octoprint_0: {pump_target: volume, pump_target: volume}, octoprint_1: {pump_target: volume, pump_target: volume}, ...} """

        # use pump_config to map pump instructions to cognate OctoPrint instances
        # should look like following: {octoprint_0: {pump_target: volume, pump_target: volume}, octoprint_1: {pump_target: volume, pump_target: volume}, ...}
        octoprint_instructions_map = {}
        for octoprint_name in self.octoprint_instances:
            octoprint_instructions = {}
            for pump_target in instructions:
                if pump_target in self.pump_conf['pumps'] and self.pump_conf['pumps'][pump_target]['octoprint_id'] == self.octoprint_instances[octoprint_name].id:
                    octoprint_instructions[pump_target] = instructions[pump_target]
            octoprint_instructions_map[octoprint_name] = octoprint_instructions
        
        return octoprint_instructions_map

    async def check_status(self, session, gcode_paths=[]):
        """ Check the status of the OctoPrint servers that map to the given G-code files.

        Args:
            session (object): The session object for making HTTP requests.
            gcode_paths (list): A list of file paths for the G-code files.

        Returns:
            result (dict): A dictionary containing the states of the OctoPrint servers and the completion status of pump jobs."""
        
        status_tasks = []
        all_jobs_complete = 0
        result = {'accept_new_jobs': False, 'octoprint_statuses': {}}

        # create get_status() tasks for each OctoPrint instance
        for octoprint_name in self.octoprint_instances:
            status_tasks.append(self.octoprint_instances[octoprint_name].get_status(session))
            result['octoprint_statuses'][octoprint_name] = {}

        try:
            status_results = await asyncio.gather(*status_tasks)
            
            # parse through status results to check OctoPrint state and pump job completion, if gcode paths are given
            for octoprint_name in self.octoprint_instances:
                index = self.octoprint_instances[octoprint_name].id
                logger.debug('check status results')
                logger.debug(status_results[index])
                result['octoprint_statuses'][octoprint_name] = status_results[index]

                if gcode_paths:
                    filename = os.path.split(gcode_paths[index])[1]
                    if (status_results[index]['progress']['completion'] >= 100) and (status_results[index]['state'] == 'Operational') and (status_results[index]['job']['file']['name'] == filename):
                        all_jobs_complete += 1

            if all_jobs_complete == len(gcode_paths):
                result['accept_new_jobs'] = True

        except Exception as e:
            logger.warn('check status issue occured: %s' % e)
        finally:
            return result

    async def fluidic_event(self, session, gcode_files, mode, arm_settings={}, print_string=''):
        """ Perform a fluidic event.

        This function runs a fluidic event, which consists of an aspiration phase and a dispense phase.
        The aspiration phase involves sending G-code commands to the syringe pumps to aspirate fluids. The dispense phase involves sending G-code commands to the syringe pumps to dispense fluids.

        Parameters:
        - session (object): The session object for the HTTP request.
        - gcode_files (list): A list of two strings representing the paths to the G-code files for aspiration and dispense.
        - mode (string): A string indicating the mode of the fluidic event.
        - arm_settings (dict): A dictionary containing arm settings (optional). arm_settings{"current_coordinates": current_coordinates, "transform_matrices": transform_matrices, "z": z}
        - print_string (string): A string representing additional information about the fluidic event (optional).

        Raises:
        - FluidicEventError: If there is an error running the fluidic event.

        Returns: None """

        await self.check_for_pause() # hang here if pause is called before starting aspiration commands
        self.robotics_status['mode'] = mode

        # create aspiration commands
        logger.info('running aspiration during %s for: %s' % (mode, print_string))
        check_files = []
        async with asyncio.TaskGroup() as aspiration_tasks:
            for octoprint_name in self.octoprint_instances:
                gcode_path = os.path.join(self.octoprint_instances[octoprint_name].gcode_folder, gcode_files[0])
                check_files.append(gcode_path)
                aspiration_tasks.create_task(self.octoprint_instances[octoprint_name].post_gcode_async(session, gcode_path))

            if arm_settings:
                aspiration_tasks.create_task(self.arm_path(arm_settings))

        # verify that syringe_pumps are ready to receive dispense commands
        check_attempts = 0
        while True:
            check = await self.check_status(session, check_files)
            logger.debug('checking status of aspiration commands')
            logger.debug(check)
            if check['accept_new_jobs']:
                break
            else:
                if check_attempts > self.robotics_conf['check_status_timout']:
                    raise FluidicEventError('check attempts for aspiration fluidic event exceeded max attempts, check OctoPrint server logs')
                #for octoprint_name in self.octoprint_instances:
                    #if check['octoprint_statuses'][octoprint_name]['state'] == 'Offline after error':
                        # attempt to reconnect to OctoPrint instance if connection is lost
                        #logger.warn('reconnecting to %s due to offline error during check_status' % octoprint_name)
                        #self.octoprint_instances[octoprint_name].disconnect()
                        #self.octoprint_instances[octoprint_name].connect()
                check_attempts = check_attempts + 1
                await asyncio.sleep(1)

        await self.check_for_pause() # hang here if pause is called
        self.robotics_status['mode'] = mode

        # create dispense commands
        logger.info('running dispense during %s for: %s' % (mode, print_string))
        check_files = []
        async with asyncio.TaskGroup() as dispense_tasks:
            for octoprint_name in self.octoprint_instances:
                gcode_path = os.path.join(self.octoprint_instances[octoprint_name].gcode_folder, gcode_files[1])
                dispense_tasks.create_task(self.octoprint_instances[octoprint_name].post_gcode_async(session, gcode_path))
                check_files.append(gcode_path)
        
        # verify that syringe pumps are ready to receive future commands
        check_attempts = 0
        while True:
            check = await self.check_status(session, check_files)
            logger.info('checking status of dispense commands')
            logger.info(check)
            if check['accept_new_jobs']:
                break
            else:
                if check_attempts > self.robotics_conf['check_status_timout']:
                    raise FluidicEventError('check attempts for dispense fluidic event exceeded max attempts, check OctoPrint server logs')
                #for octoprint_name in self.octoprint_instances:
                    #if check['octoprint_statuses'][octoprint_name]['state'] == 'Offline after error':
                        # attempt to reconnect to OctoPrint instance if connection is lost
                        #logger.warn('reconnecting to %s due to offline error during check_status' % octoprint_name)
                        #self.octoprint_instances[octoprint_name].disconnect()
                        #self.octoprint_instances[octoprint_name].connect()
                check_attempts = check_attempts + 1
                await asyncio.sleep(1)
        
        logger.info('finished %s for: %s' % (mode, print_string))

    async def prime_influx_helper(self, data):
        """ Helper method to prime the fluid pumps.

        This method primes the fluid pumps by sending priming instructions to the syringe pumps. After priming, robotics_status is updated
    
        Returns:
            result (dict): A dictionary with the result of the priming operation. """

        if self.robotics_status['mode'] == 'idle' and not self.robotics_status['prime_status']['influx']:

            # update robotics status
            self.robotics_status['mode'] = 'priming influx'
            
            # start asyncio Client Session
            session = aiohttp.ClientSession()

            # load in current config
            self.update_conf()

            # get fluid pumps from server_conf and write prime pumps instructions
            pump_instructions = {}
            print_string = ''

            for pump in self.pump_conf['pumps']:
                print_string = print_string + '{0} pump'.format(pump)
                pump_instructions[pump] = self.pump_conf['priming_steps']
            
            mapped_pump_instructions = self.map_gcode_commands(pump_instructions)
            for octoprint_name in mapped_pump_instructions:
                self.octoprint_instances[octoprint_name].write_gcode('prime_pumps', mapped_pump_instructions[octoprint_name], self.pump_conf, self.robotics_status['prime_status']['influx'])

            # create syringe_pump priming commands
            try:
                prime_pumps_tasks = []
                check_files = []

                async with asyncio.TaskGroup() as prime_pumps_tasks:
                    for octoprint_name in self.octoprint_instances:
                        gcode_path = os.path.join(self.octoprint_instances[octoprint_name].gcode_folder,'prime_pumps.gcode')
                        prime_pumps_tasks.create_task(self.octoprint_instances[octoprint_name].post_gcode_async(session, gcode_path))
                        check_files.append(gcode_path)
                
                # verify that syringe pumps are ready to receive future commands
                check_attempts = 0
                while True:
                    check = await self.check_status(session, check_files)
                    logger.debug('checking status of OctoPrint commands for prime_influx_helper')
                    logger.debug(check)
                    if check['accept_new_jobs']:
                        break
                    else:
                        if check_attempts > self.robotics_conf['check_status_timout']:
                            raise FluidicEventError('check attempts for dispense fluidic event exceeded max attempts, check OctoPrint server logs')
                        check_attempts = check_attempts + 1
                        await asyncio.sleep(1)
                
                # update robotics status upon successful priming                    
                self.robotics_status['mode'] = 'idle'
                self.robotics_status['prime_status']['influx'] = True
                logger.info('syringe pumps successfully primed')
                await session.close()
                return {'done': True, 'message': 'syringe pumps successfully primed'}
            
            except Exception:
                await session.close()
                self.stop_robotics()
                raise HelperEventError('error running prime_influx_helper, check logs for traceback')
        else:
            if self.robotics_status['mode'] != 'idle':
                return {'done': False, 'message': 'robotics_status mode is not idle'}
            if self.robotics_status['prime_status']['influx']:
                return {'done': False, 'message': 'influx syringe pumps are already primed'}

    async def prime_efflux_helper(self, data):
        """ Helper method to prime efflux board.

        This method primes efflux IPPs on efflux board by pumps by sending IPP actuation commands to eVOLVER. After priming, robotics_status is updated
    
        Returns:
            result (dict): A dictionary with the result of the priming operation. """
        
        if self.robotics_status['mode'] == 'idle' and not self.robotics_status['prime_status']['efflux']:

            # update robotics status
            self.robotics_status['mode'] = 'priming efflux'

            # start asyncio Client Session
            session = aiohttp.ClientSession()

            # load in current config
            self.update_conf()

            # build ipp efflux command
            ipp_effux_command = ['--'] * 48 # empty command
            ipp_index = 1
            ipp_number = 1
            ipp_addresses = self.pump_conf['efflux_priming_settings']['ipp_addresses']
            for quad in data['active_quads']:
                for ipp_address in ipp_addresses[quad]:
                    ipp_effux_command[ipp_address] = '{0}|{1}|{2}|{3}'.format(self.pump_conf['efflux_priming_settings']['frequency'], ipp_number, ipp_index, self.pump_conf['efflux_priming_settings']['ipp_time'])
                    ipp_index = ipp_index + 1
                        # setup efflux variables for next quad calculations
                ipp_number += 1

            # send efflux command
            self.evolver_ns.fluid_command(ipp_effux_command)
    
    async def fill_tubing_helper(self):
        """ Helper method to fill tubing using syringe pumps.

        Returns:
            result (dict): A dictionary with the result of the fill_tubing operation """
        
        # check status before executing robotic function
        if self.robotics_status['mode'] == 'idle':
            
            # update robotics status
            self.robotics_status['mode'] = 'fill_tubing'
            
            # start asyncio Client Session
            session = aiohttp.ClientSession()

            # load in current config
            self.update_conf()

            # build gcode instructions for OctoPrint instances
            fill_tubing_instructions = {}
            print_string = 'filling syringe pump tubing'

            for pump in self.pump_conf['pumps']:
                print_string = print_string + ' {0} pump '.format(pump)
                fill_tubing_instructions[pump] = 350
            mapped_fill_tubing_instructions = self.map_gcode_commands(fill_tubing_instructions)
            
            for octoprint_name in mapped_fill_tubing_instructions:
                self.octoprint_instances[octoprint_name].write_gcode('aspirate', mapped_fill_tubing_instructions[octoprint_name], self.pump_conf, self.robotics_status['prime_status']['influx'])
                self.octoprint_instances[octoprint_name].write_gcode('dispense', mapped_fill_tubing_instructions[octoprint_name], self.pump_conf, self.robotics_status['prime_status']['influx'])
            gcode_files = ['aspirate.gcode', 'dispense.gcode']

            try:
                await self.fluidic_event(session, gcode_files, 'fill_tubing', print_string=print_string)
                await session.close()
                self.robotics_status['mode'] = 'idle'
                return {'done': True, 'message': 'syringe pumps successfully filled'}
            except Exception:
                await session.close()
                self.stop_robotics()
                raise HelperEventError('error running fill_tubing_helper, check logs for traceback')

        else:
            return {'done': False, 'message': 'robotics_status mode is not idle'}
 
    async def influx_snake_helper(self, data):
        """Helper function for executing sequential fluidic_event(s) in a snake-like pattern across Smart Quads. Use for dilution and vial setup events.

        Args:
            data (dict): A dictionary containing the following keys:
                - 'command' (dict): A dictionary containing fluidic commands for syringe pump and ipp efflux.
                - 'active_quads' (list): A list of active quadrants.
                - 'mode' (str): The mode of operation ('dilution' or 'vial_setup').
                - 'wash' (bool): A flag indicating whether to perform a wash step.

        Raises:
            HelperEventError: Raised in the event a FluidicEvent Exception is caught to coordinate experiment management. """

        if self.robotics_status['mode'] == 'idle' or self.robotics_status['mode'] == 'pause':
            syringe_pump_commands = data['command']['syringe_pump_command']
            ipp_effux_command = data['command']['ipp_efflux_command']
            
            # update status (either dilution or vial_setup)
            self.robotics_status['mode'] = data['mode']

            # start asyncio Client Session
            session = aiohttp.ClientSession()
            
            # load in current config
            self.update_conf()

            # loop through sets of vials (vial_window) and execute fluid dispension events
            vial_map = [[0,1,2,3,4,5], [11,10,9,8,7,6], [12,13,14,15,16,17]]
            for quad_name in data['active_quads']:
                self.robotics_status['active_quad'] = quad_name
                vial_dilution_coordinates = [-18,36]
                wash_station_coordinates = [0,-24]
                change_row = False

                # calculate euclidean transformation matrix to convert vial_coordinates into arm coordinates using homing calibration
                # home based on vial_0 and vial_17 for each smart quad
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

                    # get list of pumps from config
                    pump_map = []
                    pump_types = []
                    for pump_id in range(self.pump_conf['pump_num']):
                        for pump in self.pump_conf['pumps']:
                            if self.pump_conf['pumps'][pump]['id'] == pump_id:
                                pump_map.append(pump)
                                pump_types.append(self.pump_conf['pumps'][pump]['type'])
                                break
                    
                    # flip pump map to account for leading pump position changing in middle row of quad
                    if row_num == 1:
                        pump_map.reverse()

                    # check if pumps have the same fluid type to configure number of vial windows and arm path
                    num_vial_windows = None
                    uniform_pump_types = None
                    if len(set(pump_types)) > 1:
                        num_vial_windows = 6 + (self.pump_conf['pump_num'] - 1) # overhang vial window for pumps with different fluid types
                        uniform_pump_types = True
                    if len(set(pump_types)) == 1:
                        num_vial_windows = 6 / len(pump_map) # no overhang vial window for pumps with same fluid types
                        uniform_pump_types = False

                    # calculate number of active vial sets
                    num_vial_windows = 6 + (self.pump_conf['pump_num'] - 1)
                    vial_window = []
                    active_pumps = []

                    # update vial window (set of vials in which dispense needles are physically above). 
                    # Vial window essentially behaves like a queue data structure, where vials are first in, first out as arm moves along snake dilution path
                    for x in range(num_vial_windows):
                        if not uniform_pump_types: 
                            vial_window = current_vial_row[num_vial_windows*x : num_vial_windows*x + num_vial_windows + 1]
                            active_pumps = pump_map

                        if uniform_pump_types:
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

                        logger.info('current vial window is: %s' % vial_window)
                        logger.info('active pumps for current vial window is: %s' % active_pumps)

                        # execute wash step for current vial_window
                        if data['wash']:
                            z = {'current': z_vial_dilution, 'target': z_wash_station}        
                            arm_settings = {"current_coordinates": vial_dilution_coordinates, "target_coordinates": wash_station_coordinates, "transform_matrices": transform_matrices, "z": z}
                            wash_pump_instructions = {}
                            for pump in active_pumps:
                                wash_pump_instructions[pump] = 0

                            mapped_wash_pump_instructions = self.map_gcode_commands(wash_pump_instructions)
                            for octoprint_name in mapped_wash_pump_instructions:
                                self.octoprint_instances[octoprint_name].write_gcode('aspirate', mapped_wash_pump_instructions[octoprint_name], self.pump_conf, self.robotics_status['prime_status']['influx'])
                                self.octoprint_instances[octoprint_name].write_gcode('dispense', mapped_wash_pump_instructions[octoprint_name], self.pump_conf, self.robotics_status['prime_status']['influx'])
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
                        pump_instructions = {}
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
                                    pump_instructions[pump] = self.pump_conf["pumps"][pump]["max_steps"]
                                    max_pump_num[pump] = max_pump_num[pump] - 1

                                if max_pump_num[pump] == 0:
                                    pump_instructions[pump] = fractional_pump_instructions[pump]
                                    fractional_pump_instructions[pump] = 'done'
                            
                            mapped_pump_instructions = self.map_gcode_commands(pump_instructions)
                            for octoprint_name in mapped_wash_pump_instructions:
                                self.octoprint_instances[octoprint_name].write_gcode('aspirate', mapped_pump_instructions[octoprint_name], self.pump_conf, self.robotics_status['prime_status']['influx'])
                                self.octoprint_instances[octoprint_name].write_gcode('dispense', mapped_pump_instructions[octoprint_name], self.pump_conf, self.robotics_status['prime_status']['influx'])
                            arm_settings = {}
                            gcode_files = ['aspirate.gcode', 'dispense.gcode'] 

                            # calculate target coordinates for next vial_window
                            # if at end of row, next vial_window will be the next row                           
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

                            await self.check_for_pause() # hang here if pause is called before starting asipration and dispense commands
                            try:
                                # influx routine resumed
                                self.robotics_status['mode'] = data['mode']
                                await self.fluidic_event(session, gcode_files, 'influx', arm_settings, print_string)
                            
                            except Exception:
                                # emergency stop, error encountered and manual intervention required
                                await session.close()
                                self.stop_robotics()
                                raise HelperEventError('error running vial_set fluidic event in influx_snake_helper, check logs for traceback')
                            
                            # check if volume for all vials in current vial window has been dispensed
                            # break loop if complete, otherwise continue pumping
                            if list(fractional_pump_instructions.values()).count('done') >= len(list(fractional_pump_instructions.values())):
                                break
                        
                        # finished dilutions for current vial_window, moving to next set of vials
                        change_row = False

                    # change row
                    change_row = True

                # reached end of dilution events for quad, move arm up before moving to next quad
                arm_coordinates_out = np.dot(transform_matrices[0], np.array([[vial_dilution_coordinates[0]], [vial_dilution_coordinates[1]], [1]]))
                await self.check_for_pause() # hang here if pause is called before moving arm to next row     
                try:
                    self.robotics_status['mode'] = data['mode'] # influx_routine resumed, change robotics_status mode
                    await self.move_arm({'x': arm_coordinates_out[0][0], 'y': arm_coordinates_out[1][0], 'z': z_vial_dilution[0]}, True)
                except Exception:
                    await session.close()
                    self.stop_robotics()
                    raise HelperEventError('error moving arm up after finishing snake influx path for %s, check logs' % quad_name)
                
                # send efflux command to continue IPPs for x amount of time if influx mode is set to dilution
                if self.robotics_status['mode'] == 'dilution':
                    self.evolver_ns.fluid_command(ipp_effux_command)

            await session.close()
            self.robotics_status['mode'] = 'idle'
            self.robotics_status['vial_window'] = None
            self.robotics_status['active_quad'] = None
            return {'done': True, 'message': 'None'}
        
        else:
            return {'done': False, 'message': 'robotics_status mode is not idle or pause'}

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

        # phase 2 coordinates (move arm to horizontally to next vial window)
        x2 = arm_settings['target_coordinates'][0]
        x3 = arm_settings['target_coordinates'][0]

        # phase 3 coordinates (move arm down into next vial window)
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
        """ Broadcasts the current robotics status to all connected clients. Also check if robotics server is connected to OctoPrint servers."""
        
        # update connection statuses of OctoPrint instances
        session = aiohttp.ClientSession()
        check = await self.check_status(session)
        for octoprint_name in self.octoprint_instances:
            if 'Offline' in check['octoprint_statuses'][octoprint_name]['state']:
                self.octoprint_instances[octoprint_name].connected = False
            else:
                self.octoprint_instances[octoprint_name].connected = True
            self.robotics_status['OctoPrint'][octoprint_name]['connection_status'] = self.octoprint_instances[octoprint_name].connected
        await session.close()

        # emit robotics status to all connected clients
        logging.info('Robotics status broadcast %s' % self.robotics_status)
        await self.sio.emit('broadcast', self.robotics_status, namespace = '/robotics')

    async def on_connect(self, sid, environ, auth):
        """ Called when client connects to server."""

        logger.info('Client connected to robotics_eVOLVER server')

    async def on_disconnect(self, sid):
        """ Called when client disconnects from server."""
        
        logger.info('Client disconnected to robotics_eVOLVER server')

    async def on_pause_robotics(self, sid, data):
        """ Pause all robotics. """

        self.robotics_status['mode'] = 'pause'        
        logger.info('Received pause request, pauing active routines.')

    async def on_resume_robotics(self, sid, data):
        """ Resume all robotics. """

        self.robotics_status['mode'] = 'idle'
        logger.info('Resuming active routines.')

    async def on_request_robotics_status(self, sid, data):
        """ Request the current robotics status. """

        logger.info('Request for current robotics status received.')
        return {'type': 'robotics', 'data': self.robotics_status}

    async def on_request_pump_conf(self, sid, data):
        """ Request the current pump configuration. """
        
        logger.info("Request for pump settings received.")
        return {'type': 'pump', 'data': self.pump_conf}
    
    async def on_fill_tubing_routine(self, sid, data):
        """ Fill tubing lines with connected syringe pumps. """

        start_time = time.time() # get start time of fill tubing routine to later calculate total elapsed time, useful for clients to gauge routine duration
        # execute fill tubing routine using fill_tubing_helper function
        try:
            logger.info('Filling syringe pump tubing')
            result = await self.fill_tubing_helper()
            end_time = time.time()
            return {'done': result['done'], 'routine': 'fill_tubing', 'robotics_status': self.robotics_status, 'elapsed_time': end_time - start_time, 'message': result['message']}
        except HelperEventError as e:
            end_time = time.time()
            logger.exception(e)
            return {'done': 'error', 'routine': 'fill_tubing', 'robotics_status': self.robotics_status, 'elapsed_time': end_time - start_time, 'message': 'HelperEventError encountered, check HT_eVOLVER logs for traceback'}

    async def on_prime_influx_routine(self, sid, data):
        """ Prime syringe pumps. Tubes must be filled first. """

        start_time = time.time() # get start time of influx routine to later calculate total elapsed time, useful for clients to gauge routine duration
        # execute prime_influx_helper function
        try:
            logger.info('Priming syringe pumps')
            result = await self.prime_influx_helper()
            end_time = time.time()
            return {'done': result['done'], 'routine': 'prime', 'robotics_status': self.robotics_status, 'elapsed_time': end_time - start_time, 'message': result['message']}
        except HelperEventError as e:
            end_time = time.time()
            logger.exception(e)
            return {'done': 'error', 'routine': 'prime', 'robotics_status': self.robotics_status, 'elapsed_time': end_time - start_time, 'message': 'HelperEventError encountered, check HT_eVOLVER logs for traceback'}

    async def on_prime_efflux_routine(self, sid, data):
        """ Prime syringe pumps. Tubes must be filled first. """

        start_time = time.time() # get start time of influx routine to later calculate total elapsed time, useful for clients to gauge routine duration
        # execute prime_efflux_helper function
        try:
            logger.info('Priming syringe pumps')
            result = await self.prime_efflux_helper(data)
            end_time = time.time()
            return {'done': result['done'], 'routine': 'prime', 'robotics_status': self.robotics_status, 'elapsed_time': end_time - start_time, 'message': result['message']}
        except HelperEventError as e:
            end_time = time.time()
            logger.exception(e)
            return {'done': 'error', 'routine': 'prime', 'robotics_status': self.robotics_status, 'elapsed_time': end_time - start_time, 'message': 'HelperEventError encountered, check HT_eVOLVER logs for traceback'}


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
        
        # get start time of influx routine to later calculate total elapsed time, useful for clients to gauge routine duration
        start_time = time.time()

        # execute influx routine using influx_snake_helper function
        try:
            logger.info('Executing the following influx routine command: %s' % data)
            result = await self.influx_snake_helper(data)
            end_time = time.time()
            return {'done': result['done'], 'routine': 'influx', 'robotics_status': self.robotics_status, 'elapsed_time': end_time - start_time, 'message':result['message']}
        except HelperEventError as e:
            end_time = time.time()
            logger.exception(e)
            return {'done': 'error', 'routine': 'influx', 'robotics_status': self.robotics_status, 'elapsed_time': end_time - start_time, 'message': 'HelperEventError encountered, check HT_eVOLVER logs for traceback'}

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
        if data['prime_status']:
            self.robotics_status['prime_status'] = data['prime_status']
        if data['reset_arm']:
            self.arm.clean_warn()
            self.arm.clean_error()
            self.arm.set_state(state=0)
    
    def on_stop_robotics(self, sid, data):
        """ Emergency stop all robotics """
        self.stop_robotics()
    
    async def on_reconnect_robotics(self, sid, data):
        """ Reconnect to OctoPrint servers and/or xArm. 
        
        Paramters:
            sid (str): The session ID.
            data (dict): The data containing the targets for reconnection. {'octoprint': True or False, 'xarm': True or False}"""
        
        if data['octoprint']:
            for octoprint_name in self.octoprint_instances:
                self.octoprint_instances[octoprint_name].connect()
        if data['xarm']:
            self.arm.connect()
        logger.info('Reconnecting to xArm and OctoPrint instances.')

    async def check_for_pause(self):
        while self.robotics_status['mode'] == 'pause':
            # pause detected, halt influx routine until pause is lifted or influx routine is stopped
            await self.sio.sleep(0.1)

    def setup_event_handlers(self):
        """ Attach event handlers to the server. Must be called before starting the server."""
        self.sio.on('disconnect', self.on_disconnect, namespace='/robotics')
        self.sio.on('connect', self.on_connect, namespace='/robotics')
        self.sio.on('fill_tubing_routine', self.on_fill_tubing_routine, namespace='/robotics')
        self.sio.on('prime_influx_routine', self.on_prime_influx_routine, namespace='/robotics')
        self.sio.on('prime_efflux_routine', self.on_prime_efflux_routine, namespace='/robotics')
        self.sio.on('influx_routine', self.on_influx_routine, namespace='/robotics')
        self.sio.on('request_robotics_status', self.on_request_robotics_status, namespace='/robotics')
        self.sio.on('request_pump_conf', self.on_request_pump_conf, namespace='/robotics')        
        self.sio.on('override_robotics_status', self.on_override_robotics_status, namespace='/robotics')
        self.sio.on('stop_robotics', self.on_stop_robotics, namespace='/robotics')
        self.sio.on('reconnect_robotics', self.on_reconnect_robotics, namespace='/robotics')
        self.sio.on('pause_robotics', self.on_pause_robotics, namespace='/robotics')
        self.sio.on('resume_robotics', self.on_resume_robotics, namespace='/robotics')