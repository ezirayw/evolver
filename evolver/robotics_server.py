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
        stop_robotics()

class FluidicEventError(Exception):
    def __init__(self, message):
        self.message = message
        logger.error('FluidicEventError Found: %s' % message)

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

#### eVOLVER CLIENT CODE ####
# enables communication with eVOLVER server to coordinate robotic tasks with eVOLVER

# GENERAL CLIENT VARIABLES # 
EVOLVER_NS = None
def setup_client(socketIO_client):
    global EVOLVER_NS
    EVOLVER_NS = EvolverNamespace('/dpu-evolver')
    socketIO_client.register_namespace(EVOLVER_NS)

#### END OF eVOLVER CLIENT CODE ####

##### ROBOTICS SERVER CODE #####
# Robotics server that runs on Raspberry Pi. Processes event based calls from dpu or other clients to control robotics hardware. 
# Handles communication with OctoPi and XArm servers
sio = socketio.AsyncServer(async_handlers=True)
def attach(app):
    """
        Attach the server to the web application.
        Initialize server from config.
    """
    sio.attach(app)

# GLOBAL SERVER VARIABLES DEFINITIONS #
# Global variable that is continuously updated throughout the server lifetime to track various states
ROBOTICS_STATUS = {
    'mode': 'idle',
    'active_quad': None,
    'vial_window': None,
    'xArm':{
        'warning_code': 0,
        'error_code': 0,
        'arm_state': None
        },
    'syringe_pumps': {
        'primed': False
        }
    }

SERVER_PATH = os.path.dirname(os.path.abspath(__file__))
ROBOTICS_CONFIG_PATH = os.path.join(SERVER_PATH, 'robotics_server_conf.yml')
DILUTIONS_PATH = os.path.join(SERVER_PATH)

with open(ROBOTICS_CONFIG_PATH,'r') as conf:
    robotics_conf = yaml.safe_load(conf)

XARM_IP = robotics_conf['xarm_ip']
XARM_INITIAL_POSITION = {'x': robotics_conf['default_arm_positions']['initial_position']['x'], 'y': robotics_conf['default_arm_positions']['initial_position']['y'], 'z': robotics_conf['default_arm_positions']['initial_position']['z']}
API_KEY = robotics_conf['octoprint_api_key']
PUMP_SETTINGS = robotics_conf['pump_settings']

# Create urls for each octoprint server instance (1 per syringe pump) + necessary gcode directories for each syringe pump
OCTOPRINT_URLS = {}
for smoothie in PUMP_SETTINGS['smoothies']:
    url = "http://" + '192.168.1.15:' + str(PUMP_SETTINGS['smoothies'][smoothie]['port'])
    OCTOPRINT_URLS[smoothie] = url
    gcode_dir_exist = os.path.exists(PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'])
    if gcode_dir_exist:
        shutil.rmtree(PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'])
    os.makedirs(PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'])

# END OF GLOBAL SERVER VARIABLES DEFINITIONS #

# xARM INITIALIZATION CODE #
arm = XArmAPI(XARM_IP)

# xArm callback functions
def error_warn_change_callback(data):
    ROBOTICS_STATUS['xArm']['error_code'] = data['error_code']
    ROBOTICS_STATUS['xArm']['warn_code'] = data['warn_code']
    if data['error_code'] != 0:
        logger.error('xArm error_code %s encountered. Stopping all robotics and killing active routine(s)', data['error_code'])
        stop_robotics()
    if data['warn_code'] != 0:
        logger.warn('xArm warning_code %s encountered.', data['warn_code'])
    arm.release_error_warn_changed_callback(error_warn_change_callback)

def state_changed_callback(data):
    if data['state']:
        ROBOTICS_STATUS['xArm']['arm_state'] = data['state']
        if[data['state']] != 0:
            stop_robotics()
    arm.release_state_changed_callback(state_changed_callback)

# Register callback functions with arm
arm.register_error_warn_changed_callback(callback=error_warn_change_callback)
arm.register_state_changed_callback(callback=state_changed_callback)

# Set initial arm parameters
arm.clean_warn()
arm.clean_error()
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
arm.set_collision_sensitivity(1)
arm.set_self_collision_detection(True)
ROBOTICS_STATUS['xArm']['arm_state'] = arm.get_state()

# End of xARM INITIALIZATION CODE #

# SERVER FUNCTIONS #
def stop_robotics():
    """ Stop all robotics. Called to stop robotics in event of errors and/or emergencies. Will interupt any concurrent robotic tasks and prevent future tasks if interrepted during a routine. """
    global ROBOTICS_STATUS

    # emergency stop xArm and cancel current syringe job commands and disconnect from OctoPrint servers
    payload_1 = {'command': 'cancel'}
    payload_2 = {'command': 'disconnect'}
    for url in OCTOPRINT_URLS:
        temp_url_1 = OCTOPRINT_URLS[url] + '/api/job'
        temp_url_2 = OCTOPRINT_URLS[url] + '/api/connection'
        header = {'X-Api-Key': API_KEY, 'Content-Type': 'application/json'}
        arm.emergency_stop()
        requests.post(temp_url_1, headers=header, params=payload_1)
        requests.post(temp_url_2, headers=header, params=payload_2)

        # update ROBOTICS_STATUS mode
        ROBOTICS_STATUS['mode'] = 'emergency_stop'
        logger.info('stop_robotics() called - check logs to identify cause of error')

def write_gcode(mode, instructions, gcode='', gcode_path=''):
    """ Write gcode file for dispensing volumes into vials """
    global ROBOTICS_STATUS

    # use motor connection settings from config to map pump instructions to proper smoothieboards
    smoothie_instructions_map = {}
    for smoothie in PUMP_SETTINGS['smoothies']:
        smoothie_instructions = {}
        for pump_target in instructions:
            if pump_target in PUMP_SETTINGS['pumps'] and PUMP_SETTINGS['pumps'][pump_target]['smoothie'] == smoothie:
                smoothie_instructions[pump_target] = instructions[pump_target]
        smoothie_instructions_map[smoothie] = smoothie_instructions

    for smoothie in smoothie_instructions_map:
        plunger_in_commands = ['' , '']
        plunger_out_commands = ['' , '']
        prime_commands = ['' , '']
        valve_commands = {'on': ['' , ''] , 'off': ['', ''], 'steps': 0 }
        count = 0

        if mode == 'aspirate':
            for pump in smoothie_instructions_map[smoothie]:
                # get pump port settings
                plunger = PUMP_SETTINGS['pumps'][pump]['motor_connections']['plunger']
                plunger_in_commands[count] = '{0}{1}'.format(plunger, smoothie_instructions_map[smoothie][pump])
                count = count + 1

            # combine gcode commands
            command = plunger_in_commands[0] + ' ' + plunger_in_commands[1]
            gcode = "G91\nG1 {0} F{1}\nM18".format(command, PUMP_SETTINGS['plunger_speed_in'])

        elif mode =='dispense':
            for pump in smoothie_instructions_map[smoothie]:
                plunger = PUMP_SETTINGS['pumps'][pump]['motor_connections']['plunger']
                valve = PUMP_SETTINGS['pumps'][pump]['motor_connections']['valve']
                valve_steps_on = PUMP_SETTINGS['pumps'][pump]['motor_connections']['valve_steps']
                valve_steps_off = PUMP_SETTINGS['pumps'][pump]['motor_connections']['valve_steps'] * -1

                plunger_out_commands[count] = '{0}-{1}'.format(plunger, smoothie_instructions_map[smoothie][pump])
                if ROBOTICS_STATUS['syringe_pumps']['primed'] and smoothie_instructions_map[smoothie][pump] != 0:
                    plunger_out_commands[count] = '{0}-{1}'.format(plunger, smoothie_instructions_map[smoothie][pump] + PUMP_SETTINGS['priming_steps'])
                    prime_commands[count] = '{0}{1}'.format(plunger, PUMP_SETTINGS['priming_steps'])
                
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

            gcode = "G91\nG1 {0} F20000\nG4 P150\nG1 {1} F{2}\nG4 P150\nG1 {3} F20000\nM18".format(valve_on, plunger_out, PUMP_SETTINGS['plunger_speed_out'], valve_off)
            if ROBOTICS_STATUS['syringe_pumps']['primed']:
                gcode = "G91\nG1 {0} F20000\nG4 P150\nG1 {1} F{2}\nG4 P150\nG1 {3} F15000\nG4 P150\nG1 {4} F20000\nM18".format(valve_on, plunger_out, PUMP_SETTINGS['plunger_speed_out'], prime, valve_off)

        elif mode == "prime_pumps":
            for pump in smoothie_instructions_map[smoothie]:
                plunger = PUMP_SETTINGS['pumps'][pump]['motor_connections']['plunger']
                valve = PUMP_SETTINGS['pumps'][pump]['motor_connections']['valve']
                valve_steps_on = PUMP_SETTINGS['pumps'][pump]['motor_connections']['valve_steps']
                valve_steps_off = PUMP_SETTINGS['pumps'][pump]['motor_connections']['valve_steps'] * -1

                plunger_in_commands[count] = '{0}{1}'.format(plunger, smoothie_instructions_map[smoothie][pump])
                valve_commands['on'][count] = '{0}{1}'.format(valve, valve_steps_on)
                valve_commands['off'][count] = '{0}{1}'.format(valve, valve_steps_off)
                count = count + 1

            # combine gcode commands
            valve_on = valve_commands['on'][0] + ' ' + valve_commands['on'][1]
            valve_off = valve_commands['off'][0] + ' ' + valve_commands['off'][1]

            gcode = 'G91\nG1 {0} F20000\nG4 P150\nG1 {1} F{2}\nG4 P150\nG1 {3} F20000\nM18'.format(valve_on, plunger_in_commands, PUMP_SETTINGS['plunger_speed_in'], valve_off)
        else:
            gcode = 'M18'

        # write command to gcode file
        filename = mode + '.gcode'
        gcode_path = os.path.join(PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'], filename)
        f = open(gcode_path, 'w')
        f.write(gcode)
        f.close()

async def post_gcode_async(session, gcode_path, smoothie):
    """ Return response status of POST request to Ocotprint server for actuating syringe pump """

    payload = {'file': open(gcode_path, 'rb'), 'print':'true'}
    # get url for target smoothie server and make API request to send gcode file to actuate pumps
    url = OCTOPRINT_URLS[smoothie] + '/api/files/local'
    header={'X-Api-Key': API_KEY }
    result = None
    try:
        async with session.post(url, headers=header, data=payload) as response:
            result = await response.json()
    except Exception as e:
        result = {'done': False}
        logger.debug('post_gcode issue occured: %s', e)
    finally:
        return {'result': result}

async def check_job(session, smoothie):
    """ Return completion status of syringe pump actuation via GET request to Ocotprint server """
    
    # get url for target smoothie server and make API request to get pump status information
    url = OCTOPRINT_URLS[smoothie] + '/api/job'
    header={'X-Api-Key': API_KEY }
    result = None
    try:
        async with session.get(url, headers=header) as response:
            result = await response.json()
    except Exception as e:
        result = False
        logger.debug('check job issue occured: %s', e)
    finally:
        return {'result': result}

async def check_status(session, gcode_paths):
    """ Return completion status of desired pump actuation event based on gcode path given. Returns True if desired pump(s) are ready for new actuation event, False if not """

    # map gcode file to cognate smoothie
    status_tasks = []
    result = None
    for gcode_path in gcode_paths:
        gcode_parent_path = os.path.split(gcode_path)[0]
        for smoothie in PUMP_SETTINGS['smoothies']:
            if PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'] == gcode_parent_path:
                status_tasks.append(check_job(session, smoothie))
                break

    # use check_job() to get status information on desired pump(s) by parsing JSON data
    try:
        status_results = await asyncio.gather(*status_tasks)
        status_complete = 0
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

async def fluidic_event(session, gcode_files, mode, arm_settings = {}, print_string = ''):
    """ Call this function to execute coordinated aspiration/dispensing events (syringe pumps + robotic arm) """
    # arm_settings{"current_coordinates": current_coordinates, "transform_matrices": transform_matrices, "z": z}
    # create aspiration commands
    try:
        aspirate_results = []
        skip_arm = False
        arm_move_result = {}
        while True:
            fluidic_tasks = []
            check_files = []
            for smoothie in range(PUMP_SETTINGS['smoothie_num']):
                gcode_path = os.path.join(PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'], gcode_files[0])
                fluidic_tasks.append(post_gcode_async(session, gcode_path, smoothie))
                check_files.append(gcode_path)
            if skip_arm == False and arm_settings:
                fluidic_tasks.append(arm_path(arm_settings))

            aspirate_results = await asyncio.gather(*fluidic_tasks, return_exceptions=True)
            if any([ ('done', 'Kill') in dict['result'].items() for dict in aspirate_results ]):
                raise FluidicEventError('kill command from arm_path() detected, exiting routine')
            elif any([ ('done', False) in dict['result'].items() for dict in aspirate_results ]):
                if aspirate_results[-1]['result']['done']:
                    skip_arm = True
                    arm_move_result = aspirate_results[-1]
                await asyncio.sleep(0.5)
            else:
                if skip_arm:
                    aspirate_results.append(arm_move_result)
                break
        logger.info('running aspiration in %s mode for: %s' % (mode, print_string))
    
    except Exception as e:
        logger.error('error running aspiration in %s mode' % (mode))
        raise e
        #return {'result': {'done':False}}

    else:
        # verify that arm and syringe pumps executed aspiration commands
        commands_complete = 0
        for dict in aspirate_results:
            if dict['result']['done']:
                commands_complete = commands_complete + 1

        # verify that syringe_pumps are ready to receive dispense commands
        if commands_complete == len(aspirate_results):
            while True:
                check = await check_status(session, check_files)
                if check['result']:
                    break
                else:
                    await asyncio.sleep(0.5)

            # create dispense commands
            try:
                dispense_results = []
                while True:
                    fluidic_tasks = []
                    check_files = []
                    for smoothie in range(PUMP_SETTINGS['smoothie_num']):
                        gcode_path = os.path.join(PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'], gcode_files[1])
                        fluidic_tasks.append(post_gcode_async(session, gcode_path, smoothie))
                        check_files.append(gcode_path)

                    dispense_results = await asyncio.gather(*fluidic_tasks, return_exceptions=True)
                    if any([ ('done', False) in dict['result'].items() for dict in dispense_results ]):
                        await asyncio.sleep(0.5)
                    else:
                        break
                logger.info('running dispense during %s for: %s' % (mode, print_string))
            
            except Exception as e:
                logger.error('error running dispense in %s mode, stopping robotics' % (mode))
                raise e
                #return {'result': {'done':False}}
            
            else:
                # verify that syringe pumps are ready to receive future commands
                commands_complete = 0
                for dict in dispense_results:
                    if dict['result']['done']:
                        commands_complete = commands_complete + 1

                if commands_complete == len(dispense_results):
                    while True:
                        check = await check_status(session, check_files)
                        if check['result']:
                            break
                        else:
                            await asyncio.sleep(0.5)
                    
                    logger.info('finished %s for: %s' % (mode, print_string))
                    return {'result': {'done': True}}
                else:
                    raise FluidicEventError('cant validate that volumes were pumped during %s for: %s' % (mode, print_string))
                    #return {'result': {'done':False}}
        else:
            raise FluidicEventError('cant validate that volumes were pumped or that arm was moved to proper location during %s for: %s' % (mode, print_string))
            #return {'result': {'done':False}}

async def prime_pumps_helper():
    """ Call this function to prime pumps after filling tubing """
    global ROBOTICS_STATUS

    if ROBOTICS_STATUS['mode'] == 'idle' and not ROBOTICS_STATUS['syringe_pumps']['primed']:

        ROBOTICS_STATUS['mode'] = 'priming'
        # start asyncio Client Session
        session = aiohttp.ClientSession()
    
        # get fluid pumps from server_conf and write prime pumps instructions
        instructions = {}
        print_string = ''

        # load in current config
        with open(ROBOTICS_CONFIG_PATH,'r') as conf:
            robotics_conf = yaml.safe_load(conf)
        PUMP_SETTINGS = robotics_conf['pump_settings']

        for pump in PUMP_SETTINGS['pumps']:
            print_string = print_string + '{0} pump'.format(pump)
            instructions[pump] = PUMP_SETTINGS['priming_steps']
        write_gcode('prime_pumps', instructions)

        # create syringe_pump priming commands
        try:
            prime_pumps_tasks = [None] * PUMP_SETTINGS['smoothie_num']
            check_files = [None] * PUMP_SETTINGS['smoothie_num']
            prime_pumps_results = []
            while True:
                for smoothie in range(PUMP_SETTINGS['smoothie_num']):
                    gcode_path = os.path.join(PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'], 'prime_pumps.gcode')
                    prime_pumps_tasks[smoothie] = post_gcode_async(session, gcode_path, smoothie)
                    check_files[smoothie] = gcode_path

                prime_pumps_results = await asyncio.gather(*prime_pumps_tasks)
                if any([ ('done', False) in dict['result'].items() for dict in prime_pumps_results ]):
                    continue
                else:
                    break
            logger.debug('priming %s', print_string)
        
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
                ROBOTICS_STATUS['syringe_pumps']['primed'] = True
                while True:
                    check = await check_status(session, check_files)
                    if check['result']:
                        break
                    else:
                        await asyncio.sleep(0.5)
                await session.close()
                ROBOTICS_STATUS['mode'] = 'idle'
                logger.info('syringe pumps successfully primed for future dispensing')
                return {'result':{'done': True}}
            else:
                await session.close()
                ROBOTICS_STATUS['mode'] = 'idle'
                logger.error('error priming syringe pumps, commands not completed')
                return {'result':{'done': False}}

async def fill_tubing_helper():
    """ Call this function to fill tubing lines for all pumps in robotics_server_conf.yml """
    global ROBOTICS_STATUS

    # check status before executing robotic function
    if ROBOTICS_STATUS['mode'] == 'idle':
        
        ROBOTICS_STATUS['mode'] = 'fill_tubing'
        # start asyncio Client Session
        session = aiohttp.ClientSession()

        # get fluid pumps from server_conf
        instructions = {}
        print_string = ''

        # load in current config
        with open(ROBOTICS_CONFIG_PATH,'r') as conf:
            robotics_conf = yaml.safe_load(conf)
        PUMP_SETTINGS = robotics_conf['pump_settings']

        for pump in PUMP_SETTINGS['pumps']:
            print_string = print_string + '{0} pump '.format(pump)
            instructions[pump] = 350
        write_gcode('aspirate', instructions)
        write_gcode('dispense', instructions)
        gcode_files = ['aspirate.gcode', 'dispense.gcode']

        print_string = 'filling syringe pump tubing'

        await fluidic_event(session, gcode_files, 'fill_tubing', print_string=print_string)
        
        ROBOTICS_STATUS['mode'] = 'idle'
        await session.close()
        return {'result':{'done': True}}

async def influx_snake_helper(data):
    """ Main method for running higher level influx routines (i.e. dilutions or setup) for specified quads.  """
    # data = {'message': fluidic_commands, 'active_quads': quads, 'mode': 'dilution', 'wash': True/False}
    global ROBOTICS_STATUS, EVOLVER_NS
    
    if ROBOTICS_STATUS['mode'] == 'idle':
        syringe_pump_commands = data['message']['syringe_pump_message']
        ipp_effux_command = data['message']['ipp_efflux_message']
        
        # update status (either dilution or vial_setup)
        ROBOTICS_STATUS['mode'] = data['mode']
        
        # start asyncio Client Session
        session = aiohttp.ClientSession()

        # get calibrated smart quad coordinates from config
        with open(ROBOTICS_CONFIG_PATH,'r') as conf:
            robotics_conf = yaml.safe_load(conf)
        
        PUMP_SETTINGS = robotics_conf['pump_settings']
        vial_map = [[0,1,2,3,4,5], [11,10,9,8,7,6], [12,13,14,15,16,17]]

        # loop through sets of vials (vial_window) and execute fluid dispension events
        for quad_name in data['active_quads']:
            ROBOTICS_STATUS['active_quad'] = quad_name
            vial_dilution_coordinates = [-18,36]
            wash_station_coordinates = [0,-24]
            change_row = False

            # calculate euclidean transformation matrix to convert vial_coordinates into arm coordinates using homing calibration
            # home based on first two and last two vials
            vial_0_out = np.array([robotics_conf['homing_coordinates'][quad_name]['vial_0']['x_out'], robotics_conf['homing_coordinates'][quad_name]['vial_0']['y']])
            vial_0_in = np.array([robotics_conf['homing_coordinates'][quad_name]['vial_0']['x_in'], robotics_conf['homing_coordinates'][quad_name]['vial_0']['y']])

            vial_17_out = np.array([robotics_conf['homing_coordinates'][quad_name]['vial_17']['x_out'], robotics_conf['homing_coordinates'][quad_name]['vial_17']['y']])
            vial_17_in = np.array([robotics_conf['homing_coordinates'][quad_name]['vial_17']['x_in'], robotics_conf['homing_coordinates'][quad_name]['vial_17']['y']])            
            
            z_vial_dilution = np.array([robotics_conf['homing_coordinates'][quad_name]['vial_0']['z_out'], robotics_conf['homing_coordinates'][quad_name]['vial_0']['z_in']])
            z_wash_station = np.array([robotics_conf['homing_coordinates'][quad_name]['wash_station']['z_out'], robotics_conf['homing_coordinates'][quad_name]['wash_station']['z_in']])
        
            vial_coordinates = np.array([[0, 36], [90, 0]])
            calibrated_coordinates_out = np.array([vial_0_out, vial_17_out])
            calibrated_coordinates_in = np.array([vial_0_in, vial_17_in])
        
            transform_matrix_out = rigid_transform(vial_coordinates, calibrated_coordinates_out)
            transform_matrix_in = rigid_transform(vial_coordinates, calibrated_coordinates_in)
            transform_matrices = np.stack((transform_matrix_out, transform_matrix_in))

            for row_num in range(np.size(vial_map, 0)):
                current_vial_row = vial_map[row_num]

                # get list of pumps from server_conf.yml
                pump_map = []
                for pump_id in range(PUMP_SETTINGS['pump_num']):
                    for pump in PUMP_SETTINGS['pumps']:
                        if PUMP_SETTINGS['pumps'][pump]['id'] == pump_id:
                            pump_map.append(pump)
                            break

                # flip pump map to account for leading pump position changing in middle row of quad
                if row_num == 1:
                    pump_map.reverse()

                # number of active vial sets
                num_vial_sets = 6 + (PUMP_SETTINGS['pump_num'] - 1)
                vial_window = []
                active_pumps = []

                # update vial window (set of vials in which dispense needles are physically above). 
                # Vial window essentially behaves like a queue data structure, where vials are first in, first out as arm moves along snake dilution path
                for x in range(num_vial_sets):
                    if x < PUMP_SETTINGS['pump_num']:
                        vial_window.append(current_vial_row[x])
                        active_pumps.append(pump_map[x])
                    if x >= PUMP_SETTINGS['pump_num']:
                        vial_window.pop(0)
                        if x < len(current_vial_row):
                            vial_window.append(current_vial_row[x])
                        if x >= len(current_vial_row):
                            active_pumps.pop(0)
                    
                    print_string = ''
                    for i in range(len(vial_window)):
                        print_string = print_string + 'vial_{0} '.format(vial_window[i])
                    print_string = print_string + 'in {0}'.format(quad_name)

                    logger.info('current vial window is: %s', vial_window)
                    logger.info('active pumps for current vial window is: %s', active_pumps)

                    # execute wash step for current vial_window
                    if data['wash']:
                        z = {'current': z_vial_dilution, 'target': z_wash_station}        
                        arm_settings = {"current_coordinates": vial_dilution_coordinates, "target_coordinates": wash_station_coordinates, "transform_matrices": transform_matrices, "z": z}
                        wash_pump_instructions = {}
                        for pump in PUMP_SETTINGS['pumps']:
                            wash_pump_instructions[pump] = 0
                        write_gcode('aspirate', wash_pump_instructions)
                        write_gcode('dispense', wash_pump_instructions)
                        gcode_files = ['aspirate.gcode', 'dispense.gcode']
                        
                        try:
                            await fluidic_event(session, gcode_files, 'wash', arm_settings, print_string)
                        except Exception as e:
                            await session.close()
                            logger.error('error running wash fluidic event in influx_snake_helper, check logs for traceback')
                            logger.exception(e)
                            return {'result': {'done':False}}
                        
                    
                    # extract pump volume (in steps) from syringe pump command and write gcode files to handle dilution events for current vial window
                    fractional_pump_instructions = {}
                    max_pump_instructions = {}
                    max_pump_num = {}
                    
                    ROBOTICS_STATUS['vial_window'] = vial_window
                    # for each vial in the current vial window extract how many maximum syringe volumes and fractional volumes will be pumped
                    for i in range(len(vial_window)):
                        active_vial_name = 'vial_{0}'.format(vial_window[i])
                        active_pump = active_pumps[i]
                        pump_step_fraction = syringe_pump_commands[active_pump][quad_name][active_vial_name] / PUMP_SETTINGS["pumps"][active_pump]["max_steps"]
                        max = int(pump_step_fraction)
                        fractional_pump_instructions[active_pump] = int((pump_step_fraction - max) * PUMP_SETTINGS["pumps"][active_pump]["max_steps"])
                        max_pump_num[active_pump] = max

                    # move arm only during first pump event to position arm for current vial_window
                    arm_moved = False
                    while True:
                        # if desired volume is above maximum syringe volume, continously pump maximum syringe volumes, otherwise pump fractional syringe volume
                        for pump in active_pumps:
                            if max_pump_num[pump] > 0:
                                max_pump_instructions[pump] = PUMP_SETTINGS["pumps"][pump]["max_steps"]
                                write_gcode('aspirate', max_pump_instructions)
                                write_gcode('dispense', max_pump_instructions)
                                max_pump_num[pump] = max_pump_num[pump] - 1

                            if max_pump_num[pump] == 0:
                                write_gcode('aspirate', fractional_pump_instructions)
                                write_gcode('dispense', fractional_pump_instructions)
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

                        try:
                            await fluidic_event(session, gcode_files, 'influx', arm_settings, print_string)
                        
                        except:
                            await session.close()
                            logger.error('error running vial_set dilution in influx_snake_helper, check logs for traceback')
                            logger.exception(e)
                            return {'result': {'done':False}}
                        
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
            try:
                await move_arm({'x': arm_coordinates_out[0][0], 'y': arm_coordinates_out[1][0], 'z': z_vial_dilution[0]}, True)
            except Exception as e:
                await session.close()
                logger.error('error moving arm to up after finishing quad influx_snake function, check logs')
                logger.exception(e)
                return {'result': {'done':False}}
            
            # send efflux command to continue IPPs for x amount of time if influx mode is set to dilution
            if ROBOTICS_STATUS['mode'] == 'dilution':
                EVOLVER_NS.fluid_command(ipp_effux_command)
                await asyncio.sleep(0.5)
                # send secondary command in case first not registered
                EVOLVER_NS.fluid_command(ipp_effux_command) 

        ROBOTICS_STATUS['mode'] = 'idle'
        ROBOTICS_STATUS['vial_window'] = None
        ROBOTICS_STATUS['active_quad'] = None
        
        await session.close()
        return {'result': {'done':True}}

def rigid_transform(quad_coordinates, arm_coordinates):
    tform = ski.transform.EuclideanTransform()
    tform.estimate(quad_coordinates,arm_coordinates)
    return tform

async def move_arm(coordinates, wait):
    """ Move arm to desired XYZ coordinates """

    global ROBOTICS_STATUS
    x = coordinates['x']
    y = coordinates['y']
    z = coordinates['z']

    with open(ROBOTICS_CONFIG_PATH,'r') as config:
        user_params = yaml.safe_load(config)['xarm_params']

    # move xARM to specified coordinates
    if ROBOTICS_STATUS['xArm']['error_code'] == 0:
        result = arm.set_position(x=x, y=y, z=z, roll=user_params['roll'], pitch=user_params['pitch'], yaw=user_params['yaw'], speed=user_params['speed'], mvacc=user_params['mvacc'], wait=wait)
        if result < 0:
            raise RoboticsError('failure moving xARM during move_arm(), error code {0} given'.format(result))
        return result
    else:
        raise RoboticsError('cannot execute move_arm() since xArm error code is not 0')

async def arm_path(arm_settings):
    """ Program arm to exit current vial_window and move to next set of vials. Returns string indicating that arm was succesfully moved """
    # arm_settings{"current_coordinates": current_coordinates, "target_coordinates": target_coordinates, "transform_matrices": transform_matrices, "z": z, "wash_dry_delay": time}
    
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
        await move_arm({'x': arm_coordinates_out[0][0], 'y': arm_coordinates_out[1][0], 'z': arm_settings['z']['current'][0]}, True)
        # add delay to allow ethanol to dry from influx needle
        if 'wash_dry_delay' in arm_settings:
            await asyncio.sleep(arm_settings['wash_dry_delay'])
        await move_arm({'x': arm_coordinates_out[0][1], 'y': arm_coordinates_out[1][1], 'z': arm_settings['z']['target'][0]}, True)
        await move_arm({'x': arm_coordinates_in[0][0], 'y': arm_coordinates_in[1][0], 'z': arm_settings['z']['target'][1]}, True)
        return {'result': {'done': True}}        
    
    except Exception as e:
        logger.exception(e)
        return {'result': {'done': 'Kill'}}        
    
async def broadcast():
    global ROBOTICS_STATUS
    logging.info('Robotics status broadcast %s' % ROBOTICS_STATUS)
    await sio.emit('broadcast', ROBOTICS_STATUS, namespace = '/robotics')

# END OF SERVER FUNCTIONS #

# SERVER EVENT HANDLER FUNCTIONS #
@sio.on('connect', namespace = '/robotics')
async def on_connect(sid, environ):
    logger.info('dpu connected to robotics_eVOLVER server')

@sio.on('disconnect', namespace = '/robotics')
async def on_disconnect(sid):
    logger.info('dpu disconnected to robotics_eVOLVER server')

@sio.on('get_pump_settings', namespace = '/robotics')
async def on_get_pump_settings(sid, data):
    """ GET pump settings from Robotics_server.conf file """
    data = {'data': PUMP_SETTINGS}
    logger.info("Retreiving pump settings...")
    await sio.emit('active_pump_settings', data, namespace = '/robotics')

@sio.on('fill_tubing', namespace = '/robotics')
async def on_fill_tubing(sid, data):
    """ Fill tubing lines. Run prior to using robotics system """
    sio.start_background_task(fill_tubing_helper)
    logger.info('Filling syringe pump tubing')

@sio.on('prime_pump', namespace = '/robotics')
async def on_prime_pump(sid, data):
    """ Prime pump after filling lines """
    sio.start_background_task(prime_pumps_helper)
    logger.info('Priming syringe pumps')

@sio.on('influx_snake', namespace = '/robotics')
async def on_dilutions(sid, data):
    """ Perform dilutions and media setup for target vials. """
    sio.start_background_task(influx_snake_helper,data)
    logger.info('Executing the following dilution commands: %s', data)

@sio.on('request_robotics_status', namespace = '/robotics')
async def on_request_robotics_status(sid, data):
    logger.info('Retreiving active robotics status...')
    await sio.emit('active_robotics_status', ROBOTICS_STATUS, namespace = '/robotics')

@sio.on('override_robotics_status', namespace = '/robotics')
async def on_override_robotics_status(sid, data):
    """ Override ROBOTICS_STATUS """
    if len(data['parameters']) > 2:
        logger.warning('Incorrect number of parameters passed - try again.')
        return
    if 'xArm' in data['parameters']:
        logger.info('Overriding robotics status...')
        ROBOTICS_STATUS['xArm'][data['parameters'][1]] = data['value']
        return
    if 'syringe_pump' in data['parameters']:
        logger.info('Overriding robotics status...')
        ROBOTICS_STATUS['syringe_pumps'][data['parameters'][1]] = data['value']
        return
    ROBOTICS_STATUS[data['parameters']] = data['value']

@sio.on('stop_robotics', namespace = '/robotics')
async def on_stop_robotics(sid, data):
    """ Emergency stop all robotics """
    stop_robotics()