import time, os
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

#### CLIENT SIDE CODE ####
# enables communication with eVOLVER server to coordinate robotic tasks with eVOLVER

# GENERAL CLIENT VARIABLES # 
EVOLVER_NS = None

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

def setup_client(socketIO_client):
    global EVOLVER_NS
    EVOLVER_NS = EvolverNamespace('/dpu-evolver')
    socketIO_client.register_namespace(EVOLVER_NS)

#### END OF CLIENT SIDE CODE ####

##### SERVER SIDE CODE #####
# robotics server that runs on Raspberry Pi. Processes event based calls from dpu or other clients to control robotics hardware. Handles communication with OctoPi and XArm servers
sio = socketio.AsyncServer(async_handlers=True)
def attach(app):
    """
        Attach the server to the web application.
        Initialize server from config.
    """
    sio.attach(app)

# GENERAL SERVER VARIABLES #
# master variable that is continuously updated throughout robotics_server lifetime to track various states
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
        'OctoPrint_instances':[],
        'syringe_pump_status': [],
        'prime': False
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

# create urls for each octoprint server instance (1 per syringe pump) + necessary gcode directories for each syringe pump
OCTOPRINT_URLS = {}
for smoothie in PUMP_SETTINGS['smoothies']:
    url = "http://" + '192.168.1.15:' + str(PUMP_SETTINGS['smoothies'][smoothie]['port'])
    OCTOPRINT_URLS[smoothie] = url
    gcode_dir_exist = os.path.exists(PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'])
    if gcode_dir_exist:
        shutil.rmtree(PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'])
    os.makedirs(PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'])

# END OF GENERAL SERVER VARIABLES #

# xARM INITIALIZATION CODE #
arm = XArmAPI(XARM_IP)

# xArm callback functions
def error_warn_change_callback(data):
    ROBOTICS_STATUS['xArm']['error_code'] = data['error_code']
    ROBOTICS_STATUS['xArm']['warn_code'] = data['warn_code']
    if data['error_code'] != 0:
        stop_robotics()
        logger.error('xArm error_code %s encountered. Stopping all robotics and killing active routine(s)', (data['error_code']))
    if data['warn_code'] != 0:
        logger.error('xArm warning_code %s encountered.', (data['warn_code']))
    arm.release_error_warn_changed_callback(error_warn_change_callback)

def state_changed_callback(data):
    if data['state']:
        ROBOTICS_STATUS['xArm']['arm_state'] = data['state']

# Register callback functions with arm
arm.register_error_warn_changed_callback(callback=error_warn_change_callback)
arm.register_state_changed_callback(callback=state_changed_callback)

# Set initial arm parameters
arm.clean_warn()
arm.clean_error()
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
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
        try:
            arm.emergency_stop()
            requests.post(temp_url_1, headers=header, params=payload_1)
            requests.post(temp_url_2, headers=header, params=payload_2)
            logger.info('robotics abruptly stopped - check logs to identify cause of error')

            # update ROBOTICS_STATUS mode
            ROBOTICS_STATUS['mode'] = 'emergency_stop'
            return
        except:
            return

def write_gcode(mode, instructions, gcode='', gcode_path=''):
    """ Write gcode file for dispensing volumes into vials """
    
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

        if mode == "fill_tubing_in" or mode == 'volume_in':
            for pump in smoothie_instructions_map[smoothie]:
                # get pump port settings
                plunger = PUMP_SETTINGS['pumps'][pump]['motor_connections']['plunger']
                plunger_in_commands[count] = '{0}{1}'.format(plunger, smoothie_instructions_map[smoothie][pump])
                count = count + 1

            # combine gcode commands
            command = plunger_in_commands[0] + ' ' + plunger_in_commands[1]
            gcode = "G91\nG1 {0} F{1}\nM18".format(command, PUMP_SETTINGS['plunger_speed_in'])

        if mode == "fill_tubing_out" or mode =='volume_out':
            for pump in smoothie_instructions_map[smoothie]:
                plunger = PUMP_SETTINGS['pumps'][pump]['motor_connections']['plunger']
                valve = PUMP_SETTINGS['pumps'][pump]['motor_connections']['valve']
                valve_steps_on = PUMP_SETTINGS['pumps'][pump]['motor_connections']['valve_steps']
                valve_steps_off = PUMP_SETTINGS['pumps'][pump]['motor_connections']['valve_steps'] * -1

                if mode == "fill_tubing_out":
                    plunger_out_commands[count] = '{0}-{1}'.format(plunger, smoothie_instructions_map[smoothie][pump])
                if mode == "volume_out":
                    plunger_out_commands[count] = '{0}-{1}'.format(plunger, smoothie_instructions_map[smoothie][pump] + PUMP_SETTINGS['priming_steps'])
                prime_commands[count] = '{0}{1}'.format(plunger, PUMP_SETTINGS['priming_steps'])
                valve_commands['on'][count] = '{0}{1}'.format(valve, valve_steps_on)
                valve_commands['off'][count] = '{0}{1}'.format(valve, valve_steps_off)
                count = count + 1

            # combine gcode commands
            valve_on = valve_commands['on'][0] + ' ' + valve_commands['on'][1]
            valve_off = valve_commands['off'][0] + ' ' + valve_commands['off'][1]
            plunger_out = plunger_out_commands[0] + ' ' + plunger_out_commands[1]
            prime_out = prime_commands[0] + ' ' + prime_commands[1]

            if mode == "fill_tubing_out":
                gcode = "G91\nG1 {0} F20000\nG1 {1} F{2}\nG1 {3} F20000\nM18".format(valve_on, plunger_out, PUMP_SETTINGS['plunger_speed_out'], valve_off)
            if mode == "volume_out":
                gcode = "G91\nG1 {0} F20000\nG1 {1} F{2}\nG4 P100\nG1 {3} F20000\nG1 {4} F20000\nM18".format(valve_on, plunger_out, PUMP_SETTINGS['plunger_speed_out'], prime_out, valve_off)

        if mode == "prime_pumps":
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

            gcode = 'G91\nG1 {0} F20000\nG1 {1} F{2}\nG1 {3} F20000\nM18'.format(valve_on, plunger_in_commands, PUMP_SETTINGS['plunger_speed_in'], valve_off)
        else:
            gcode = gcode

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
    try:
        async with session.post(url, headers=header, data=payload) as response:
            return await response.json()
    except Exception as e:
        return 'retry'

async def check_job(session, smoothie):
    """ Return completion status of syringe pump actuation via GET request to Ocotprint server """
    
    # get url for target smoothie server and make API request to get pump status information
    url = OCTOPRINT_URLS[smoothie] + '/api/job'
    header={'X-Api-Key': API_KEY }
    try:
        async with session.get(url, headers=header) as response:
            return await response.json()
    except Exception as e:
        return 'check_job error'

async def check_status(session, gcode_paths):
    """ Return completion status of desired pump actuation event based on gcode path given. Returns True if desired pump(s) are ready for new actuation event, False if not """
    
    status_tasks = []
    for gcode_path in gcode_paths:
        # map gcode file to cognate smoothie
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
            if (status_results[i]['progress']['completion'] >= 100) and (status_results[i]['state'] == 'Operational') and (status_results[i]['job']['file']['name'] == filename):
                status_complete = status_complete + 1
        if status_complete == len(gcode_paths):
            return True
        else:
            return False
    except Exception as e:
        return False

async def prime_pumps_helper():
    """ Call this function to prime pumps after filling tubing """
    global ROBOTICS_STATUS

    if ROBOTICS_STATUS['mode'] == 'idle':

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
                if 'retry' in prime_pumps_results:
                    pass
                else:
                    break
            logger.debug('priming %s', (print_string))
        except Exception as e:
            await session.close()
            logger.error('error priming pumps: %s' %(e))
            return

        # verify that syringe pumps executed priming commands
        commands_complete = 0
        for result in prime_pumps_results:
            if result['done'] == True:
                commands_complete = commands_complete + 1

        # verify status of syringe_pumps to receive future commands
        if commands_complete == len(prime_pumps_tasks):
            ROBOTICS_STATUS['syringe_pumps']['prime'] = True
            while True:
                check = await check_status(session, check_files)
                if check:
                    break
                else:
                    pass
            await session.close()
            ROBOTICS_STATUS['mode'] = 'idle'
            logger.info('syringe pumps successfully primed for future dispensing')
            return
        else:
            await session.close()
            ROBOTICS_STATUS['xArm']['prime'] = False
            ROBOTICS_STATUS['mode'] = 'idle'
            logger.error('error priming syringe pumps')
            return

async def fill_tubing_helper():
    """ Call this function to fill tubing lines for all pumps that are in fluid mode (check/update server_conf.yml to set modes for pumps) """
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
        write_gcode('fill_tubing_in', instructions)
        write_gcode('fill_tubing_out', instructions)

        # create pre-dispense commands
        try:
            check_files = [None] * PUMP_SETTINGS['smoothie_num']
            fill_in_tasks = [None] * PUMP_SETTINGS['smoothie_num']
            fill_in_results = []
            while True:
                for smoothie in range(PUMP_SETTINGS['smoothie_num']):
                    gcode_path = os.path.join(PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'], 'fill_tubing_in.gcode')
                    fill_in_tasks[smoothie] = post_gcode_async(session, gcode_path, smoothie)
                    check_files[smoothie] = gcode_path
                fill_in_results = await asyncio.gather(*fill_in_tasks)
                if fill_in_results[0] == 'retry':
                    pass
                else:
                    break
            logger.debug('filling in tubing for %s', (print_string))

        except Exception as e:
            logger.warning('error filling in tubing')
            stop_robotics()
            await session.close()
            return

        # verify that pre-dispense commands executed
        commands_complete = 0
        for result in fill_in_results:
            if result['done'] == True:
                commands_complete = commands_complete + 1

        # verify that syringe_pumps are ready to receieve dispense commands
        if commands_complete == len(fill_in_tasks):
            while True:
                check = await check_status(session, check_files)
                if check:
                    break
                else:
                    pass

            # create dispense commands
            try:
                fill_out_tasks = [None] * PUMP_SETTINGS['smoothie_num']
                check_files = [None] * PUMP_SETTINGS['smoothie_num']
                fill_out_results = []
                while True:
                    for smoothie in range(PUMP_SETTINGS['smoothie_num']):
                        gcode_path = os.path.join(PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'], 'fill_tubing_out.gcode')
                        fill_out_tasks[smoothie] = post_gcode_async(session, gcode_path, smoothie)
                        check_files[smoothie] = gcode_path

                    fill_out_results = await asyncio.gather(*fill_out_tasks)
                    if fill_out_results[0] == 'retry':
                        pass
                    else:
                        break
                logger.debug('filling out tubing for %s', (print_string))

            except Exception as e:
                logger.warning('error filling out tubing')
                stop_robotics()
                await session.close()
                return

            # verify that dispense commands are executed
            commands_complete = 0
            for result in fill_out_results:
                if result['done'] == True:
                    commands_complete = commands_complete + 1

            # verify that syringe_pumps are ready to recieve future commands
            if commands_complete == len(fill_out_tasks):
                while True:
                    check = await check_status(session, check_files)
                    if check:
                        break
                    else:
                        pass
            else:
                logger.warning('cant validate that filling out tasks for %s were comleted', (print_string))
                stop_robotics()
                await session.close()
                return 'cant validate that filling out tasks for {0} were completed'.format(print_string)
        else:
            logger.warning('cant validate that filling in tasks for %s were comleted', (print_string))
            stop_robotics()
            await session.close()
            return 'cant validate that filling in tasks for {0} were comleted'.format(print_string)
        
        ROBOTICS_STATUS['mode'] = 'idle'
        await session.close()
        return 'fill tubing cycle success'

async def dilutions_helper(data):
    """ Main method for dilution routine for specified quads. Called every time eVOLVER client sends dilutions command to Robotics server """

    global ROBOTICS_STATUS, EVOLVER_NS
    if ROBOTICS_STATUS['mode'] == 'idle':
        syringe_pump_commands = data['message']['syringe_pump_message']
        ipp_effux_command = data['message']['ipp_efflux_message']
        
        # update status    
        ROBOTICS_STATUS['mode'] = 'influx'
        
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
            current_coordinates = [-18,36]
            change_row = False

            # calculate euclidean transformation matrix to convert vial_coordinates into arm coordinates using homing calibration
            # home based on first two and last two vials
            vial_1_out = np.array([robotics_conf['homing_coordinates'][quad_name]['vial_1']['x_out'], robotics_conf['homing_coordinates'][quad_name]['vial_1']['y']])
            vial_1_in = np.array([robotics_conf['homing_coordinates'][quad_name]['vial_1']['x_in'], robotics_conf['homing_coordinates'][quad_name]['vial_1']['y']])

            vial_17_out = np.array([robotics_conf['homing_coordinates'][quad_name]['vial_17']['x_out'], robotics_conf['homing_coordinates'][quad_name]['vial_17']['y']])
            vial_17_in = np.array([robotics_conf['homing_coordinates'][quad_name]['vial_17']['x_in'], robotics_conf['homing_coordinates'][quad_name]['vial_17']['y']])            
            
            z = np.array([robotics_conf['homing_coordinates'][quad_name]['vial_1']['z_out'], robotics_conf['homing_coordinates'][quad_name]['vial_1']['z_in']])        
        
            vial_coordinates = np.array([[0, 36], [90, 0]])
            calibrated_coordinates_out = np.array([vial_1_out, vial_17_out])
            calibrated_coordinates_in = np.array([vial_1_in, vial_17_in])
        
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
                if current_coordinates[1] == 18:
                    pump_map.reverse()

                # number of active vial sets
                num_vial_sets = 6 + (PUMP_SETTINGS['pump_num'] - 1)
                vial_window = []
                active_pumps = []

                # update vial window (set of vials in which arm is physically above). 
                # Vial window essentially behaves like a queue data structure, where vials are first in, first out as arm moves along dilution path
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

                    logger.info('current vial window is: %s', vial_window)
                    logger.info('active pumps for current vial window is: %s', active_pumps)

                    # using dilutions data strcuture, write gcode files to handle fluid dispension events for vial window
                    instructions = {}
                    ROBOTICS_STATUS['vial_window'] = vial_window
                    for i in range(len(vial_window)):
                        # for current vial, get cogante pump and steps
                        active_vial_name = 'vial_{0}'.format(vial_window[i])
                        active_pump = active_pumps[i]
                        pump_steps = syringe_pump_commands[active_pump][quad_name][active_vial_name]
                        instructions[active_pump] = pump_steps
                    write_gcode('volume_in', instructions)
                    write_gcode('volume_out', instructions)
                    
                    # get gcode files for vials in vial window
                    print_string = ''
                    for i in range(len(vial_window)):
                        print_string = print_string + 'vial_{0} '.format(vial_window[i])
                    print_string = print_string + 'in {0}'.format(quad_name)

                    # create pre-dispense commands
                    try:
                        pre_dispense_results = []
                        skip = False
                        arm_move_result = []
                        while True:
                            fluidic_tasks = []
                            check_files = []
                            for smoothie in range(PUMP_SETTINGS['smoothie_num']):
                                gcode_path = os.path.join(PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'], 'volume_in.gcode')
                                fluidic_tasks.append(post_gcode_async(session, gcode_path, smoothie))
                                check_files.append(gcode_path)
                            if skip == False:
                                fluidic_tasks.append(snake_path_step(current_coordinates, change_row, transform_matrices, z))

                            pre_dispense_results = await asyncio.gather(*fluidic_tasks)
                            if 'retry' in pre_dispense_results:
                                if pre_dispense_results[-1][0] == 'arm_moved':
                                    skip = True
                                    arm_move_result = pre_dispense_results[-1]
                                time.sleep(0.1)
                                continue
                            else:
                                if skip:
                                    pre_dispense_results.append(arm_move_result)
                                break
                        logger.debug('pumping in dilution volume for: %s', (print_string))
                    except Exception as e:
                        logger.error('error pumping in dilution volume: %s', (e))
                        stop_robotics()
                        await session.close()
                        return

                    # verify that arm and syringe pumps executed pre-dispense commands
                    commands_complete = 0
                    if pre_dispense_results[-1][0] == 'arm_moved':
                        commands_complete = commands_complete + 1
                        current_coordinates = pre_dispense_results[-1][1]
                    for result in pre_dispense_results[0:-1]:
                        if result['done'] == True:
                            commands_complete = commands_complete + 1

                    # verify that syringe_pumps are ready to receive dispense commands
                    if commands_complete == len(pre_dispense_results):
                        while True:
                            check = await check_status(session, check_files)
                            if check:
                                break
                            else:
                                pass

                        # create dispense commands
                        try:
                            dispense_results = []
                            while True:
                                fluidic_tasks = []
                                check_files = []
                                for smoothie in range(PUMP_SETTINGS['smoothie_num']):
                                    gcode_path = os.path.join(PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'], 'volume_out.gcode')
                                    fluidic_tasks.append(post_gcode_async(session, gcode_path, smoothie))
                                    check_files.append(gcode_path)
                                
                                dispense_results = await asyncio.gather(*fluidic_tasks)
                                if 'retry' in dispense_results:
                                    time.sleep(0.1)
                                    continue
                                else:
                                    break
                            logger.debug('pumping out dilution volume for: %s', (print_string))
                        except Exception as e:
                            logger.error('error pumping out dilution volume: %s' % (e))
                            stop_robotics()
                            await session.close()
                            return

                        # verify that syringe pumps are ready to receive future commands
                        commands_complete = 0
                        for result in dispense_results:
                            if result['done'] == True:
                                commands_complete = commands_complete + 1

                        if commands_complete == len(dispense_results):
                            while True:
                                check = await check_status(session, check_files)
                                if check:
                                    break
                                else:
                                    pass
                            logger.debug('finished dilution routine for: %s', (print_string))
                        else:
                            logger.error('cant validate that dilution volumes were pumped out for: %s', (print_string))
                            stop_robotics()
                            await session.close()
                            return
                    else:
                        logger.error('cant validate that dilution volumes were pumped in or that arm was moved to proper location for: %s', (print_string))
                        stop_robotics()
                        await session.close()
                        return

                    # finished dilutions for current vial_window, moving to next set of vials
                    change_row = False

                # change row
                change_row = True

            # reached end of dilution events for quad, move arm up before moving to next quad
            arm_coordinates_out = np.dot(transform_matrices[0], np.array([[current_coordinates[0]], [current_coordinates[1]], [1]]))
            await move_arm({'x': arm_coordinates_out[0][0], 'y': arm_coordinates_out[1][0], 'z': z[0]}, True)
            
            # send efflux command to continue IPPs for x amount of time
            EVOLVER_NS.fluid_command(ipp_effux_command)

        ROBOTICS_STATUS['mode'] = 'idle'
        ROBOTICS_STATUS['vial_window'] = None
        ROBOTICS_STATUS['active_quad'] = None
        
        await session.close()
        return 'dilution routine success'

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
    try:
        if ROBOTICS_STATUS['xArm']['error_code'] == 0:
            await asyncio.sleep(0.5)
            return arm.set_position(x=x, y=y, z=z, roll=user_params['roll'], pitch=user_params['pitch'], yaw=user_params['yaw'], speed=user_params['speed'], mvacc=user_params['mvacc'], wait=wait)
    except:
        logger.warning('error moving arm')
        stop_robotics()
        return 1

async def snake_path_step(current_coordinates, change_row, transform_matrices, z):
    """ Program arm to exit current vial_window and move to next set of vials. Returns string indicating that arm was succesfully moved """
    phase_1 = None
    row_logic = 1
    
    if current_coordinates[1] == 18:
        row_logic = -1
    
    # phase 1 coordinates (move arm up)
    x1_out = current_coordinates[0]
    y1_out = current_coordinates[1]
    
    # phase 2 coordinates (move arm above next vial window)
    # if end of row is reached move arm to next row
    x2_out = None
    y2_out = None
    if change_row:
        x2_out = x1_out
        y2_out = y1_out - 18
    else:
        x2_out = x1_out + row_logic*18 # subtract if in middle row, add if in first or third
        y2_out = y1_out

    # phase 3 coordinates (move arm into next vial window)
    x3_in = x2_out
    y3_in = y2_out

    next_coordinates_out = np.array([[x1_out, x2_out], [y1_out, y2_out], [1,1]])
    next_coordinates_in = np.array([[x3_in], [y3_in], [1]])

    # transform vial coordinates into arm coordinates for each phase
    arm_coordinates_out = np.dot(transform_matrices[0], next_coordinates_out)
    arm_coordinates_in = np.dot(transform_matrices[1], next_coordinates_in)

    # move arm to next vial window using transformed vial coordinates
    phase_1 = await move_arm({'x': arm_coordinates_out[0][0], 'y': arm_coordinates_out[1][0], 'z': z[0]}, True)

    if phase_1 == 0:
        phase_2 = await move_arm({'x': arm_coordinates_out[0][1], 'y': arm_coordinates_out[1][1], 'z': z[0]}, True)
    if phase_2 == 0:
        await move_arm({'x': arm_coordinates_in[0][0], 'y': arm_coordinates_in[1][0], 'z': z[1]}, True)

    # return new vial coordinates
    current_coordinates = np.array([x3_in, y3_in])
    return ['arm_moved', current_coordinates]

async def broadcast():
    global ROBOTICS_STATUS
    logging.info('Robotics status broadcast %s' % (ROBOTICS_STATUS))
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

@sio.on('dilutions', namespace = '/robotics')
async def on_dilutions(sid, data):
    """ Perform dilutions for target vials. Pump in and dispense dilution volume """
    sio.start_background_task(dilutions_helper,data)
    logger.info('Executing the following dilution commands: %s', (data))

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




