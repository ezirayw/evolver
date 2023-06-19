import time, os, json
import yaml
import aiohttp
import socketio
import asyncio
import shutil
import logging

logger = logging.getLogger(__name__)
EVOLVER_NS = None

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


sio = socketio.AsyncServer(async_handlers=True, ping_timeout=60)
def attach(app):
    """
        Attach the server to the web application.
        Initialize server from config.
    """
    sio.attach(app)

##### ROBOTICS SERVER VARIABLES #####
ROBOTICS_STATUS = {
    "operation": 'idle',
    "active_quad": None,
    "active_vials": None
}

# get settings from yaml file to setup server
SERVER_PATH = os.path.dirname(os.path.abspath(__file__))
ROBOTICS_CONFIG_PATH = os.path.join(SERVER_PATH, 'robotics_server_conf.yml')
DILUTIONS_PATH = os.path.join(SERVER_PATH)
ROBOTICS_CALIBRATION_PATH = os.path.join(SERVER_PATH, 'robotics_calibrations.json')

with open(ROBOTICS_CONFIG_PATH,'r') as conf:
    robotics_conf = yaml.safe_load(conf)

XARM_IP = robotics_conf['xarm_ip']
API_KEY = robotics_conf['octoprint_api_key']
PUMP_SETTINGS = robotics_conf['pump_settings']

# create urls for each octoprint server instance (1 per syringe pump)
# create necessary gcode directories for each syringe pump
OCTOPRINT_URLS = {}
for smoothie in PUMP_SETTINGS['smoothies']:
    url = "http://" + '192.168.1.15:' + str(PUMP_SETTINGS['smoothies'][smoothie]['port'])
    OCTOPRINT_URLS[smoothie] = url
    gcode_dir_exist = os.path.exists(PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'])
    if gcode_dir_exist:
        shutil.rmtree(PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'])
    os.makedirs(PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'])


############## xARM INITIALIZATION CODE ##################
"""
# xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK
# git clone git@github.com:xArm-Developer/xArm-Python-SDK.git
# cd xArm-Python-SDK
# python setup.py install
"""
from xarm import version
from xarm.wrapper import XArmAPI

arm = XArmAPI(XARM_IP)
arm.clean_warn()
arm.clean_error()
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
arm.reset(wait=True)

time.sleep(1)

params = {'speed': 5, 'acc': 2000, 'angle_speed': 20, 'angle_acc': 500, 'events': {}, 'variables': {}, 'quit': False}

# Register error/warn changed callback
def error_warn_change_callback(data):
    if data and data['error_code'] != 0:
        arm.set_state(4)
        params['quit'] = True
        logger.warning('err={}, quit'.format(data['error_code']))
        arm.release_error_warn_changed_callback(error_warn_change_callback)
arm.register_error_warn_changed_callback(error_warn_change_callback)


# Register state changed callback
def state_changed_callback(data):
    if data and data['state'] == 4:
        if arm.version_number[0] >= 1 and arm.version_number[1] >= 1 and arm.version_number[2] > 0:
            params['quit'] = True
            logger.debug('state=4, quit')
            arm.release_state_changed_callback(state_changed_callback)
arm.register_state_changed_callback(state_changed_callback)


# Register counter value changed callback
if hasattr(arm, 'register_count_changed_callback'):
    def count_changed_callback(data):
        logger.debug('counter val: ' + str(data['count']))
    arm.register_count_changed_callback(count_changed_callback)

############### END xARM INIT ##################
## This script is to define smoothieboard related functions, such as priming fluidic lines or dispensing volumes into vials

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
            gcode = "G91\nG1 {0} F15000\nM18".format(command)

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
                gcode = "G91\nG1 {0} F20000\nG1 {1} F15000\nG1 {2} F20000\nM18".format(valve_on, plunger_out, valve_off)
            if mode == "volume_out":
                gcode = "G91\nG1 {0} F20000\nG1 {1} F15000\nG4 P100\nG1 {2} F20000\nG1 {3} F20000\nM18".format(valve_on, plunger_out, prime_out, valve_off)

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
            plunger_in = plunger_in_commands[0] + ' ' + plunger_in_commands[1]
            valve_on = valve_commands['on'][0] + ' ' + valve_commands['on'][1]
            valve_off = valve_commands['off'][0] + ' ' + valve_commands['off'][1]

            gcode = 'G91\nG1 {0} F20000\nG1 {1} F15000\nG1 {2} F20000\nM18'.format(valve_on, plunger_in_commands, valve_off)
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
    except aiohttp.ClientError as e:
        return 'retry'
    else:
        return 'post_gcode_async error'

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

    # create tasks for prime pump events
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
        return 'error filling out tubing'

    # verify that robotic pipette has executed fill out events
    tasks_complete_prime_pumps = 0
    for result in prime_pumps_results:
        if result['done'] == True:
            tasks_complete_prime_pumps = tasks_complete_prime_pumps + 1

    # check status of pumps before priming pumps
    if prime_pumps_tasks == len(prime_pumps_tasks):
        while True:
            check = await check_status(session, check_files)
            if check:
                break
            else:
                pass
        await session.close()
        return 'priming pumps success'
    else:
        await session.close()
        return 'prime_pumps tasks not completed {0}'.format(print_string)


async def fill_tubing_helper():
    """ Call this function to fill tubing lines for all pumps that are in fluid mode (check/update server_conf.yml to set modes for pumps) """
    # update status
    global ROBOTICS_STATUS
    ROBOTICS_STATUS = {
        "operation": "fill_tubing",
        "active_quad": None,
        "active_vials": None
    }
    
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
        instructions[pump] = 300
    write_gcode('fill_tubing_in', instructions)
    write_gcode('fill_tubing_out', instructions)

    # create tasks for fill_tubing input events
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
        ROBOTICS_STATUS = {
            "operation": "error",
            "active_quad": None,
            "active_vials": None
        }
        await session.close()
        return 'error filling in tubing'

    # verify that robotic pipette has executed input events
    tasks_complete_fill_in = 0
    for result in fill_in_results:
        if result['done'] == True:
            tasks_complete_fill_in = tasks_complete_fill_in + 1

    # check status of pumps before moving to fill out events
    if tasks_complete_fill_in == len(fill_in_tasks):
        while True:
            check = await check_status(session, check_files)
            if check:
                break
            else:
                pass

        # create tasks for fill out events
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
            ROBOTICS_STATUS = {
                "operation": "error",
                "active_quad": None,
                "active_vials": None
            }
            await session.close()
            return 'error filling out tubing'

        # verify that robotic pipette has executed fill out events
        tasks_complete_fill_out = 0
        for result in fill_out_results:
            if result['done'] == True:
                tasks_complete_fill_out = tasks_complete_fill_out + 1

        # check status of pumps before priming pumps
        if tasks_complete_fill_out == len(fill_out_tasks):
            while True:
                check = await check_status(session, check_files)
                if check:
                    break
                else:
                    pass
        else:
            logger.warning('cant validate that filling out tasks for %s were comleted', (print_string))
            ROBOTICS_STATUS = {
                "operation": "error",
                "active_quad": None,
                "active_vials": None
            }
            await session.close()
            return 'cant validate that filling out tasks for {0} were completed'.format(print_string)
    else:
        logger.warning('cant validate that filling in tasks for %s were comleted', (print_string))
        ROBOTICS_STATUS = {
            "operation": "error",
            "active_quad": None,
            "active_vials": None
        }
        await session.close()
        return 'cant validate that filling in tasks for {0} were comleted'.format(print_string)
    
    ROBOTICS_STATUS = {
        "operation": "idle",
        "active_quad": None,
        "active_vials": None
    }

    await session.close()
    return 'fill tubing cycle success'

async def dilutions_helper(data):
    """ Main method for dilution routine for specified quads. Called every time eVOLVER client sends dilutions command to Robotics server """

    global ROBOTICS_STATUS, EVOLVER_NS
    
    syringe_pump_commands = data['message']['syringe_pump_message']
    ipp_effux_command = data['message']['ipp_efflux_message']
    
    # update status    
    ROBOTICS_STATUS = {
        "operation": "influx",
        "active_quad": None,
        "active_vials": None
    }
    
    # start asyncio Client Session
    session = aiohttp.ClientSession()

    # get vial coordinates from calibrations
    f1 = open(ROBOTICS_CALIBRATION_PATH)
    coordinate_config = json.load(f1)
    f1.close()

    # load in current config
    with open(ROBOTICS_CONFIG_PATH,'r') as conf:
        robotics_conf = yaml.safe_load(conf)
    PUMP_SETTINGS = robotics_conf['pump_settings']

    # loop through vials and execute fluidic events
    for quad_name in data['active_quads']:
        ROBOTICS_STATUS['active_quad'] = quad_name
        # create data structure to map fluidic events
        vial_map = [[0,1,2,3,4,5], [11,10,9,8,7,6], [12,13,14,15,16,17]]
        change_row = False

        for row_num in range(len(vial_map)):
            row = vial_map[row_num]

            # get list of pumps from server_conf.yml
            pump_map = []
            for pump_id in range(PUMP_SETTINGS['pump_num']):
                for pump in PUMP_SETTINGS['pumps']:
                    if PUMP_SETTINGS['pumps'][pump]['id'] == pump_id:
                        pump_map.append(pump)
                        break

            if row_num % 2 > 0:
                pump_map.reverse()

            # number of fluidic events based on number of pumps
            number_events = 6 + (PUMP_SETTINGS['pump_num'] - 1)
            active_vials = []
            active_pumps = []

            for event_num in range(number_events):
                # get list of vials and active pumps for current fluidic event
                if event_num < PUMP_SETTINGS['pump_num']:
                    active_vials.append(row[event_num])
                    active_pumps.append(pump_map[event_num])
                if event_num >= PUMP_SETTINGS['pump_num']:
                    active_vials.pop(0)
                    if event_num < len(row):
                        active_vials.append(row[event_num])
                    if event_num >= len(row):
                        active_pumps.pop(0)

                logger.info('active vials for fluidic event %d are: %s', event_num, active_vials)
                logger.info('active pumps for fluidic event %d are: %s', event_num, active_pumps)

                # using dilutions data strcuture, write gcode files to handle dilution events for active vials
                instructions = {}
                ROBOTICS_STATUS["active_vials"] = active_vials
                for i in range(len(active_vials)):
                    # for current vial, get cogante pump and steps
                    active_vial_name = 'vial_{0}'.format(active_vials[i])
                    active_pump = active_pumps[i]
                    pump_steps = syringe_pump_commands[active_pump][quad_name][active_vial_name]
                    instructions[active_pump] = pump_steps
                write_gcode('volume_in', instructions)
                write_gcode('volume_out', instructions)
                
                # get gcode files for active vials
                print_string = ''
                for i in range(len(active_vials)):
                    print_string = print_string + 'vial_{0} '.format(active_vials[i])
                print_string = print_string + 'in {0}'.format(quad_name)

                # create tasks for fluidic input events
                try:
                    fluidic_results = []
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
                            fluidic_tasks.append(pipette_next_step(row_num, quad_name, change_row, coordinate_config))

                        fluidic_results = await asyncio.gather(*fluidic_tasks)
                        if 'retry' in fluidic_results:
                            if fluidic_results[-1][0] == 'arm_moved':
                                skip = True
                                arm_move_result = fluidic_results[-1]
                            time.sleep(0.1)
                            continue
                        else:
                            if skip:
                                fluidic_results.append(arm_move_result)
                            break
                    logger.debug('pumping in dilution volume for: %s', (print_string))

                except Exception as e:
                    logger.warning('error pumping in dilution volume: %s', (e))
                    ROBOTICS_STATUS["operation"] = 'error'
                    await session.close()
                    return e

                # verify that robotic pipette has executed input events and update arm coordinate configurations
                tasks_complete = 0
                if fluidic_results[-1][0] == 'arm_moved':
                    tasks_complete = tasks_complete + 1
                    coordinate_config = fluidic_results[-1][1]
                for result in fluidic_results[0:-1]:
                    if result['done'] == True:
                        tasks_complete = tasks_complete + 1

                # check status of pumps before moving to output events
                if tasks_complete == len(fluidic_results):
                    while True:
                        check = await check_status(session, check_files)
                        if check:
                            break
                        else:
                            pass

                    # create tasks for fluidic output events
                    try:
                        fluidic_results = []
                        while True:
                            fluidic_tasks = []
                            check_files = []
                            for smoothie in range(PUMP_SETTINGS['smoothie_num']):
                                gcode_path = os.path.join(PUMP_SETTINGS['smoothies'][smoothie]['gcode_path'], 'volume_out.gcode')
                                fluidic_tasks.append(post_gcode_async(session, gcode_path, smoothie))
                                check_files.append(gcode_path)
                            
                            fluidic_results = await asyncio.gather(*fluidic_tasks)
                            if 'retry' in fluidic_results:
                                time.sleep(0.1)
                                continue
                            else:
                                break
                        logger.debug('pumping out dilution volume for: %s', (print_string))

                    except Exception as e:
                        logger.warning('error pumping out dilution volume')
                        ROBOTICS_STATUS["operation"] = 'error'
                        await session.close()
                        return e

                    # verify that arm has completed fluidic output events
                    tasks_complete = 0
                    for result in fluidic_results:
                        if result['done'] == True:
                            tasks_complete = tasks_complete + 1

                    # check status of pumps before moving to dilution events
                    if tasks_complete == len(fluidic_results):
                        while True:
                            check = await check_status(session, check_files)
                            if check:
                                break
                            else:
                                pass
                        logger.debug('finished dilution routine for: %s', (print_string))


                    else:
                        logger.warning('cant validate that dilution volumes were pumped out for: %s', (print_string))
                        ROBOTICS_STATUS["operation"] = 'error'
                        await session.close()
                        return 'cant validate that dilution volumes were pumped out for: {0}'.format(print_string)
                else:
                    logger.warning('cant validate that dilution volumes were pumped in or that arm was moved to proper location for: %s', (print_string))
                    ROBOTICS_STATUS["operation"] = 'error'
                    await session.close()
                    return 'cant validate that dilution volumes were pumped in and arm moved to proper location for: {0}'.format(print_string)

                # end of fluidic event
                change_row = False

            # reached end of row, increment arm coordinates to change row
            change_row = True

        # reached end of fluidic events for quad, move arm up before moving to next quad
        coordinates = {'x': coordinate_config[quad_name]['x_out'], 'y': coordinate_config[quad_name]['y'], 'z': coordinate_config[quad_name]['z_out']}
        await move_arm(coordinates, True)
        
        # send efflux command to continue IPPs for x amount of time
        EVOLVER_NS.fluid_command(ipp_effux_command)

    ROBOTICS_STATUS = {
        "operation": 'idle',
        "active_quad": None,
        "active_vials": None
    }
    
    await session.close()
    return 'dilution routine success'

## ## This script is to define xarm related functions that are not covered by auto-generated blockly functions. Useful for more precise movements.
async def move_arm(coordinates, wait):
    """ Return string indicating that arm was moved to specified XYZ location in space """
    x = coordinates['x']
    y = coordinates['y']
    z = coordinates['z']

    with open(ROBOTICS_CONFIG_PATH,'r') as config:
        user_params = yaml.safe_load(config)['xarm_params']

    # move xARM to specified coordinates
    try:
        if arm.error_code == 0 and not params['quit']:
            await asyncio.sleep(0.5)
            return arm.set_position(x=x, y=y, z=z, roll=user_params['roll'], pitch=user_params['pitch'], yaw=user_params['yaw'], speed=user_params['speed'], mvacc=user_params['mvacc'], wait=wait)
    except:
        logger.warning('error moving arm')


async def pipette_next_step(row_num, quad_name, change_row, coordinate_config):
    arm_location = {}
    part1 = None
    part2 = None

    # move arm up
    arm_location['x'] = coordinate_config[quad_name]['x_out']
    arm_location['y'] = coordinate_config[quad_name]['y']
    arm_location['z'] = coordinate_config[quad_name]['z_out']
    part1 = await move_arm(arm_location, False)

    if part1 == 0:
        # move arm above next set of active_vials
        if change_row:
            coordinate_config[quad_name]['x_in'] = coordinate_config[quad_name]['x_in'] + 18
            coordinate_config[quad_name]['x_out'] = coordinate_config[quad_name]['x_out'] + 18
            arm_location['x'] = coordinate_config[quad_name]['x_out']
            part2 = await move_arm(arm_location, False)
        else:
            if row_num % 2 == 0:
                coordinate_config[quad_name]['y'] = coordinate_config[quad_name]['y'] + 18
                arm_location['y'] = coordinate_config[quad_name]['y']
            else:
                coordinate_config[quad_name]['y'] = coordinate_config[quad_name]['y'] - 18
                arm_location['y'] = coordinate_config[quad_name]['y']
            part2 = await move_arm(arm_location, False)

        if part2 == 0:
            # move arm down into next set of active_vials
            arm_location['x'] = coordinate_config[quad_name]['x_in']
            arm_location['z'] = coordinate_config[quad_name]['z_in']
            await move_arm(arm_location, True)

    return ['arm_moved', coordinate_config]

async def broadcast():
    global ROBOTICS_STATUS
    logging.info('Robotics status broadcast %s' % (ROBOTICS_STATUS))
    await sio.emit('broadcast', ROBOTICS_STATUS, namespace = '/robotics')

## This script is to define all API calls to the flask server. Ensure that functions called in routes exist are pulled ./functions or ./manual
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

@sio.on('arm_test', namespace = '/robotics')
async def on_arm_test(sid, data):
    above_evolver()
    above_to_zero()

@sio.on('set_arm', namespace = '/robotics')
async def on_set_arm(sid, data):
    above_evolver()
    time.sleep(1)

@sio.on('reset_arm', namespace = '/robotics')
async def on_reset_arm(sid, data):
    above_to_zero()
    time.sleep(1)

@sio.on('move_to_quad', namespace = '/robotics')
async def on_move_to_quad(sid, data):
    quad = data['quad']
    if quad == 0:
        logger.info('moving to smart quad 0')
        quad_0_location()
    if quad == 1:
        logger.info('moving to smart quad 1')
        quad_1_location()
    if quad == 2:
        logger.info('moving to smart quad 2')
        quad_2_location()
    if quad == 3:
        logger.info('moving to smart quad 3')
        quad_3_location()
    time.sleep(1)

@sio.on('fill_tubing', namespace = '/robotics')
async def on_fill_tubing(sid, data):
    """ Fill fluidic tubing lines. Run prior to using robotics system """
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
