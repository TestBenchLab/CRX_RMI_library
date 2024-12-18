from server.exceptions import ServerException
from utils import GetAttrEnum, get_config_from_file, get_validation_schemas
from dotenv import load_dotenv
from typing import Callable, Dict, Tuple
from server.amqp import AMQPClient
from server.client import Request, Response
from server.http import HttpServer
from server.validation import Validator
from exceptions import BaseException
from server.exceptions import ServerException
import logging
import argparse
from redis import Redis, exceptions as redis_exceptions
import json
import traceback
import sys
import os
import csv
import socket
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname("."), '../CRX_RMI_library/src')))

#import model
#from model.action import Action
#from model.definition import Drilling
#import mars

load_dotenv()

__VALIDATION_SCHEMA_DIR = './schemas'
__SERVER_CONFIG_FILE = './config/server.yaml'
__MARS_CONFIG_FILE = './config//mars.yaml'

# declare amqp topics
__AMQP_TOPICS = 'request.rmi_library','report.rmi_library'

# declare redis constants
# for status
STATUS_KEYSPACE = 'mars.rmi_library.status'
REDIS_CLIENT: Redis = None

LOGGER = logging.getLogger("rmi_library")

# ROBOT_IP = "192.168.1.10"
ROBOT_IP = "192.168.0.104"
ROBOT_PORT = 16001
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

ErrorID_to_str = {
  2556932 : "Invalid Position Register (2556932)",
  2556936 : "Cannot Execute TP program (2556936)",
  2556937 : "RMI is Not Running (2556937)",
  2556941 : "Invalid RMI Command (2556941)",
  2556943 : "Invalid Controller State (2556943)",
  2556950 : "Invalid Text String (2556950)",
  2556957 : "Invalid sequence ID (2556957)"
}

key_mapping = {
    'J1': 'X',
    'J2': 'Y',
    'J3': 'Z',
    'J4': 'W',
    'J5': 'P',
    'J6': 'R',
    'J7': 'Ext1',
    'J8': 'Ext2',
    'J9': 'Ext3'
}


TIME_BUFFER = 0.005 # waiting time between each instruction
SEQUENCE_ID = -1

def get_error_string(error_code:int):
  global ErrorID_to_str
  error_str = ErrorID_to_str[error_code] if error_code in ErrorID_to_str else "Error code : " + str(error_code)
  return error_str

# define the status list
class STATUS(GetAttrEnum):
  INIT = 'init'
  READY = 'ready'
  RUNNING = 'running'
  ERROR = 'error'

# init the global vars
status:STATUS = None

def update_status(new_status:STATUS)->None:
  global status
  """update the process status locally and in Redis Database

  Args:
      new_status (STATUS): the new process status
  """
  status = new_status
  REDIS_CLIENT.set(STATUS_KEYSPACE, status)

def purge_status():
  """delete the redis keyspaces
  """
  REDIS_CLIENT.delete(STATUS_KEYSPACE)
  REDIS_CLIENT.close()

class ConfigLoader(argparse.Action):
  def __call__(self, parser, namespace, values, option_strings=None) -> Dict:
    try:
      if '--environment-config' in option_strings:
        return get_config_from_file(values)
      elif '--validation-schemas' in option_strings:
        assert os.path.isdir(values)
        # get validations schemas stored in __VALIDATION_SCHEMA_DIR
        return get_validation_schemas(values)
    except BaseException as error:
      error.add_in_stack(['CONFIG'])
      raise error
    except AssertionError as error:
      raise BaseException(["CONFIG", "VALIDATION_SCHEMA"],
                        f"validation schema directory {values} not found") 


def get_http_para_from_config(http_config:Dict) -> Tuple[str, int]:
  """read the configuration and return http host and port

  Args:
      http_config (Dict): config definition

  Raises:
      BaseException: raise if config not conform

  Returns:
      Tuple[str, int]: http host and port
  """
  try:
    host = http_config['host']
    port = http_config['port']
    return host, port
  except KeyError as error:
    raise BaseException(["SERVER", "HTTP"],
                        f"http server config is not conform, {error.args[0]} parameter is missing")


def build_AMQP_client(amqp_config:Dict):
  """build an AMQP client from the configuration

  Args:
      amqp_config (Dict): amqp configuration

  Raises:
      BaseException: raise if config is not conform
      ServerException: raise if error during amqp client init

  Returns:
      Response: response object
  """
  try:
    host = amqp_config['host']
    port = amqp_config['port']
    exchange = amqp_config['exchange']
    ex_name = exchange['name']
    ex_type = exchange['type']
    AMQP_CLIENT = AMQPClient(name="rmi_library",
                            host=host,
                            port=port,
                            exchange_name=ex_name,
                            exchange_type=ex_type)                  
    return AMQP_CLIENT

  except KeyError as error:
    missing_key = error.args[0]
    raise BaseException(['CONFIG', 'SERVER', 'AMQP'],
                        f"the amqp configuration parameter {missing_key} is missing")

  except ServerException as error:
    error.add_in_stack(['SERVER'])
    raise error

def route_request(request:Request,
                  response:Response,
                  next:Callable):
  """function to route the AMQP method request

  Args:
      request (Request): _description_
      response (Response): _description_
      next (Callable): _description_
  """
  # >LOG
  print(f'message received on {request.query.get("path")}')
  
  if request.query.get('path') == '/rmi_library/test'\
    and request.query.get('method') == 'GET':
      try:
        
        init_rmi_connection()

        # config = {"UToolNumber" : 2, "UFrameNumber" : 3, "Front" : 0, "Up" : 0, "Left" : 0, "Flip" : 0, "Turn4" : 0, "Turn5" : 0, "Turn6" : 0}

        # measured_positions = []
        
        # with open('coordonnees.csv', newline='') as csvfile:
        #   row_count = sum(1 for _ in csv.DictReader(csvfile))

        # with open('coordonnees.csv', newline='') as csvfile:
        #     reader = csv.DictReader(csvfile)
        #     i=0
        #     for row in reader:
        #       print(f'{i}/{row_count}')
        #       rmi_write_position_register(10, config, row)
        #       rmi_call('INBOLT')
              
        #       _, pr11 = rmi_read_position_register(11)
        #       _, pr12 = rmi_read_position_register(12)
            
        #       # measured_positions.append(pr11.get("Position").values())
        #       measured_positions.append(pr12.get("Position").values())
              
        #       i += 1
        
        config = {"UToolNumber" : 2, "UFrameNumber" : 3, "Front" : 0, "Up" : 0, "Left" : 0, "Flip" : 0, "Turn4" : 0, "Turn5" : 0, "Turn6" : 0}

        measured_positions = []
        
        with open('coordonnees.csv', newline='') as csvfile:
          row_count = sum(1 for _ in csv.DictReader(csvfile))

        with open('coordonnees.csv', newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            i=0
            for row in reader:
              print(f'{i}/{row_count}')
              rmi_write_position_register(10, config, row)
              rmi_call('INBOLT')
              
              _, pr11 = rmi_read_position_register(11)
              _, pr12 = rmi_read_position_register(12)
            
              # measured_positions.append(pr11.get("Position").values())
              measured_positions.append(pr12.get("Position").values())
              
              i += 1
              
        # with open ("output.csv",'a') as csvfile:
        #   print('writing csv')
        #   writer = csv.writer(csvfile, delimiter=',',
        #                     quotechar='|', quoting=csv.QUOTE_MINIMAL)

        #   for row in measured_positions:
        #     writer.writerow(row)              
        
        ##print("Message received :", request.body)
        #response = rmi_connect()
        #time.sleep(4)
        #b, response = rmi_get_status()
        ##print("response :", response)
        #time.sleep(4)
        #res = is_robot_available_to_initialize(response)
        #print(res)
        #time.sleep(4)
        #print(":)")
        #b, response = rmi_initialize()
        #print("response :", response, b)
        
        #rmi_set_uf_ut(2,2)
        #uf = rmi_read_uf_data(2)
        #print(uf)
        #rmi_write_uf_data(2, {"X" : 1, "Y" : 1, "Z" : 1, "W" : 1, "P" : 1, "R" : 1})
        #uf = rmi_read_uf_data(2)
        #print(uf)
        
        #_, pos = rmi_read_cartesian_position()
        #print(pos)
        #_, pos = rmi_read_joint_angles()
        #print(pos)
        #rmi_set_override(52)
      
        #rmi_connect()
        #rmi_reset()
        
        #_, uf_ut = rmi_get_uf_ut()
        #print(uf_ut)
        
        #_, pr = rmi_read_position_register(1)
        #print(pr)        
        
        #_, pr = rmi_read_position_register(2)
        #print(pr)
        
        #_, tcp_speed = rmi_read_tcp_speed()
        #print(tcp_speed)

        #_, response = rmi_set_u_frame(3)
        #print("response :", response)
        
        #joint_angles = {key_mapping.get(key, key): value for key, value in position.get('JointAngle', {}).items()}
        
        # _, response = rmi_set_u_frame(3)
                
        # config = {"UToolNumber" : 2, "UFrameNumber" : 3, "Front" : 0, "Up" : 0, "Left" : 0, "Flip" : 0, "Turn4" : 0, "Turn5" : 0, "Turn6" : 0}
        # _, response = rmi_read_cartesian_position()
        # print("position1", response.get("Position"))
        
        # _, response = rmi_wait_time(5.0)        
        # _, response = rmi_linear_motion(config,{'X': 90.0, 'Y': 90.0, 'Z': 90.0, 'W': 0.0, 'P': 0.0, 'R': 0.0, 'Ext1': 0.0, 'Ext2': 0.0, 'Ext3': 0.0},"mmSec",100,"FINE",1)
        #_, response = rmi_joint_motion(config,response.get("Position"),"Percent",100,"FINE",1)
        #_, response = rmi_joint_motion(config,{'X': 90.0, 'Y': 90.0, 'Z': 90.0, 'W': 0.0, 'P': 0.0, 'R': 0.0, 'Ext1': 0.0, 'Ext2': 0.0, 'Ext3': 0.0},"Percent",100,"FINE",1)

        # _, response = rmi_read_cartesian_position()
        # print("position2", response.get("Position"))


        #_, response = rmi_set_u_tool(1)
        #print("response :", response, "\n")
        
        #rmi_connect()
        #b, response = rmi_get_status()
        #print("response :", response, b)
        
      except Exception as e:
        print(e)
  elif request.query.get('path') == '/rmi_request/init_rmi_connection':
    try:
      #init_rmi_connection()
      rmi_connect()
      b, response = rmi_get_status()
      print("response :", response, b)
    except Exception as e:
        print(e)
  else:
    # >LOG
    print('path not conform')

def send_message(packet):
  type(packet)
  try:
    #send the message
    sock.sendall((json.dumps(packet) + "\r\n").encode('utf-8'))
    
    #decode the response
    response = sock.recv(1024).decode('utf-8')
    response_data = json.loads(response)
    print("REPONSE RECUE", response_data)
    #print(f"Robot response : {response_data}")
    return response_data
  except Exception as e: print("error send_message()", e)

def rmi_connect():
  time.sleep(TIME_BUFFER)
  try:
    # we create a new socket with base ip and port
    global sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ROBOT_IP, ROBOT_PORT))
    
    # connection to robot via RMI
    connect_packet = {"Communication" : "FRC_Connect"}
    response = send_message(connect_packet)
    if response is None:
      raise Exception("Error while sending connect_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      # if ErrorID == 0, the service is connected to robot
      is_connected = error_id == 0
      error_str = get_error_string(error_id)
      
      # the connection request give us a new port number to use, so we recreate the socket
      new_port = response.get("PortNumber", None)
      if new_port is None:
        raise Exception("Error while fetching new PortNumber")
      else:
        sock.close()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ROBOT_IP, new_port))
      LOGGER.info("RMI_CONNECT successful") if is_connected else LOGGER.info("RMI_CONNECT failed, ErrorID = " + error_str)
      return is_connected, response
  except Exception as e: LOGGER.error(f"error rmi_connect(): {str(e)}")

def rmi_disconnect():
  time.sleep(TIME_BUFFER)
  try:
    disconnect_packet = {"Communication" : "FRC_Disconnect"}
    response = send_message(disconnect_packet)
    if response is None:
      raise Exception("Error while sending disconnect_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      # if ErrorID == 0, the service is disconnected from robot
      is_disconnected = error_id == 0
      error_str = get_error_string(error_id)
      if is_disconnected:
        global sock
        sock.close()
        LOGGER.info("RMI_DISCONNECT successful")
      else:
        LOGGER.info("RMI_DISCONNECT failed, ErrorID = " + error_str)
      return is_disconnected, response
  except Exception as e: print(f"error rmi_disconnect(): {str(e)}")


def rmi_get_status(verbose=True):
  time.sleep(TIME_BUFFER)
  try:
    get_status_packet = {"Command" : "FRC_GetStatus"}
    response = send_message(get_status_packet)
    if response is None:
      raise Exception("Error while sending get_status_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      if verbose:
        error_str = get_error_string(error_id)
        LOGGER.info("RMI_GETSTATUS successful") if request_successful else LOGGER.info("RMI_GETSTATUS failed, ErrorID = " + error_str)
      print("STATUS =", response) #temp
      return request_successful, response
  except Exception as e: print("error rmi_get_status()", e)


def is_robot_available_to_initialize(data):
  if data.get("ErrorID", -1) != 0:
    print("Error while fetching robot status")
  else:
    servo_ready = data.get("ServoReady", None)
    tp_mode = data.get("TPMode", None)
    
    if servo_ready is not None and tp_mode is not None:
      is_available = servo_ready == 1 and tp_mode == 0
      return is_available
    else:
      print("Error while reading ServoReady or TPMode")
      return False


def rmi_initialize():
  time.sleep(TIME_BUFFER)
  try:
    initialize_packet = {"Command" : "FRC_Initialize"}
    response = send_message(initialize_packet)
    if response is None:
      raise Exception("Error while sending initialize_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      #if ErrorID == 0, rmi is initialized
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_INITIALIZE successful") if request_successful else LOGGER.info("RMI_INITIALIZE failed, ErrorID = " + error_str)
      return request_successful, response
  except Exception as e: print("error rmi_initialize()", e)

##En fait, ne sert à rien, ces messages sont envoyés depuis le robot
#def rmi_terminate():
#  try:
#    initialize_packet = {"Command" : "FRC_Terminate"}
#    response = send_message(initialize_packet)
#    return response
#  except Exception as e: print("error rmi_terminate()", e)

def rmi_abort():
  time.sleep(TIME_BUFFER)
  try:
    abort_packet = {"Command" : "FRC_Abort"}
    response = send_message(abort_packet)
    if response is None:
      raise Exception("Error while sending abort_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_ABORT successful") if request_successful else LOGGER.info("RMI_ABORT failed, ErrorID = " + error_str)
      return response
  except Exception as e: print("error rmi_abort()", e)


def rmi_reset():
  time.sleep(TIME_BUFFER)
  try:
    reset_packet = {"Command" : "FRC_Reset"}
    response = send_message(reset_packet)
    if response is None:
      raise Exception("Error while sending reset_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_RESET successful") if request_successful else LOGGER.info("RMI_RESET failed, ErrorID = " + error_str)
      return response
  except Exception as e: print("error rmi_reset()", e)


def rmi_pause():
  time.sleep(TIME_BUFFER)
  try:
    pause_packet = {"Command" : "FRC_Pause"}
    response = send_message(pause_packet)
    if response is None:
      raise Exception("Error while sending pause_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_PAUSE successful") if request_successful else LOGGER.info("RMI_PAUSE failed, ErrorID = " + error_str)
      return response
  except Exception as e: print("error rmi_pause()", e)
  

def rmi_continue():
  time.sleep(TIME_BUFFER)
  try:
    continue_packet = {"Command" : "FRC_Continue"}
    response = send_message(continue_packet)
    if response is None:
      raise Exception("Error while sending continue_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_CONTINUE successful") if request_successful else LOGGER.info("RMI_CONTINUE failed, ErrorID = " + error_str)
      return response
  except Exception as e: print("error rmi_continue()", e)
  

#TODO savoir à quoi sert le "count" optionnel
def rmi_read_error():
  time.sleep(TIME_BUFFER)
  try:
    read_error_packet = {"Command" : "FRC_ReadError"}
    response = send_message(read_error_packet)
    return response
  except Exception as e: print("error read_error_packet()", e)


def init_rmi_connection():
  try:
    is_connected, _ = rmi_connect()
    if not is_connected:
      raise Exception("Error while executing rmi_connect()")
    rmi_reset()
    rmi_abort()
    _, status = rmi_get_status()
    global SEQUENCE_ID
    #SEQUENCE_ID = status.get("NextSequenceID")
    SEQUENCE_ID = 1
    is_status_ok = is_robot_available_to_initialize(status)
    LOGGER.info("Robot ready for initialization") if is_status_ok else LOGGER.info("waiting for ServoReady = 1 and TPMODE = 0...")
    while not is_status_ok:
      time.sleep(1)
      _, status = rmi_get_status(False)
      is_status_ok = is_robot_available_to_initialize(status)
    print("Robot status is OK, initialization...")
    _, _ = rmi_initialize()
  except Exception as e: print("error init_rmi_connection()", e)


def rmi_set_uf_ut(uf:int, ut:int, group=1):
  try:
    time.sleep(TIME_BUFFER)
    assert(0 <= uf and uf <= 255 and 0 <= ut and ut <= 255), "uf or ut is out of range"
    set_uf_ut_packet = {"Command" : "FRC_SetUFrameUTool",
                        "UFrameNumber" : uf,
                        "UToolNumber" : ut,
                        "Group" : group}
    response = send_message(set_uf_ut_packet)
    if response is None:
      raise Exception("Error while sending set_uf_ut_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_SET_UF_UT successful") if request_successful else LOGGER.info("RMI_SET_UF_UT failed, ErrorID = " + error_str)
      return request_successful, response
  except AssertionError as ae: print(ae)
  except Exception as e: print("error rmi_set_uf_ut()", e)


def rmi_read_uf_data(uf:int, group=1):
  try:
    time.sleep(TIME_BUFFER)
    assert(0 <= uf and uf <= 255), "uf number is out of range"
    read_uf_data_packet = {"Command" : "FRC_ReadUFrameData",
                        "FrameNumber" : uf,
                        "Group" : group}
    response = send_message(read_uf_data_packet)
    if response is None:
      raise Exception("Error while sending read_uf_data_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_READ_UF_DATA successful") if request_successful else LOGGER.info("RMI_READ_UF_DATA failed, ErrorID = " + error_str)
      return response
  except AssertionError as ae: print(ae)
  except Exception as e: print("error rmi_read_uf_data()", e)


def rmi_write_uf_data(uf_number:int, frame:dict, group=1):
  try:
    time.sleep(TIME_BUFFER)
    expected_keys = {"X", "Y", "Z", "W", "P", "R"}
    assert set(frame.keys()) == expected_keys, "the input frame isn't correctly written ; frame : " + str(frame)
    assert(0 <= uf_number and uf_number <= 255), "uf number is out of range"
    write_uf_data_packet = {"Command" : "FRC_WriteUFrameData",
                        "FrameNumber" : uf_number,
                        "Frame" : frame,
                        "Group" : group}
    response = send_message(write_uf_data_packet)
    if response is None:
      raise Exception("Error while sending write_uf_data_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_WRITE_UF_DATA successful") if request_successful else LOGGER.info("RMI_WRITE_UF_DATA failed, ErrorID = " + error_str)
      return request_successful, response
  except AssertionError as ae: print(ae)
  except Exception as e: print("error rmi_write_uf_data()", e)


def rmi_read_ut_data(ut:int, group=1):
  try:
    time.sleep(TIME_BUFFER)
    assert(0 <= ut and ut <= 255), "ut number is out of range"
    read_ut_data_packet = {"Command" : "FRC_ReadUToolData",
                        "ToolNumber" : ut,
                        "Group" : group}
    response = send_message(read_ut_data_packet)
    if response is None:
      raise Exception("Error while sending read_ut_data_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_READ_UT_DATA successful") if request_successful else LOGGER.info("RMI_READ_UT_DATA failed, ErrorID = " + error_str)
      return response
  except AssertionError as ae: print(ae)
  except Exception as e: print("error rmi_read_ut_data()", e)


def rmi_write_ut_data(ut_number:int, frame:dict, group=1):
  try:
    time.sleep(TIME_BUFFER)
    expected_keys = {"X", "Y", "Z", "W", "P", "R"}
    assert set(frame.keys()) == expected_keys, "the input frame isn't correctly written ; frame : " + str(frame)
    assert(0 <= ut_number and ut_number <= 255), "ut number is out of range"
    write_ut_data_packet = {"Command" : "FRC_WriteUToolData",
                        "ToolNumber" : ut_number,
                        "Frame" : frame,
                        "Group" : group}
    response = send_message(write_ut_data_packet)
    if response is None:
      raise Exception("Error while sending write_ut_data_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_WRITE_UT_DATA successful") if request_successful else LOGGER.info("RMI_WRITE_UT_DATA failed, ErrorID = " + error_str)
      return request_successful, response
  except AssertionError as ae: print(ae)
  except Exception as e: print("error rmi_write_ut_data()", e)


def rmi_read_d_in(port_number:int):
  a = 0
  #TODO


def rmi_write_d_out(port_number:int, port_value):
  a = 0
  #TODO


def rmi_read_cartesian_position(group=1):
  time.sleep(TIME_BUFFER)
  try:
    read_cartesian_position_packet = {"Command" : "FRC_ReadCartesianPosition",
                                      "Group" : group}
    response = send_message(read_cartesian_position_packet)
    if response is None:
      raise Exception("Error while sending read_cartesian_position_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_READ_CARTESIAN_POSITION successful") if request_successful else LOGGER.info("RMI_READ_CARTESIAN_POSITION failed, ErrorID = " + error_str)
      return request_successful, response
  except Exception as e: print("error rmi_read_cartesian_position()", e)


def rmi_read_joint_angles(group=1):
  time.sleep(TIME_BUFFER)
  try:
    read_joint_angles_packet = {"Command" : "FRC_ReadJointAngles",
                                "Group" : group}
    response = send_message(read_joint_angles_packet)
    if response is None:
      raise Exception("Error while sending read_joint_angles_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_READ_CARTESIAN_POSITION successful") if request_successful else LOGGER.info("RMI_READ_CARTESIAN_POSITION failed, ErrorID = " + error_str)
      return request_successful, response
  except Exception as e: print("error rmi_read_joint_angles()", e)


def rmi_set_override(value:int):
  time.sleep(TIME_BUFFER)
  try:
    assert(1 <= value and value <= 100), "override value is out of range"
    set_override_packet =  {"Command" : "FRC_SetOverRide",
                            "Value" : value}
    response = send_message(set_override_packet)
    if response is None:
      raise Exception("Error while sending set_override_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_SET_OVERRIDE successful") if request_successful else LOGGER.info("RMI_SET_OVERRIDE failed, ErrorID = " + error_str)
      return request_successful, response
  except AssertionError as ae: print(ae)
  except Exception as e: print("error rmi_set_override()", e)


def rmi_get_uf_ut(group=1):
  time.sleep(TIME_BUFFER)
  try:
    get_uf_ut_packet = {"Command" : "FRC_GetUFrameUTool",
                        "Group" : group}
    response = send_message(get_uf_ut_packet)
    if response is None:
      raise Exception("Error while sending get_uf_ut_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_GET_UF_UT successful") if request_successful else LOGGER.info("RMI_GET_UF_UT failed, ErrorID = " + error_str)
      return request_successful, response
  except Exception as e: print("error rmi_read_joint_angles()", e)


def rmi_read_position_register(register:int, group=1):
  time.sleep(TIME_BUFFER)
  try:
    assert(1 <= register and register <= 100), "register value is out of range"
    read_pr_packet = {"Command" : "FRC_ReadPositionRegister",
                      "RegisterNumber" : register,
                      "Group" : group}
    response = send_message(read_pr_packet)
    if response is None:
      raise Exception("Error while sending read_pr_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_READ_PR successful") if request_successful else LOGGER.info("RMI_READ_PR failed, ErrorID = " + error_str)
      return request_successful, response
  except AssertionError as ae: print(ae)
  except Exception as e: print("error rmi_read_position_register()", e)


def rmi_write_position_register(register:int, config:dict, position:dict, group=1):
  try:
    time.sleep(TIME_BUFFER)
    expected_keys = {"UToolNumber", "UFrameNumber", "Front", "Up", "Left", "Flip", "Turn4", "Turn5", "Turn6"}
    assert set(config.keys()) == expected_keys, "the config isn't correctly written ; config : " + str(config)
    expected_keys = {"X", "Y", "Z", "W", "P", "R"}
    assert set(position.keys()) == expected_keys, "the input position isn't correctly written ; position : " + str(position)
    assert(1 <= register and register <= 100), "register value is out of range"
    write_pr_packet =  {"Command" : "FRC_WritePositionRegister",
                        "RegisterNumber" : register,
                        "Configuration" : config,
                        "Position" : position,
                        "Group" : group}
    
    response = send_message(write_pr_packet)
    if response is None:
      raise Exception("Error while sending write_pr")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_WRITE_PR successful") if request_successful else LOGGER.info("RMI_WRITE_PR failed, ErrorID = " + error_str)
      return request_successful, response
  except AssertionError as ae: print(ae)
  except Exception as e: print("error rmi_write_position_register()", e)


def rmi_read_tcp_speed():
  time.sleep(TIME_BUFFER)
  try:
    read_tcp_speed_packet = {"Command" : "FRC_ReadTCPSpeed"}
    response = send_message(read_tcp_speed_packet)
    if response is None:
      raise Exception("Error while sending read_tcp_speed_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_READ_TCP_SPEED successful") if request_successful else LOGGER.info("RMI_READ_TCP_SPEED failed, ErrorID = " + error_str)
      return request_successful, response
  except Exception as e: print("error rmi_read_tcp_speed()", e)


def rmi_wait_din(port_number:int, port_value:str):
  time.sleep(TIME_BUFFER)
  try:
    assert (port_value in ["ON", "OFF"]), "Invalid port value ; port_value : " + port_value
    assert (0 <= port_number and port_number <= 512), "Invalid port number ; port_number : " + port_number
    global SEQUENCE_ID
    sequence_id = SEQUENCE_ID
    SEQUENCE_ID += 1
    wait_din_packet = {"Instruction" : "FRC_WaitDIN",
                       "SequenceID" : sequence_id,
                       "PortNumber" : port_number,
                       "portValue" : port_value}
    response = send_message(wait_din_packet)
    if response is None:
      raise Exception("Error while sending wait_din_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_WAIT_DIN successful") if request_successful else LOGGER.info("RMI_WAIT_DIN failed, ErrorID = " + error_str)
      return request_successful, response
  except AssertionError as ae: print(ae)
  except Exception as e: print("error rmi_wait_din()", e)


def rmi_set_u_frame(frame_number:int):
  time.sleep(TIME_BUFFER)
  try:
    assert(1 <= frame_number and frame_number <= 10), "frame_number value is out of range"
    global SEQUENCE_ID
    sequence_id = SEQUENCE_ID
    SEQUENCE_ID += 1
    set_u_frame_packet =  {"Instruction" : "FRC_SetUFrame",
                           "SequenceID" : sequence_id,
                           "FrameNumber" : frame_number}
    response = send_message(set_u_frame_packet)
    if response is None:
      raise Exception("Error while sending set_u_frame_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_SET_U_FRAME successful") if request_successful else LOGGER.info("RMI_SET_U_FRAME failed, ErrorID = " + error_str)
      return request_successful, response
  except AssertionError as ae: print(ae)
  except Exception as e: print("error rmi_set_u_frame()", e)


def rmi_set_u_tool(tool_number:int):
  time.sleep(TIME_BUFFER)
  try:
    assert(1 <= tool_number and tool_number <= 9), "tool_number value is out of range"
    global SEQUENCE_ID
    sequence_id = SEQUENCE_ID
    SEQUENCE_ID += 1
    set_u_tool_packet = {"Instruction" : "FRC_SetUTool",
                         "SequenceID" : sequence_id,
                         "ToolNumber" : tool_number}
    response = send_message(set_u_tool_packet)
    if response is None:
      raise Exception("Error while sending set_u_tool_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_SET_U_TOOL successful") if request_successful else LOGGER.info("RMI_SET_U_TOOL failed, ErrorID = " + error_str)
      return request_successful, response
  except AssertionError as ae: print(ae)
  except Exception as e: print("error rmi_set_u_tool()", e)


def rmi_wait_time(waiting_time:float):
  time.sleep(TIME_BUFFER)
  try:
    assert(0 <= waiting_time), "waiting_time is not positive"
    global SEQUENCE_ID
    sequence_id = SEQUENCE_ID
    SEQUENCE_ID += 1
    wait_time_packet = {"Instruction" : "FRC_WaitTime",
                        "SequenceID" : sequence_id,
                        "Time" : waiting_time}
    response = send_message(wait_time_packet)
    if response is None:
      raise Exception("Error while sending wait_time_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_WAIT_TIME successful") if request_successful else LOGGER.info("RMI_WAIT_TIME failed, ErrorID = " + error_str)
      return request_successful, response
  except AssertionError as ae: print(ae)
  except Exception as e: print("error rmi_wait_time()", e)


def rmi_set_payload(schedule_number:int):
  time.sleep(TIME_BUFFER)
  try:
    assert(0 <= schedule_number and 1 != 1), "schedule_number is out of range"
    global SEQUENCE_ID
    sequence_id = SEQUENCE_ID
    SEQUENCE_ID += 1
    set_pay_load_packet = {"Instruction" : "FRC_SetPayLoad",
                           "SequenceID" : sequence_id,
                           "ScheduleNumber" : schedule_number}
    response = send_message(set_pay_load_packet)
    if response is None:
      raise Exception("Error while sending set_pay_load_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_SET_PAYLOAD successful") if request_successful else LOGGER.info("RMI_SET_PAYLOAD failed, ErrorID = " + error_str)
      return request_successful, response
  except AssertionError as ae: print(ae)
  except Exception as e: print("error rmi_set_payload()", e)


def rmi_call(program_name:str):
  time.sleep(TIME_BUFFER)
  try:
    global SEQUENCE_ID
    sequence_id = SEQUENCE_ID
    SEQUENCE_ID += 1
    call_packet = {"Instruction" : "FRC_Call",
                   "SequenceID" : sequence_id,
                   "ProgramName" : program_name}
    response = send_message(call_packet)
    if response is None:
      raise Exception("Error while sending call_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_CALL successful") if request_successful else LOGGER.info("RMI_CALL failed, ErrorID = " + error_str)
      return request_successful, response
  except Exception as e: print("error rmi_call()", e)


def rmi_linear_motion(config:dict, position:dict, speed_type:str, speed, term_type:str, term_value):
  time.sleep(TIME_BUFFER)
  try:
    expected_keys = {"UToolNumber", "UFrameNumber", "Front", "Up", "Left", "Flip", "Turn4", "Turn5", "Turn6"}
    assert set(config.keys()) == expected_keys, "the config isn't correctly written ; config : " + str(config)
    expected_keys = {"X", "Y", "Z", "W", "P", "R", "Ext1", "Ext2", "Ext3"}
    assert set(position.keys()) == expected_keys, "the input position isn't correctly written ; position : " + str(position)
    expected_speed_type = ["mmSec", "Time", "mSec"]
    assert(speed_type in expected_speed_type), "speed_type isn't correctly written ; speed_type : " + speed_type
    expected_term_type = ["FINE", "CNT", "CR"]
    assert(term_type in expected_term_type), "term_type isn't correctly written ; term_type : " + term_type
    assert(1 <= term_value and term_value <= 100), "term_value is out of range"
    
    global SEQUENCE_ID
    sequence_id = SEQUENCE_ID
    SEQUENCE_ID += 1
    linear_motion_packet = {"Instruction" : "FRC_LinearMotion",
                            "SequenceID" : sequence_id,
                            "Configuration" : config,
                            "Position" : position,
                            "SpeedType" : speed_type,
                            "Speed" : speed,
                            "TermType" : term_type,
                            "TermValue" : term_value}
    linear_motion_packet = linear_motion_packet # merge two dicts
    response = send_message(linear_motion_packet)
    if response is None:
      raise Exception("Error while sending linear_motion_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_LINEAR_MOTION successful") if request_successful else LOGGER.info("RMI_LINEAR_MOTION failed, ErrorID = " + error_str)
      return request_successful, response
  except AssertionError as ae: print(ae)
  except Exception as e: print("error rmi_linear_motion()", e)


def rmi_linear_relative(config:dict, position:dict, speed_type:str, speed, term_type:str, term_value, optionals:dict):
  time.sleep(TIME_BUFFER)
  try:
    expected_keys = {"UToolNumber", "UFrameNumber", "Front", "Up", "Left", "Flip", "Turn4", "Turn5", "Turn6"}
    assert set(config.keys()) == expected_keys, "the config isn't correctly written ; config : " + str(config)
    expected_keys_position = {"X", "Y", "Z", "W", "P", "R", "Ext1", "Ext2", "Ext3"}
    assert set(position.keys()) == expected_keys_position, "the input position isn't correctly written ; position : " + str(position)
    expected_speed_type = ["mmSec", "InchMin", "Time", "mSec"]
    assert(speed_type in expected_speed_type), "speed_type isn't correctly written ; speed_type : " + speed_type
    expected_term_type = ["FINE", "CNT", "CR"]
    assert(term_type in expected_term_type), "term_type isn't correctly written ; term_type : " + term_type
    assert(1 <= term_value and term_value <= 100), "term_value is out of range"
    
    global SEQUENCE_ID
    sequence_id = SEQUENCE_ID
    SEQUENCE_ID += 1
    linear_relative_packet = {"Instruction" : "FRC_LinearRelative",
                              "SequenceID" : sequence_id,
                              "Configuration" : config,
                              "Position" : position,
                              "SpeedType" : speed_type,
                              "Speed" : speed,
                              "TermType" : term_type,
                              "TermValue" : term_value}
    linear_relative_packet = linear_relative_packet | optionals # merge two dicts
    response = send_message(linear_relative_packet)
    if response is None:
      raise Exception("Error while sending linear_relative_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_LINEAR_RELATIVE successful") if request_successful else LOGGER.info("RMI_LINEAR_RELATIVE failed, ErrorID = " + error_str)
      return request_successful, response
  except AssertionError as ae: print(ae)
  except Exception as e: print("error rmi_linear_relative()", e)

def rmi_joint_motion(config:dict, position:dict, speed_type:str, speed, term_type:str, term_value):
  time.sleep(TIME_BUFFER)
  try:
    expected_keys = {"UToolNumber", "UFrameNumber", "Front", "Up", "Left", "Flip", "Turn4", "Turn5", "Turn6"}
    assert set(config.keys()) == expected_keys, "the config isn't correctly written ; config : " + str(config)
    expected_keys_position = {"X", "Y", "Z", "W", "P", "R", "Ext1", "Ext2", "Ext3"}
    assert set(position.keys()) == expected_keys_position, "the input position isn't correctly written ; position : " + str(position)
    expected_speed_type = ["Percent", "Time", "mSec"]
    assert(speed_type in expected_speed_type), "speed_type isn't correctly written ; speed_type : " + speed_type
    expected_term_type = ["FINE", "CNT", "CR"]
    assert(term_type in expected_term_type), "term_type isn't correctly written ; term_type : " + term_type
    assert(1 <= term_value and term_value <= 100), "term_value is out of range"
    
    global SEQUENCE_ID
    sequence_id = SEQUENCE_ID
    SEQUENCE_ID += 1
    joint_motion_packet = {"Instruction" : "FRC_JointMotion",
                              "SequenceID" : sequence_id,
                              "Configuration" : config,
                              "Position" : position,
                              "SpeedType" : speed_type,
                              "Speed" : speed,
                              "TermType" : term_type,
                              "TermValue" : term_value}
    
    joint_motion_packet = joint_motion_packet #| optionals # merge two dicts
    response = send_message(joint_motion_packet)
    if response is None:
      raise Exception("Error while sending joint_motion_packet")
    error_id = response.get("ErrorID", None)
    if error_id is None:
      raise Exception("Error while fetching ErrorID")
    else:
      request_successful = error_id == 0
      error_str = get_error_string(error_id)
      LOGGER.info("RMI_JOINT_MOTION successful") if request_successful else LOGGER.info("RMI_JOINT_MOTION failed, ErrorID = " + error_str)
      return request_successful, response
  except AssertionError as ae: print(ae)
  except Exception as e: print("error rmi_joint_motion()", e)


def test_all_functions():
  print("Run all function tests\n")

def test_print(request:Request,
               response:Response,
               next:Callable):

  try:
    #update the status
    # REDIS_CLIENT.set(STATUS_KEYSPACE, STATUS.RUNNING)
    update_status(STATUS.RUNNING)
    
    # extract the actioin uid from the body
    print("Début test_print()")
    uid = request.body.get('message')
    assert uid, 'no message defined'
    print(uid)

  finally:
    print("Fin test_print()")
    REDIS_CLIENT.set(STATUS_KEYSPACE, STATUS.READY)

def get_redis_client_from_config(redis_config:Dict):
  """build a redis client from configuration

  Args:
      redis_config (Dict): redis configuration

  Raises:
      BaseException: raise if config not conform
      BaseException: raise if error during redis client init

  Returns:
      _type_: _description_
  """
  try:
    host = redis_config['host']
    port = redis_config['port']
    database = redis_config['database']

    client = Redis(host, port, database)
    client.ping()
    
    return client

  except KeyError as error:
    missing_key = error.args[0]
    raise BaseException(['CONFIG', 'SERVER', 'REDIS'],
                        f"the redis configuration parameter {missing_key} is missing")
  except redis_exceptions.ConnectionError as error:
    raise BaseException(['CONFIG', 'SERVER', 'REDIS'],
                        f"the redis server is not reachable")


def build_validator(schemas_dict:Dict)-> Validator:
  """build the validator used to validate client request

  Args:
      schemas_dict (Dict): dictionnary of json validation schemas

  Returns:
      Validator: validator object
  """
  print("build_validator()")
  print(schemas_dict)
  # instanciate validator to validate request 
  validator = Validator()
  # add all schemas in the validation object
  for path, schema in schemas_dict.items():
    validator.add_schema(path, schema)
  
  return validator

def main(activated_server:str,
         server_config:str,
         environment_config:str,
         validation_schemas:str):

  global REDIS_CLIENT

  AMQP_CLIENT:AMQPClient = None
  HTTP_SERVER:HttpServer = None
  
  # get server configurations
  amqp_config:Dict = server_config.get('amqp')
  http_config:Dict = server_config.get('http')
  redis_config = server_config.get('redis')

  # build the redis client
  REDIS_CLIENT = get_redis_client_from_config(redis_config)

  # build the validator object
  request_validator = build_validator(validation_schemas)

  # update the status to init
  # REDIS_CLIENT.set(STATUS_KEYSPACE, STATUS.INIT)
  update_status(STATUS.INIT)

  # load mars environment
  LOGGER.info("load mars environment")
  #model.EQUIPMENT,\
  #model.REFERENCE,\
  #model.COMMAND_REGISTER,\
  #model.DB_DRIVER = mars.build_environment(environment_config)


  # if amqp server configuration is defined and if parameter activate == true
  if activated_server == 'amqp' and amqp_config:
    LOGGER.info("build amqp client")
    AMQP_CLIENT = build_AMQP_client(amqp_config)
    LOGGER.info('configure amqp client')
    AMQP_CLIENT.add_queue(label="request_report",
                          topics=__AMQP_TOPICS)
    
    '''
    # prepare a consumer pipeline
    # no topic parameter for publish => report_topic contained in the message header
    req_pipeline = CPipeline([CFunction(build_commands),
                              CFunction(AMQP_CLIENT.publish)])
    '''

    AMQP_CLIENT.add_consumer('request.rmi_library', route_request)
  
  elif activated_server == 'http' and http_config :
    # configure the server
    LOGGER.info("build http server")
    HTTP_HOST, HTTP_PORT = get_http_para_from_config(http_config)
    HTTP_SERVER = HttpServer(name="rmi_library",
                             validator=request_validator)

    # configure the endpoints
    LOGGER.info("configure http server")
    
    HTTP_SERVER.add_consumers('/rmi_library/drillings',
                             'test_print',
                             [test_print],
                             methods=['PUT'])
    

  # if no config raise an error
  if not HTTP_SERVER and not AMQP_CLIENT:
    raise BaseException(['CONFIG', 'SERVER'],
                        "no server activated, check the configuration")
  
  #update the status to ready
  REDIS_CLIENT.set(STATUS_KEYSPACE, STATUS.READY)

  # run the http or amqp com
  if HTTP_SERVER:
    LOGGER.info('run http server and wait for messages')
    # http_server run on new tread - not implemented yet
    HTTP_SERVER.run(HTTP_HOST, HTTP_PORT)

  if AMQP_CLIENT:
    # run amqp server on the current tread
    LOGGER.info('run amqp server and wait for messages')
    AMQP_CLIENT.run()


if __name__ == '__main__':
  try:
    # define log format
    LOG_FORMAT = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'

    # init argparser
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', "--verbose", action='store_true')

    # read the configurations
    LOGGER.info("load configurations")
    parser.add_argument('-s', '--server',
                        type=str,
                        choices=['amqp', 'http'],
                        default='amqp',
                        help='type of server used for communications')
    
    parser.add_argument('--server-config',
                        type=str,
                        nargs=1,
                        action=ConfigLoader,
                        help='path of server configuration yaml file')

    parser.add_argument('--environment-config',
                        type=str,
                        nargs=1,
                        action=ConfigLoader,
                        help='path of environment configuration yaml file')
    
    parser.add_argument('--validation-schemas',
                        type=str,
                        nargs=1,
                        action=ConfigLoader,
                        help='path of the directory contains schemas for requests validations')

    # update arguements
    args = parser.parse_args()
    args.validation_schemas = get_validation_schemas(__VALIDATION_SCHEMA_DIR)\
                              if not args.validation_schemas else args.validation_schemas
    args.server_config = get_config_from_file(__SERVER_CONFIG_FILE)\
                         if not args.server_config else args.server_config
    args.environment_config = get_config_from_file(__MARS_CONFIG_FILE)\
                              if not args.environment_config else args.environment_config
    
    # update the logger config
    if args.verbose:
      logging.basicConfig(level=logging.DEBUG, format=LOG_FORMAT)
    else:
      logging.basicConfig(level=logging.INFO, format=LOG_FORMAT)
    
    logging.getLogger("pika").setLevel(logging.WARNING)


    LOGGER.info("run rmi_library service")
    
    # call main function
    main(activated_server=args.server,
         server_config=args.server_config,
         environment_config=args.environment_config,
         validation_schemas=args.validation_schemas)

  except BaseException as error:
    print("ERRRRRRRRRRRRRREUR")
    LOGGER.fatal(error.describe())
    sys.exit(1)
  except KeyboardInterrupt as error:
    LOGGER.info("manual interruption of the program")
    sys.exit(1)
  except Exception as error:
    LOGGER.error(error)
    traceback.print_exc()
  finally:
    # delete the status keyspace
    if(REDIS_CLIENT):
      purge_status()
    exit(1)