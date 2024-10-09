from multiprocessing.connection import wait
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
#import model
#from model.action import Action
#from model.definition import Drilling
from redis import Redis, exceptions as redis_exceptions
#import mars
import json
import traceback

import sys
import os
# add .src in path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname("."), '../CRX_RMI_library/src')))

# import modules
import socket
import time

load_dotenv()

__VALIDATION_SCHEMA_DIR = './schemas'
__SERVER_CONFIG_FILE = './config/server.yaml'
__MARS_CONFIG_FILE = './config/mars.yaml'

# declare amqp topics
__AMQP_TOPICS = 'request.rmi_library','report.rmi_library'

# declare redis constants
# for status
STATUS_KEYSPACE = 'mars.rmi_library.status'
REDIS_CLIENT: Redis = None

LOGGER = logging.getLogger("rmi_library")

ROBOT_IP = "192.168.1.10"
ROBOT_PORT = 16001
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

ErrorID_to_str = {
  2556941 : "Invalid RMI Command (2556941)"
}

TIME_BUFFER = 0.005 #time to wait between each instruction

def get_error_string(error_code:int):
  global ErrorID_to_str
  error_str = ErrorID_to_str[error_code] if error_code in ErrorID_to_str else "empty"
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
        
        init_rmi_connection()
      except Exception as e:
        print(e)
  else:
    # >LOG
    print('not conform path')

def send_message(packet):
  type(packet)
  try:
    #send the message
    sock.sendall((json.dumps(packet) + "\r\n").encode('utf-8'))
    
    #decode the response
    response = sock.recv(1024).decode('utf-8')
    response_data = json.loads(response)
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
      #if ErrorID == 0, the service is connected to robot
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
      #if ErrorID == 0, the service is disconnected from robot
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
      error_str = get_error_string(error_id)
      if verbose:
        LOGGER.info("RMI_GETSTATUS successful") if request_successful else LOGGER.info("RMI_GETSTATUS failed, ErrorID = " + error_str)
      return request_successful, response
  except Exception as e: print("error rmi_get_status()", e)


def is_robot_available_to_initialize(data):
  if data.get("ErrorID", -1) != 0:
    print("Error while fetching robot status")
  else:
    servo_ready = data.get("ServoReady", None)
    tp_mode = data.get("TPMode", None)
    
    if servo_ready is not None and tp_mode is not None:
      is_available = servo_ready == 1 and tp_mode == 1
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
    return response
  except Exception as e: print("error rmi_abort()", e)

def rmi_pause():
  time.sleep(TIME_BUFFER)
  try:
    pause_packet = {"Command" : "FRC_Pause"}
    response = send_message(pause_packet)
    return response
  except Exception as e: print("error rmi_pause()", e)

def rmi_continue():
  time.sleep(TIME_BUFFER)
  try:
    continue_packet = {"Command" : "FRC_Continue"}
    response = send_message(continue_packet)
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
    _, status = rmi_get_status()
    is_status_ok = is_robot_available_to_initialize(status)
    LOGGER.info("Robot ready for initialization") if is_status_ok else LOGGER.info("waiting for ServoReady = 1 and TPMODE = 1...")
    while not is_status_ok:
      time.sleep(1)
      _, status = rmi_get_status(False)
      is_status_ok = is_robot_available_to_initialize(status)
    print("Robot status is OK, initialization...")
    _, _ = rmi_initialize()
  except Exception as e: print("error init_rmi_connection()", e)

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