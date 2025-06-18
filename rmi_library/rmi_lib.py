import logging
import json
import traceback
import sys
import socket
import time
import coloredlogs

LOG_FORMAT = "%(levelname)s - %(message)s"
coloredlogs.install(level=logging.INFO, fmt=LOG_FORMAT)
LOGGER = logging.getLogger("rmi_library")


class RMILibrary:
    def __init__(
        self, robot_ip="192.168.0.10", robot_port=16001
    ):  # ROBOT REEL = "192.168.0.104" / ROBOGUIDE = "192.168.0.10"
        self.ROBOT_IP = robot_ip
        self.ROBOT_PORT = robot_port
        self.sock = None
        self.TIME_BUFFER = 0.005  # waiting time between each instruction
        self.SEQUENCE_ID = -1
        self.global_verbose = True

        self.ErrorID_to_str = {
            2556932: "Invalid Position Register (2556932)",
            2556936: "Cannot Execute TP program (2556936)",
            2556937: "RMI is Not Running (2556937)",
            2556941: "Invalid RMI Command (2556941)",
            2556943: "Invalid Controller State (2556943)",
            2556950: "Invalid Text String (2556950)",
            2556957: "Invalid sequence ID (2556957)",
            2556977: "Invalid Instruction packet (2556977)",
            2556954: "Robot is Already Connected (2556954)",
            2556971: "Robot in Single Step Mode (2556971)",
        }

        self.startup_sequence(verbose=True)

    def is_socket_active(self, verbose=True):
        """Check if the socket is currently active"""
        if self.sock:
            try:
                self.sock.send(b"")
                return True
            except (socket.error, BrokenPipeError):
                pass
        if verbose:
            LOGGER.error("RMI socket is not active")
        return False

    def get_error_string(self, error_code: int):
        error_str = (
            self.ErrorID_to_str[error_code] if error_code in self.ErrorID_to_str else "Error ID : " + str(error_code)
        )
        return error_str

    def quick_test(self):
        try:
            self.global_verbose = True

            config = {
                "UToolNumber": 7,
                "UFrameNumber": 0,
                "Front": 1,
                "Up": 1,
                "Left": 0,
                "Flip": 1,
                "Turn4": 0,
                "Turn5": 0,
                "Turn6": 0,
            }

            while True:
                _, current_position = self.rmi_read_cartesian_position()
                print(current_position)
                config = current_position["Configuration"]
                self.rmi_set_u_tool(config["UToolNumber"])
                self.rmi_set_u_frame(config["UFrameNumber"])
                current_position = current_position["Position"]
                updated_position = current_position.copy()
                updated_position.update({"Z": current_position["Z"] + 5})
                _, response = self.rmi_linear_motion(config, updated_position, "mmSec", 5, "FINE", 15)
                logging.error(response)
                input("Press Enter to continue...")

        except Exception:
            LOGGER.error(traceback.format_exc())

    def send_message(self, packet, verbose=True):
        verbose_ = verbose if self.global_verbose else False
        try:
            self.sock.sendall((json.dumps(packet) + "\r\n").encode("utf-8"))  # send message
            response = self.sock.recv(1024).decode("utf-8")  # decode response
            response_data = json.loads(response)
            if verbose_:
                LOGGER.info(f"REPONSE RECUE: {response_data}")
            return response_data
        except Exception:
            pass

    def rmi_connect(self, verbose=True):
        time.sleep(self.TIME_BUFFER)
        try:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((self.ROBOT_IP, self.ROBOT_PORT))
            except Exception as e:
                LOGGER.error(f"Socket connection error: {e}")
                return False, None

            connect_packet = {"Communication": "FRC_Connect"}
            response = self.send_message(packet=connect_packet, verbose=verbose)
            if response is None:
                raise Exception("Failed to send connect_packet. No response received.")

            error_id = response.get("ErrorID")
            if error_id is None:
                raise Exception("ErrorID is missing in the response.")

            is_connected = (error_id == 0) or (error_id == 2556954)  # 2556954 = "Robot is Already Connected"

            new_port = response.get("PortNumber")
            if new_port is None:
                raise Exception("PortNumber is missing in the response. Unable to proceed.")

            self.sock.close()  # Close the socket if it's already connected
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.ROBOT_IP, new_port))

            if is_connected:
                LOGGER.warning("RMI connection successful.")
            else:
                if verbose:
                    error_str = self.get_error_string(error_id)
                    LOGGER.error(f"RMI connection failed: {error_str}")

            return is_connected, response

        except Exception as e:
            if verbose:
                LOGGER.error(f"RMI connection error: {str(e)}")
            is_connected = False
            return is_connected, e

    def rmi_disconnect(self):
        time.sleep(self.TIME_BUFFER)
        try:
            disconnect_packet = {"Communication": "FRC_Disconnect"}
            response = self.send_message(packet=disconnect_packet)
            if response is None:
                raise Exception("Error while sending disconnect_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                # if ErrorID == 0, the service is disconnected from robot
                is_disconnected = error_id == 0
                error_str = self.get_error_string(error_id)
                if is_disconnected:
                    self.sock.close()
                    LOGGER.warning("RMI_DISCONNECT successful")
                else:
                    LOGGER.error(f"RMI_DISCONNECT failed: {error_str}")
                return is_disconnected, response
        except Exception as e:
            LOGGER.error(f"error rmi_disconnect(): {str(e)}")

    def rmi_get_status(self, verbose=True):
        time.sleep(self.TIME_BUFFER)
        try:
            get_status_packet = {"Command": "FRC_GetStatus"}
            response = self.send_message(packet=get_status_packet)
            if response is None:
                raise Exception("Error while sending get_status_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                if verbose:
                    error_str = self.get_error_string(error_id)
                    (
                        LOGGER.warning("RMI_GETSTATUS successful")
                        if request_successful
                        else LOGGER.error(f"RMI_GETSTATUS failed: {error_str}")
                    )
                return request_successful, response
        except Exception as e:
            LOGGER.error(f"error rmi_get_status(): {e}")

    def is_robot_available_to_initialize(self, data):
        if data.get("ErrorID", -1) != 0:
            LOGGER.error("Error while fetching robot status")
        else:
            servo_ready = data.get("ServoReady", None)
            tp_mode = data.get("TPMode", None)

            if servo_ready is not None and tp_mode is not None:
                is_available = servo_ready == 1 and tp_mode == 0
                return is_available
            else:
                LOGGER.error("Error while reading ServoReady or TPMode")
                return False

    def rmi_initialize(self):
        time.sleep(self.TIME_BUFFER)
        try:
            initialize_packet = {"Command": "FRC_Initialize"}
            response = self.send_message(initialize_packet)
            if response is None:
                raise Exception("Error while sending initialize_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                # if ErrorID == 0, rmi is initialized
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_INITIALIZE successful")
                    if request_successful
                    else LOGGER.error(f"RMI_INITIALIZE failed: {error_str}")
                )
                return request_successful, response
        except Exception as e:
            LOGGER.error("error rmi_initialize()", e)

    def rmi_abort(self):
        time.sleep(self.TIME_BUFFER)
        try:
            abort_packet = {"Command": "FRC_Abort"}
            response = self.send_message(packet=abort_packet)
            if response is None:
                raise Exception("Error while sending abort_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                if request_successful:
                    LOGGER.warning("RMI_ABORT successful")
                else:
                    LOGGER.error(f"RMI_ABORT failed: {error_str}")
                return request_successful
        except Exception as e:
            LOGGER.error(f"error rmi_abort(): {e}")
            return False

    def rmi_reset(self):
        time.sleep(self.TIME_BUFFER)
        try:
            reset_packet = {"Command": "FRC_Reset"}
            response = self.send_message(packet=reset_packet)
            if response is None:
                raise Exception("Error while sending reset_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_RESET successful")
                    if request_successful
                    else LOGGER.error(f"RMI_RESET failed: {error_str}")
                )
                return response
        except Exception as e:
            LOGGER.error(f"error rmi_reset(): {e}")
            return None

    def rmi_pause(self):
        time.sleep(self.TIME_BUFFER)
        try:
            pause_packet = {"Command": "FRC_Pause"}
            response = self.send_message(packet=pause_packet)
            if response is None:
                raise Exception("Error while sending pause_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_PAUSE successful")
                    if request_successful
                    else LOGGER.error(f"RMI_PAUSE failed: {error_str}")
                )
                return response
        except Exception as e:
            LOGGER.error(f"error rmi_pause(): {e}")

    def rmi_continue(self):
        time.sleep(self.TIME_BUFFER)
        try:
            continue_packet = {"Command": "FRC_Continue"}
            response = self.send_message(packet=continue_packet)
            if response is None:
                raise Exception("Error while sending continue_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.et_error_string(error_id)
                (
                    LOGGER.warning("RMI_CONTINUE successful")
                    if request_successful
                    else LOGGER.error(f"RMI_CONTINUE failed: {error_str}")
                )
                return response
        except Exception as e:
            LOGGER.error(f"error rmi_continue(): {e}")

    def rmi_read_error(self):
        time.sleep(self.TIME_BUFFER)
        try:
            read_error_packet = {"Command": "FRC_ReadError"}
            response = self.send_message(packet=read_error_packet)
            return response
        except Exception as e:
            LOGGER.error(f"error read_error(): {e}")

    def startup_sequence(self, verbose=True, max_attempts=10) -> bool:
        attempt = 0
        while attempt < max_attempts:
            try:
                success, _ = self.rmi_connect(verbose=verbose)
                self.is_rmi_running = success
                if not success:
                    attempt += 1
                    continue

                if not self.rmi_reset():
                    attempt += 1
                    continue

                _, status = self.rmi_get_status()
                self.SEQUENCE_ID = 1
                is_status_ok = self.is_robot_available_to_initialize(status)
                if not is_status_ok:
                    LOGGER.warning("waiting for ServoReady = 1 and TPMODE = 0...")

                while not is_status_ok:
                    time.sleep(1)
                    LOGGER.error("Robot status is not OK, retrying initialization...")
                    _, status = self.rmi_get_status(False)
                    is_status_ok = self.is_robot_available_to_initialize(status)

                success, _ = self.rmi_initialize()
                if not success:
                    attempt += 1
                    continue

                LOGGER.warning("Robot ready for initialization")
                return True

            except Exception as e:
                LOGGER.error(f"error in startup_sequence() attempt {attempt+1}: {e}")
                attempt += 1
                continue

        LOGGER.error(f"Connection failed after {max_attempts} attempts.")
        return False

    def rmi_set_uf_ut(self, uf: int, ut: int, group=1):
        try:
            time.sleep(self.TIME_BUFFER)
            assert 0 <= uf and uf <= 255 and 0 <= ut and ut <= 255, "uf or ut is out of range"
            set_uf_ut_packet = {"Command": "FRC_SetUFrameUTool", "UFrameNumber": uf, "UToolNumber": ut, "Group": group}
            response = self.send_message(packet=set_uf_ut_packet)
            if response is None:
                raise Exception("Error while sending set_uf_ut_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_SET_UF_UT successful")
                    if request_successful
                    else LOGGER.error(f"RMI_SET_UF_UT failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_set_uf_ut(): {e}")

    def rmi_read_uf_data(self, uf: int, group=1):
        try:
            time.sleep(self.TIME_BUFFER)
            assert 0 <= uf and uf <= 255, "uf number is out of range"
            read_uf_data_packet = {"Command": "FRC_ReadUFrameData", "FrameNumber": uf, "Group": group}
            response = self.send_message(packet=read_uf_data_packet)
            if response is None:
                raise Exception("Error while sending read_uf_data_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_READ_UF_DATA successful")
                    if request_successful
                    else LOGGER.error(f"RMI_READ_UF_DATA failed: {error_str}")
                )
                return response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_read_uf_data(): {e}")

    def rmi_write_uf_data(self, uf_number: int, frame: dict, group=1):
        try:
            time.sleep(self.TIME_BUFFER)
            expected_keys = {"X", "Y", "Z", "W", "P", "R"}
            assert set(frame.keys()) == expected_keys, "the input frame isn't correctly written ; frame : " + str(frame)
            assert 0 <= uf_number and uf_number <= 255, "uf number is out of range"
            write_uf_data_packet = {
                "Command": "FRC_WriteUFrameData",
                "FrameNumber": uf_number,
                "Frame": frame,
                "Group": group,
            }
            response = self.send_message(packet=write_uf_data_packet)
            if response is None:
                raise Exception("Error while sending write_uf_data_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_WRITE_UF_DATA successful")
                    if request_successful
                    else LOGGER.error(f"RMI_WRITE_UF_DATA failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_write_uf_data(): {e}")

    def rmi_read_ut_data(self, ut: int, group=1):
        try:
            time.sleep(self.TIME_BUFFER)
            assert 0 <= ut and ut <= 255, "ut number is out of range"
            read_ut_data_packet = {"Command": "FRC_ReadUToolData", "ToolNumber": ut, "Group": group}
            response = self.send_message(packet=read_ut_data_packet)
            if response is None:
                raise Exception("Error while sending read_ut_data_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_READ_UT_DATA successful")
                    if request_successful
                    else LOGGER.error(f"RMI_READ_UT_DATA failed: {error_str}")
                )
                return response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_read_ut_data(): {e}")

    def rmi_write_ut_data(self, ut_number: int, frame: dict, group=1):
        try:
            time.sleep(self.TIME_BUFFER)
            expected_keys = {"X", "Y", "Z", "W", "P", "R"}
            assert set(frame.keys()) == expected_keys, "the input frame isn't correctly written ; frame : " + str(frame)
            assert 0 <= ut_number and ut_number <= 255, "ut number is out of range"
            write_ut_data_packet = {
                "Command": "FRC_WriteUToolData",
                "ToolNumber": ut_number,
                "Frame": frame,
                "Group": group,
            }
            response = self.send_message(packet=write_ut_data_packet)
            if response is None:
                raise Exception("Error while sending write_ut_data_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_WRITE_UT_DATA successful")
                    if request_successful
                    else LOGGER.error(f"RMI_WRITE_UT_DATA failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_write_ut_data(): {e}")

    def rmi_read_cartesian_position(self, group=1):
        time.sleep(self.TIME_BUFFER)
        try:
            read_cartesian_position_packet = {"Command": "FRC_ReadCartesianPosition", "Group": group}
            response = self.send_message(packet=read_cartesian_position_packet)
            if response is None:
                raise Exception("Error while sending read_cartesian_position_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_READ_CARTESIAN_POSITION successful")
                    if request_successful
                    else LOGGER.error(f"RMI_READ_CARTESIAN_POSITION failed: {error_str}")
                )
                return request_successful, response
        except Exception as e:
            LOGGER.error(f"Error in rmi_read_cartesian_position(): {e}")

    def rmi_read_joint_angles(self, group=1):
        time.sleep(self.TIME_BUFFER)
        try:
            read_joint_angles_packet = {"Command": "FRC_ReadJointAngles", "Group": group}
            response = self.send_message(packet=read_joint_angles_packet)
            if response is None:
                raise Exception("Error while sending read_joint_angles_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_READ_CARTESIAN_POSITION successful")
                    if request_successful
                    else LOGGER.error(f"RMI_READ_CARTESIAN_POSITION failed: {error_str}")
                )
                return request_successful, response
        except Exception as e:
            LOGGER.error(f"error rmi_read_joint_angles(): {e}")

    def rmi_set_override(self, value: int):
        time.sleep(self.TIME_BUFFER)
        try:
            assert 1 <= value and value <= 100, "override value is out of range"
            set_override_packet = {"Command": "FRC_SetOverRide", "Value": value}
            response = self.send_message(packet=set_override_packet)
            if response is None:
                raise Exception("Error while sending set_override_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_SET_OVERRIDE successful")
                    if request_successful
                    else LOGGER.error(f"RMI_SET_OVERRIDE failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_set_override(): {e}")

    def rmi_get_uf_ut(self, group=1):
        time.sleep(self.TIME_BUFFER)
        try:
            get_uf_ut_packet = {"Command": "FRC_GetUFrameUTool", "Group": group}
            response = self.send_message(packet=get_uf_ut_packet)
            if response is None:
                raise Exception("Error while sending get_uf_ut_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_GET_UF_UT successful")
                    if request_successful
                    else LOGGER.error(f"RMI_GET_UF_UT failed: {error_str}")
                )
                return request_successful, response
        except Exception as e:
            LOGGER.error(f"error rmi_get_uf_ut(): {e}")

    def rmi_read_position_register(self, register: int, group=1):
        time.sleep(self.TIME_BUFFER)
        try:
            assert 1 <= register and register <= 100, "register value is out of range"
            read_pr_packet = {"Command": "FRC_ReadPositionRegister", "RegisterNumber": register, "Group": group}
            response = self.send_message(packet=read_pr_packet)
            if response is None:
                raise Exception("Error while sending read_pr_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_READ_PR successful")
                    if request_successful
                    else LOGGER.error(f"RMI_READ_PR failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_read_position_register(): {e}")

    def rmi_write_position_register(self, register: int, config: dict, position: dict, group=1):
        try:
            time.sleep(self.TIME_BUFFER)
            expected_keys = {"UToolNumber", "UFrameNumber", "Front", "Up", "Left", "Flip", "Turn4", "Turn5", "Turn6"}
            assert set(config.keys()) == expected_keys, "the config isn't correctly written ; config : " + str(config)
            expected_keys = {"X", "Y", "Z", "W", "P", "R"}
            assert (
                set(position.keys()) == expected_keys
            ), "the input position isn't correctly written ; position : " + str(position)
            assert 1 <= register and register <= 100, "register value is out of range"
            write_pr_packet = {
                "Command": "FRC_WritePositionRegister",
                "RegisterNumber": register,
                "Configuration": config,
                "Position": position,
                "Group": group,
            }
            response = self.send_message(packet=write_pr_packet)
            if response is None:
                raise Exception("Error while sending write_pr")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_WRITE_PR successful")
                    if request_successful
                    else LOGGER.error(f"RMI_WRITE_PR failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_write_position_register(): {e}")

    def rmi_read_tcp_speed(self):
        time.sleep(self.TIME_BUFFER)
        try:
            read_tcp_speed_packet = {"Command": "FRC_ReadTCPSpeed"}
            response = self.send_message(packet=read_tcp_speed_packet)
            if response is None:
                raise Exception("Error while sending read_tcp_speed_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_READ_TCP_SPEED successful")
                    if request_successful
                    else LOGGER.error(f"RMI_READ_TCP_SPEED failed: {error_str}")
                )
                return request_successful, response
        except Exception as e:
            LOGGER.error(f"error rmi_read_tcp_speed(): {e}")

    def rmi_wait_din(self, port_number: int, port_value: str):
        time.sleep(self.TIME_BUFFER)
        try:
            assert port_value in ["ON", "OFF"], "Invalid port value ; port_value : " + port_value
            assert 0 <= port_number and port_number <= 512, "Invalid port number ; port_number : " + port_number
            sequence_id = self.SEQUENCE_ID
            self.SEQUENCE_ID += 1
            wait_din_packet = {
                "Instruction": "FRC_WaitDIN",
                "SequenceID": sequence_id,
                "PortNumber": port_number,
                "portValue": port_value,
            }
            response = self.send_message(packet=wait_din_packet)
            if response is None:
                raise Exception("Error while sending wait_din_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_WAIT_DIN successful")
                    if request_successful
                    else LOGGER.error(f"RMI_WAIT_DIN failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_wait_din(): {e}")

    def rmi_set_u_frame(self, frame_number: int):
        time.sleep(self.TIME_BUFFER)
        try:
            assert 0 <= frame_number and frame_number <= 10, "frame_number value is out of range"
            sequence_id = self.SEQUENCE_ID
            self.SEQUENCE_ID += 1
            set_u_frame_packet = {
                "Instruction": "FRC_SetUFrame",
                "SequenceID": sequence_id,
                "FrameNumber": frame_number,
            }
            response = self.send_message(packet=set_u_frame_packet)
            if response is None:
                raise Exception("Error while sending set_u_frame_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_SET_U_FRAME successful")
                    if request_successful
                    else LOGGER.error(f"RMI_SET_U_FRAME failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_set_u_frame(): {e}")

    def rmi_set_u_tool(self, tool_number: int):
        time.sleep(self.TIME_BUFFER)
        try:
            assert 1 <= tool_number and tool_number <= 9, "tool_number value is out of range"
            sequence_id = self.SEQUENCE_ID
            self.SEQUENCE_ID += 1
            set_u_tool_packet = {"Instruction": "FRC_SetUTool", "SequenceID": sequence_id, "ToolNumber": tool_number}
            response = self.send_message(packet=set_u_tool_packet)
            if response is None:
                raise Exception("Error while sending set_u_tool_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_SET_U_TOOL successful")
                    if request_successful
                    else LOGGER.error(f"RMI_SET_U_TOOL failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_set_u_tool(): {e}")

    def rmi_wait_time(self, waiting_time: float):
        time.sleep(self.TIME_BUFFER)
        try:
            assert 0 <= waiting_time, "waiting_time is not positive"
            sequence_id = self.SEQUENCE_ID
            self.SEQUENCE_ID += 1
            wait_time_packet = {"Instruction": "FRC_WaitTime", "SequenceID": sequence_id, "Time": waiting_time}
            response = self.send_message(packet=wait_time_packet)
            if response is None:
                raise Exception("Error while sending wait_time_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_WAIT_TIME successful")
                    if request_successful
                    else LOGGER.error(f"RMI_WAIT_TIME failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_wait_time(): {e}")

    def rmi_set_payload(self, schedule_number: int):
        time.sleep(self.TIME_BUFFER)
        try:
            assert 0 <= schedule_number and 1 != 1, "schedule_number is out of range"
            sequence_id = self.SEQUENCE_ID
            self.SEQUENCE_ID += 1
            set_pay_load_packet = {
                "Instruction": "FRC_SetPayLoad",
                "SequenceID": sequence_id,
                "ScheduleNumber": schedule_number,
            }
            response = self.send_message(packet=set_pay_load_packet)
            if response is None:
                raise Exception("Error while sending set_pay_load_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_SET_PAYLOAD successful")
                    if request_successful
                    else LOGGER.error(f"RMI_SET_PAYLOAD failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_set_payload(): {e}")

    def rmi_call(self, program_name: str):
        time.sleep(self.TIME_BUFFER)
        try:
            if not program_name or not isinstance(program_name, str):
                raise ValueError("Invalid program_name. It must be a non-empty string.")
            sequence_id = self.SEQUENCE_ID
            self.SEQUENCE_ID += 1
            call_packet = {"Instruction": "FRC_Call", "SequenceID": sequence_id, "ProgramName": program_name}
            response = self.send_message(packet=call_packet)
            if response is None:
                raise Exception("Error while sending call_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_CALL successful")
                    if request_successful
                    else LOGGER.error(f"RMI_CALL failed: {error_str}")
                )
                return request_successful, response
        except Exception as e:
            LOGGER.error(f"error rmi_call(): {e}")

    def rmi_linear_motion(self, config: dict, position: dict, speed_type: str, speed, term_type: str, term_value):
        time.sleep(self.TIME_BUFFER)
        try:
            expected_keys = {"UToolNumber", "UFrameNumber", "Front", "Up", "Left", "Flip", "Turn4", "Turn5", "Turn6"}
            assert set(config.keys()) == expected_keys, "the config isn't correctly written ; config : " + str(config)
            expected_keys = {"X", "Y", "Z", "W", "P", "R", "Ext1", "Ext2", "Ext3"}
            assert (
                set(position.keys()) == expected_keys
            ), "the input position isn't correctly written ; position : " + str(position)
            expected_speed_type = ["mmSec", "Time", "mSec"]
            assert speed_type in expected_speed_type, "speed_type isn't correctly written ; speed_type : " + speed_type
            expected_term_type = ["FINE", "CNT", "CR"]
            assert term_type in expected_term_type, "term_type isn't correctly written ; term_type : " + term_type
            assert 1 <= term_value and term_value <= 100, "term_value is out of range"

            sequence_id = self.SEQUENCE_ID
            self.SEQUENCE_ID += 1
            linear_motion_packet = {
                "Instruction": "FRC_LinearMotion",
                "SequenceID": sequence_id,
                "Configuration": config,
                "Position": position,
                "SpeedType": speed_type,
                "Speed": speed,
                "TermType": term_type,
                "TermValue": term_value,
            }
            linear_motion_packet = linear_motion_packet  # merge two dicts
            response = self.send_message(packet=linear_motion_packet)
            if response is None:
                raise Exception("Error while sending linear_motion_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_LINEAR_MOTION successful")
                    if request_successful
                    else LOGGER.error(f"RMI_LINEAR_MOTION failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_linear_motion(): {e}")

    def rmi_linear_relative(
        self, config: dict, position: dict, speed_type: str, speed, term_type: str, term_value, optionals: dict = {}
    ):
        time.sleep(self.TIME_BUFFER)
        try:
            expected_keys = {"UToolNumber", "UFrameNumber", "Front", "Up", "Left", "Flip", "Turn4", "Turn5", "Turn6"}
            assert set(config.keys()) == expected_keys, "the config isn't correctly written ; config : " + str(config)
            expected_keys_position = {"X", "Y", "Z", "W", "P", "R", "Ext1", "Ext2", "Ext3"}
            assert (
                set(position.keys()) == expected_keys_position
            ), "the input position isn't correctly written ; position : " + str(position)
            expected_speed_type = ["mmSec", "InchMin", "Time", "mSec"]
            assert speed_type in expected_speed_type, "speed_type isn't correctly written ; speed_type : " + speed_type
            expected_term_type = ["FINE", "CNT", "CR"]
            assert term_type in expected_term_type, "term_type isn't correctly written ; term_type : " + term_type
            assert 1 <= term_value and term_value <= 100, "term_value is out of range"
            sequence_id = self.SEQUENCE_ID
            self.SEQUENCE_ID += 1
            linear_relative_packet = {
                "Instruction": "FRC_LinearRelative",
                "SequenceID": sequence_id,
                "Configuration": config,
                "Position": position,
                "SpeedType": speed_type,
                "Speed": speed,
                "TermType": term_type,
                "TermValue": term_value,
            }
            linear_relative_packet = linear_relative_packet | optionals  # merge two dicts
            response = self.send_message(packet=linear_relative_packet)
            if response is None:
                raise Exception("Error while sending linear_relative_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_LINEAR_RELATIVE successful")
                    if request_successful
                    else LOGGER.error(f"RMI_LINEAR_RELATIVE failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_linear_relative(): {e}")

    def rmi_joint_motion(self, config: dict, position: dict, speed_type: str, speed, term_type: str, term_value):
        time.sleep(self.TIME_BUFFER)
        try:
            expected_keys = {"UToolNumber", "UFrameNumber", "Front", "Up", "Left", "Flip", "Turn4", "Turn5", "Turn6"}
            assert set(config.keys()) == expected_keys, "the config isn't correctly written ; config : " + str(config)
            expected_keys_position = {"X", "Y", "Z", "W", "P", "R", "Ext1", "Ext2", "Ext3"}
            assert (
                set(position.keys()) == expected_keys_position
            ), "the input position isn't correctly written ; position : " + str(position)
            expected_speed_type = ["Percent", "Time", "mSec"]
            assert speed_type in expected_speed_type, "speed_type isn't correctly written ; speed_type : " + speed_type
            expected_term_type = ["FINE", "CNT", "CR"]
            assert term_type in expected_term_type, "term_type isn't correctly written ; term_type : " + term_type
            assert 1 <= term_value and term_value <= 100, "term_value is out of range"

            sequence_id = self.SEQUENCE_ID
            self.SEQUENCE_ID += 1
            joint_motion_packet = {
                "Instruction": "FRC_JointMotion",
                "SequenceID": sequence_id,
                "Configuration": config,
                "Position": position,
                "SpeedType": speed_type,
                "Speed": speed,
                "TermType": term_type,
                "TermValue": term_value,
            }

            joint_motion_packet = joint_motion_packet  # | optionals # merge two dicts
            response = self.send_message(packet=joint_motion_packet)
            if response is None:
                raise Exception("Error while sending joint_motion_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_JOINT_MOTION successful")
                    if request_successful
                    else LOGGER.error(f"RMI_JOINT_MOTION failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_joint_motion(): {e}")

    def rmi_write_d_out(self, port_number: int, port_value: str, group=1):
        try:
            time.sleep(self.TIME_BUFFER)
            assert port_number >= 0 or port_number <= 2048, "DOUT port number is out of range"
            assert port_value in ["ON", "OFF"], "DOUT value must be ON or OFF"
            write_d_out_data_packet = {"Command": "FRC_WriteDOUT", "PortNumber": port_number, "PortValue": port_value}
            response = self.send_message(packet=write_d_out_data_packet)

            if response is None:
                raise Exception("Error while sending write_d_out_data_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_WRITE_D_OUT successful")
                    if request_successful
                    else LOGGER.error(f"RMI_WRITE_D_OUT failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_write_d_out(): {e}")

    def rmi_read_d_in(self, port_number: int, group=1):
        try:
            time.sleep(self.TIME_BUFFER)
            assert port_number >= 0 or port_number <= 2048, "DIN port number is out of range"
            write_d_in_data_packet = {"Command": "FRC_ReadDIN", "PortNumber": port_number}
            response = self.send_message(packet=write_d_in_data_packet)

            if response is None:
                raise Exception("Error while sending write_d_in_data_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.warning("RMI_WRITE_D_IN successful")
                    if request_successful
                    else LOGGER.error(f"RMI_WRITE_D_IN failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_write_d_in(): {e}")


if __name__ == "__main__":
    try:
        # define log format
        LOG_FORMAT = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"

        logging.basicConfig(
            level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s", handlers=[logging.StreamHandler()]
        )

        LOGGER.info("run rmi_library service")
        test_object = RMILibrary()
        test_object.quick_test()

    except BaseException as error:
        LOGGER.fatal(error.describe())
        sys.exit(1)
    except KeyboardInterrupt as error:
        LOGGER.info("manual interruption of the program")
        sys.exit(1)
    except Exception as error:
        LOGGER.error(f"Error in rmi_library - main: {error}")
        traceback.LOGGER.error_exc()
    finally:
        exit(1)
