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

        # Full RMI error ID reference table (B-84464EN-12/01, MajorVersion 9).
        # ErrorID = 2556928 + RMIT number.
        self.ErrorID_to_str = {
            2556929: "Internal System Error (RMIT-001)",
            2556930: "Invalid UTool Number (RMIT-002)",
            2556931: "Invalid UFrame Number (RMIT-003)",
            2556932: "Invalid Position Register (RMIT-004)",
            2556933: "Invalid Speed Override (RMIT-005)",
            2556934: "Cannot Execute TP program (RMIT-006)",
            2556935: "Controller Servo is Off (RMIT-007)",
            2556936: "Teach Pendant is Enabled (RMIT-008)",
            2556937: "RMI is Not Running (RMIT-009)",
            2556938: "TP Program is Not Paused (RMIT-010)",
            2556939: "Cannot Resume TP Program (RMIT-011)",
            2556940: "Cannot Reset Controller (RMIT-012)",
            2556941: "Invalid RMI Command (RMIT-013)",
            2556942: "RMI Command Fail (RMIT-014)",
            2556943: "Invalid Controller State (RMIT-015)",
            2556944: "Please Cycle Power (RMIT-016)",
            2556945: "Invalid Payload Schedule (RMIT-017)",
            2556946: "Invalid Motion Option (RMIT-018)",
            2556947: "Invalid Vision Register (RMIT-019)",
            2556948: "Invalid RMI Instruction (RMIT-020)",
            2556949: "Invalid Value (RMIT-021)",
            2556950: "Invalid Text String (RMIT-022)",
            2556951: "Invalid Position Data (RMIT-023)",
            2556952: "RMI is In HOLD State (RMIT-024)",
            2556953: "Remote Device Disconnected (RMIT-025)",
            2556954: "Robot is Already Connected (RMIT-026)",
            2556955: "Wait for Command Done (RMIT-027)",
            2556956: "Wait for Instruction Done (RMIT-028)",
            2556957: "Invalid sequence ID number (RMIT-029)",
            2556958: "Invalid Speed Type (RMIT-030)",
            2556959: "Invalid Speed Value (RMIT-031)",
            2556960: "Invalid Positioning Type (RMIT-032)",
            2556961: "Invalid CNT Value (RMIT-033)",
            2556962: "Invalid LCB Port Type (RMIT-034)",
            2556963: "Invalid ACC Value (RMIT-035)",
            2556964: "Invalid Destination Position (RMIT-036)",
            2556965: "Invalid VIA Position (RMIT-037)",
            2556966: "Invalid Port Number (RMIT-038)",
            2556967: "Invalid Group Number (RMIT-039)",
            2556968: "Invalid Group Mask (RMIT-040)",
            2556969: "Joint motion with COORD (RMIT-041)",
            2556970: "Incremental motn with COORD (RMIT-042)",
            2556971: "Robot in Single Step Mode (RMIT-043)",
            2556972: "Invalid Position Data Type (RMIT-044)",
            2556973: "Not Ready for ASCII Packet (RMIT-045)",
            2556974: "ASCII Conversion Failed (RMIT-046)",
            2556975: "Invalid ASCII Instruction (RMIT-047)",
            2556976: "Invalid Number of Groups (RMIT-048)",
            2556977: "Invalid Instruction packet (RMIT-049)",
            2556978: "Invalid ASCII packet (RMIT-050)",
            2556979: "Invalid ASCII string size (RMIT-051)",
            2556980: "Invalid Application Tool (RMIT-052)",
            2556981: "Invalid Call Program Name (RMIT-053)",
            2556982: "Joint motion with ALIM (RMIT-054)",
            2556983: "Cannot Use ALIM Instruction (RMIT-055)",
            2556984: "Need to finish S-motion (RMIT-056)",
            2556985: "S-motion is not loaded (RMIT-057)",
            2556986: "Please Cycle power for init (RMIT-058)",
            2556987: "ROS 2 is not loaded (RMIT-059)",
            2556988: "Invalid Register Number (RMIT-060)",
            2556989: "Invalid Data Type (RMIT-061)",
            2556990: "Invalid I/O Port Type (RMIT-062)",
            2556991: "Invalid Variable Name (RMIT-063)",
            2556992: "Invalid Variable Value (RMIT-064)",
            2556993: "J519 Is not loaded (RMIT-065)",
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
                LOGGER.debug(f"REPONSE RECUE: {response_data}")
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
                LOGGER.debug("RMI connection successful.")
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
                    LOGGER.debug("RMI_DISCONNECT successful")
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
                        LOGGER.debug("RMI_GETSTATUS successful")
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

    def rmi_initialize(self, group_mask: int = None, rtsa: str = None, pltz_mode: str = None):
        time.sleep(self.TIME_BUFFER)
        try:
            # GroupMask is required on multi-group systems (bit-field, one bit per group).
            # RTSA enables Singularity Avoidance (R792); PLTZMODE sets the palletizing header.
            initialize_packet = {"Command": "FRC_Initialize"}
            if group_mask is not None:
                assert 0 <= group_mask and group_mask <= 255, "group_mask is out of range"
                initialize_packet["GroupMask"] = group_mask
            if rtsa is not None:
                assert rtsa in ["ON", "OFF"], "rtsa must be ON or OFF"
                initialize_packet["RTSA"] = rtsa
            if pltz_mode is not None:
                expected_pltz = ["ZERODN", "ZEROUP", "PSPIDN", "PSPIUP", "MSPIDN", "MSPIUP"]
                assert pltz_mode in expected_pltz, "pltz_mode isn't correctly written ; pltz_mode : " + pltz_mode
                initialize_packet["PLTZMODE"] = pltz_mode
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
                    LOGGER.debug("RMI_INITIALIZE successful")
                    if request_successful
                    else LOGGER.error(f"RMI_INITIALIZE failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_initialize(): {e}")

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
                    LOGGER.debug("RMI_ABORT successful")
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
                    LOGGER.debug("RMI_RESET successful")
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
                    LOGGER.debug("RMI_PAUSE successful")
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
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.debug("RMI_CONTINUE successful")
                    if request_successful
                    else LOGGER.error(f"RMI_CONTINUE failed: {error_str}")
                )
                return response
        except Exception as e:
            LOGGER.error(f"error rmi_continue(): {e}")

    def rmi_read_error(self, count: int = None):
        time.sleep(self.TIME_BUFFER)
        try:
            read_error_packet = {"Command": "FRC_ReadError"}
            if count is not None:
                assert 1 <= count and count <= 5, "count is out of range (valid range is 1 to 5)"
                read_error_packet["Count"] = count
            response = self.send_message(packet=read_error_packet)
            return response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error read_error(): {e}")

    def startup_sequence(self, verbose=True, max_attempts=5) -> bool:
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

                if attempt % 2 == 0:
                    if not self.rmi_abort():
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

                LOGGER.debug("Robot ready for initialization")
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
                    LOGGER.debug("RMI_SET_UF_UT successful")
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
                    LOGGER.debug("RMI_READ_UF_DATA successful")
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
                    LOGGER.debug("RMI_WRITE_UF_DATA successful")
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
                    LOGGER.debug("RMI_READ_UT_DATA successful")
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
                    LOGGER.debug("RMI_WRITE_UT_DATA successful")
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
                    LOGGER.debug("RMI_READ_CARTESIAN_POSITION successful")
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
                    LOGGER.debug("RMI_READ_CARTESIAN_POSITION successful")
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
                    LOGGER.debug("RMI_SET_OVERRIDE successful")
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
                    LOGGER.debug("RMI_GET_UF_UT successful")
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
                    LOGGER.debug("RMI_READ_PR successful")
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
                    LOGGER.debug("RMI_WRITE_PR successful")
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
                    LOGGER.debug("RMI_READ_TCP_SPEED successful")
                    if request_successful
                    else LOGGER.error(f"RMI_READ_TCP_SPEED failed: {error_str}")
                )
                return request_successful, response
        except Exception as e:
            LOGGER.error(f"error rmi_read_tcp_speed(): {e}")

    def rmi_read_register(self, register_number: int):
        time.sleep(self.TIME_BUFFER)
        try:
            assert 1 <= register_number, "register_number is out of range"
            read_register_packet = {"Command": "FRC_ReadRegister", "RegisterNumber": register_number}
            response = self.send_message(packet=read_register_packet)
            if response is None:
                raise Exception("Error while sending read_register_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.debug("RMI_READ_REGISTER successful")
                    if request_successful
                    else LOGGER.error(f"RMI_READ_REGISTER failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_read_register(): {e}")

    def rmi_write_register(self, register_number: int, value, data_type: str = "Integer"):
        time.sleep(self.TIME_BUFFER)
        try:
            assert 1 <= register_number, "register_number is out of range"
            assert data_type in ["Integer", "Float"], "data_type must be Integer or Float"
            write_register_packet = {
                "Command": "FRC_WriteRegister",
                "RegisterNumber": register_number,
                "RegisterValue": value,
                "DataType": data_type,
            }
            response = self.send_message(packet=write_register_packet)
            if response is None:
                raise Exception("Error while sending write_register_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.debug("RMI_WRITE_REGISTER successful")
                    if request_successful
                    else LOGGER.error(f"RMI_WRITE_REGISTER failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_write_register(): {e}")

    def rmi_read_string_register(self, register_number: int):
        time.sleep(self.TIME_BUFFER)
        try:
            assert 1 <= register_number, "register_number is out of range"
            read_sr_packet = {"Command": "FRC_ReadStringRegister", "RegisterNumber": register_number}
            response = self.send_message(packet=read_sr_packet)
            if response is None:
                raise Exception("Error while sending read_sr_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.debug("RMI_READ_STRING_REGISTER successful")
                    if request_successful
                    else LOGGER.error(f"RMI_READ_STRING_REGISTER failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_read_string_register(): {e}")

    def rmi_write_string_register(self, register_number: int, string_value: str):
        time.sleep(self.TIME_BUFFER)
        try:
            assert 1 <= register_number, "register_number is out of range"
            assert len(string_value) <= 254, "string_value must be 254 bytes or less"
            write_sr_packet = {
                "Command": "FRC_WriteStringRegister",
                "RegisterNumber": register_number,
                "StringValue": string_value,
            }
            response = self.send_message(packet=write_sr_packet)
            if response is None:
                raise Exception("Error while sending write_sr_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.debug("RMI_WRITE_STRING_REGISTER successful")
                    if request_successful
                    else LOGGER.error(f"RMI_WRITE_STRING_REGISTER failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_write_string_register(): {e}")

    def rmi_read_variable(self, variable_name: str):
        time.sleep(self.TIME_BUFFER)
        try:
            assert variable_name.startswith("$"), "variable_name must include the leading '$'"
            assert len(variable_name) <= 64, "variable_name must be 64 bytes or less"
            read_variable_packet = {"Command": "FRC_ReadVariable", "VariableName": variable_name}
            response = self.send_message(packet=read_variable_packet)
            if response is None:
                raise Exception("Error while sending read_variable_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.debug("RMI_READ_VARIABLE successful")
                    if request_successful
                    else LOGGER.error(f"RMI_READ_VARIABLE failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_read_variable(): {e}")

    def rmi_write_variable(self, variable_name: str, variable_type: str, value):
        time.sleep(self.TIME_BUFFER)
        try:
            assert variable_name.startswith("$"), "variable_name must include the leading '$'"
            assert len(variable_name) <= 64, "variable_name must be 64 bytes or less"
            assert variable_type in ["Integer", "Float"], "variable_type must be Integer or Float"
            write_variable_packet = {
                "Command": "FRC_WriteVariable",
                "VariableName": variable_name,
                "VariableType": variable_type,
                "Value": value,
            }
            response = self.send_message(packet=write_variable_packet)
            if response is None:
                raise Exception("Error while sending write_variable_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.debug("RMI_WRITE_VARIABLE successful")
                    if request_successful
                    else LOGGER.error(f"RMI_WRITE_VARIABLE failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_write_variable(): {e}")

    def rmi_read_io_port(self, port_type: str, port_number: int):
        time.sleep(self.TIME_BUFFER)
        try:
            expected_port_type = ["AI", "GI", "DI", "RI", "AO", "GO", "DO", "RO", "FLAG"]
            assert port_type in expected_port_type, "port_type isn't correctly written ; port_type : " + str(port_type)
            assert 0 <= port_number, "port_number is out of range"
            read_io_port_packet = {"Command": "FRC_ReadIOPort", "PortType": port_type, "PortNumber": port_number}
            response = self.send_message(packet=read_io_port_packet)
            if response is None:
                raise Exception("Error while sending read_io_port_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.debug("RMI_READ_IO_PORT successful")
                    if request_successful
                    else LOGGER.error(f"RMI_READ_IO_PORT failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_read_io_port(): {e}")

    def rmi_write_io_port(self, port_type: str, port_number: int, port_value):
        time.sleep(self.TIME_BUFFER)
        try:
            expected_port_type = ["AO", "GO", "DO", "RO", "FLAG"]
            assert port_type in expected_port_type, "port_type isn't correctly written ; port_type : " + str(port_type)
            assert 0 <= port_number, "port_number is out of range"
            write_io_port_packet = {
                "Command": "FRC_WriteIOPort",
                "PortType": port_type,
                "PortNumber": port_number,
                "PortValue": port_value,
            }
            response = self.send_message(packet=write_io_port_packet)
            if response is None:
                raise Exception("Error while sending write_io_port_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.debug("RMI_WRITE_IO_PORT successful")
                    if request_successful
                    else LOGGER.error(f"RMI_WRITE_IO_PORT failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_write_io_port(): {e}")

    def rmi_set_payload_id(self, schedule_number: int, group=1):
        time.sleep(self.TIME_BUFFER)
        try:
            assert 0 <= schedule_number, "schedule_number is out of range"
            set_payload_id_packet = {
                "Command": "FRC_SetPayloadID",
                "ScheduleNumber": schedule_number,
                "Group": group,
            }
            response = self.send_message(packet=set_payload_id_packet)
            if response is None:
                raise Exception("Error while sending set_payload_id_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.debug("RMI_SET_PAYLOAD_ID successful")
                    if request_successful
                    else LOGGER.error(f"RMI_SET_PAYLOAD_ID failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_set_payload_id(): {e}")

    def rmi_set_payload_value(
        self,
        schedule_number: int,
        mass: float,
        cg_x: float,
        cg_y: float,
        cg_z: float,
        in_x: float = None,
        in_y: float = None,
        in_z: float = None,
        group=1,
    ):
        time.sleep(self.TIME_BUFFER)
        try:
            assert 0 <= schedule_number, "schedule_number is out of range"
            # Mass is in kg, CG_* in cm, IN_* (payload inertia) in kgcm^2 and optional.
            set_payload_value_packet = {
                "Command": "FRC_SetPayloadValue",
                "ScheduleNumber": schedule_number,
                "Group": group,
                "Mass": mass,
                "CG_X": cg_x,
                "CG_Y": cg_y,
                "CG_Z": cg_z,
            }
            if in_x is not None:
                set_payload_value_packet["IN_X"] = in_x
            if in_y is not None:
                set_payload_value_packet["IN_Y"] = in_y
            if in_z is not None:
                set_payload_value_packet["IN_Z"] = in_z
            response = self.send_message(packet=set_payload_value_packet)
            if response is None:
                raise Exception("Error while sending set_payload_value_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.debug("RMI_SET_PAYLOAD_VALUE successful")
                    if request_successful
                    else LOGGER.error(f"RMI_SET_PAYLOAD_VALUE failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_set_payload_value(): {e}")

    def rmi_restart(self):
        time.sleep(self.TIME_BUFFER)
        try:
            # FRC_Restart resumes a paused program from the next instruction (sequence ID must restart at 1).
            restart_packet = {"Command": "FRC_Restart"}
            response = self.send_message(packet=restart_packet)
            if response is None:
                raise Exception("Error while sending restart_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                if request_successful:
                    self.SEQUENCE_ID = 1
                    LOGGER.debug("RMI_RESTART successful")
                else:
                    LOGGER.error(f"RMI_RESTART failed: {error_str}")
                return request_successful, response
        except Exception as e:
            LOGGER.error(f"error rmi_restart(): {e}")

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
                    LOGGER.debug("RMI_WAIT_DIN successful")
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
                    LOGGER.debug("RMI_SET_U_FRAME successful")
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
                    LOGGER.debug("RMI_SET_U_TOOL successful")
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
                    LOGGER.debug("RMI_WAIT_TIME successful")
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
            assert schedule_number > 0 and schedule_number < 9, "schedule_number is out of range"
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
                    LOGGER.debug("RMI_SET_PAYLOAD successful")
                    if request_successful
                    else LOGGER.error(f"RMI_SET_PAYLOAD failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_set_payload(): {e}")

    def rmi_call(self, program_name: str, params: list = None):
        time.sleep(self.TIME_BUFFER)
        try:
            if not program_name or not isinstance(program_name, str):
                raise ValueError("Invalid program_name. It must be a non-empty string.")
            assert len(program_name) <= 36, "program_name must be 36 bytes or less"
            sequence_id = self.SEQUENCE_ID
            self.SEQUENCE_ID += 1
            call_packet = {"Instruction": "FRC_Call", "SequenceID": sequence_id, "ProgramName": program_name}
            # Optional program parameters: a list of (param_type, param_value) tuples (max 10).
            # Each pair is sent as consecutive ParamTypeN / ParamValueN keys.
            if params:
                assert len(params) <= 10, "a maximum of 10 parameters is allowed"
                expected_param_types = ["AR", "PR", "SR", "R", "P", "Constant", "String"]
                for index, (param_type, param_value) in enumerate(params, start=1):
                    assert (
                        param_type in expected_param_types
                    ), "param_type isn't correctly written ; param_type : " + str(param_type)
                    call_packet[f"ParamType{index}"] = param_type
                    call_packet[f"ParamValue{index}"] = param_value
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
                    LOGGER.debug("RMI_CALL successful")
                    if request_successful
                    else LOGGER.error(f"RMI_CALL failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
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
                    LOGGER.debug("RMI_LINEAR_MOTION successful")
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
                    LOGGER.debug("RMI_LINEAR_RELATIVE successful")
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
                    LOGGER.debug("RMI_JOINT_MOTION successful")
                    if request_successful
                    else LOGGER.error(f"RMI_JOINT_MOTION failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_joint_motion(): {e}")


    def rmi_joint_motion_JRep(self, jointAngles: dict, speed_type: str, speed, term_type: str):
        time.sleep(self.TIME_BUFFER)
        try:
            expected_keys_jointAngles = {"J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"}
            assert (
                set(jointAngles.keys()) == expected_keys_jointAngles
            ), "the input position isn't correctly written ; position : " + str(jointAngles)
            expected_speed_type = ["Percent", "Time", "mSec"]
            assert speed_type in expected_speed_type, "speed_type isn't correctly written ; speed_type : " + speed_type
            expected_term_type = ["FINE", "CNT", "CR"]
            assert term_type in expected_term_type, "term_type isn't correctly written ; term_type : " + term_type

            sequence_id = self.SEQUENCE_ID
            self.SEQUENCE_ID += 1
            joint_motion_packet = {
                "Instruction": "FRC_JointMotionJRep",
                "SequenceID": sequence_id,
                "JointAngle": jointAngles,
                "SpeedType": speed_type,
                "Speed": speed,
                "TermType": term_type,
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
                    LOGGER.debug("rmi_joint_motion_JRep successful")
                    if request_successful
                    else LOGGER.error(f"rmi_joint_motion_JRep failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_joint_motion_JRep(): {e}")

    def rmi_joint_relative(
        self, config: dict, position: dict, speed_type: str, speed, term_type: str, term_value, optionals: dict = None
    ):
        time.sleep(self.TIME_BUFFER)
        try:
            optionals = optionals or {}
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
            joint_relative_packet = {
                "Instruction": "FRC_JointRelative",
                "SequenceID": sequence_id,
                "Configuration": config,
                "Position": position,
                "SpeedType": speed_type,
                "Speed": speed,
                "TermType": term_type,
                "TermValue": term_value,
            }
            joint_relative_packet = joint_relative_packet | optionals  # merge two dicts
            response = self.send_message(packet=joint_relative_packet)
            if response is None:
                raise Exception("Error while sending joint_relative_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.debug("RMI_JOINT_RELATIVE successful")
                    if request_successful
                    else LOGGER.error(f"RMI_JOINT_RELATIVE failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_joint_relative(): {e}")

    def rmi_circular_motion(
        self,
        config: dict,
        position: dict,
        via_config: dict,
        via_position: dict,
        speed_type: str,
        speed,
        term_type: str,
        term_value,
        optionals: dict = None,
    ):
        time.sleep(self.TIME_BUFFER)
        try:
            optionals = optionals or {}
            expected_keys = {"UToolNumber", "UFrameNumber", "Front", "Up", "Left", "Flip", "Turn4", "Turn5", "Turn6"}
            assert set(config.keys()) == expected_keys, "the config isn't correctly written ; config : " + str(config)
            assert (
                set(via_config.keys()) == expected_keys
            ), "the via_config isn't correctly written ; via_config : " + str(via_config)
            expected_keys_position = {"X", "Y", "Z", "W", "P", "R", "Ext1", "Ext2", "Ext3"}
            assert (
                set(position.keys()) == expected_keys_position
            ), "the input position isn't correctly written ; position : " + str(position)
            assert (
                set(via_position.keys()) == expected_keys_position
            ), "the via_position isn't correctly written ; via_position : " + str(via_position)
            expected_speed_type = ["mmSec", "InchMin", "Time", "mSec"]
            assert speed_type in expected_speed_type, "speed_type isn't correctly written ; speed_type : " + speed_type
            expected_term_type = ["FINE", "CNT", "CR"]
            assert term_type in expected_term_type, "term_type isn't correctly written ; term_type : " + term_type
            assert 1 <= term_value and term_value <= 100, "term_value is out of range"

            sequence_id = self.SEQUENCE_ID
            self.SEQUENCE_ID += 1
            circular_motion_packet = {
                "Instruction": "FRC_CircularMotion",
                "SequenceID": sequence_id,
                "Configuration": config,
                "Position": position,
                "ViaConfiguration": via_config,
                "ViaPosition": via_position,
                "SpeedType": speed_type,
                "Speed": speed,
                "TermType": term_type,
                "TermValue": term_value,
            }
            circular_motion_packet = circular_motion_packet | optionals  # merge two dicts
            response = self.send_message(packet=circular_motion_packet)
            if response is None:
                raise Exception("Error while sending circular_motion_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.debug("RMI_CIRCULAR_MOTION successful")
                    if request_successful
                    else LOGGER.error(f"RMI_CIRCULAR_MOTION failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_circular_motion(): {e}")

    def rmi_circular_relative(
        self,
        config: dict,
        position: dict,
        via_config: dict,
        via_position: dict,
        speed_type: str,
        speed,
        term_type: str,
        term_value,
        optionals: dict = None,
    ):
        time.sleep(self.TIME_BUFFER)
        try:
            optionals = optionals or {}
            expected_keys = {"UToolNumber", "UFrameNumber", "Front", "Up", "Left", "Flip", "Turn4", "Turn5", "Turn6"}
            assert set(config.keys()) == expected_keys, "the config isn't correctly written ; config : " + str(config)
            assert (
                set(via_config.keys()) == expected_keys
            ), "the via_config isn't correctly written ; via_config : " + str(via_config)
            expected_keys_position = {"X", "Y", "Z", "W", "P", "R", "Ext1", "Ext2", "Ext3"}
            assert (
                set(position.keys()) == expected_keys_position
            ), "the input position isn't correctly written ; position : " + str(position)
            assert (
                set(via_position.keys()) == expected_keys_position
            ), "the via_position isn't correctly written ; via_position : " + str(via_position)
            expected_speed_type = ["mmSec", "InchMin", "Time", "mSec"]
            assert speed_type in expected_speed_type, "speed_type isn't correctly written ; speed_type : " + speed_type
            expected_term_type = ["FINE", "CNT", "CR"]
            assert term_type in expected_term_type, "term_type isn't correctly written ; term_type : " + term_type
            assert 1 <= term_value and term_value <= 100, "term_value is out of range"

            sequence_id = self.SEQUENCE_ID
            self.SEQUENCE_ID += 1
            circular_relative_packet = {
                "Instruction": "FRC_CircularRelative",
                "SequenceID": sequence_id,
                "Configuration": config,
                "Position": position,
                "ViaConfiguration": via_config,
                "ViaPosition": via_position,
                "SpeedType": speed_type,
                "Speed": speed,
                "TermType": term_type,
                "TermValue": term_value,
            }
            circular_relative_packet = circular_relative_packet | optionals  # merge two dicts
            response = self.send_message(packet=circular_relative_packet)
            if response is None:
                raise Exception("Error while sending circular_relative_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.debug("RMI_CIRCULAR_RELATIVE successful")
                    if request_successful
                    else LOGGER.error(f"RMI_CIRCULAR_RELATIVE failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_circular_relative(): {e}")

    def rmi_joint_relative_JRep(
        self, jointAngles: dict, speed_type: str, speed, term_type: str, term_value, optionals: dict = None
    ):
        time.sleep(self.TIME_BUFFER)
        try:
            optionals = optionals or {}
            expected_keys_jointAngles = {"J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"}
            assert (
                set(jointAngles.keys()) == expected_keys_jointAngles
            ), "the input position isn't correctly written ; position : " + str(jointAngles)
            expected_speed_type = ["Percent", "Time", "mSec"]
            assert speed_type in expected_speed_type, "speed_type isn't correctly written ; speed_type : " + speed_type
            expected_term_type = ["FINE", "CNT", "CR"]
            assert term_type in expected_term_type, "term_type isn't correctly written ; term_type : " + term_type
            assert 1 <= term_value and term_value <= 100, "term_value is out of range"

            sequence_id = self.SEQUENCE_ID
            self.SEQUENCE_ID += 1
            joint_relative_packet = {
                "Instruction": "FRC_JointRelativeJRep",
                "SequenceID": sequence_id,
                "JointAngle": jointAngles,
                "SpeedType": speed_type,
                "Speed": speed,
                "TermType": term_type,
                "TermValue": term_value,
            }
            joint_relative_packet = joint_relative_packet | optionals  # merge two dicts
            response = self.send_message(packet=joint_relative_packet)
            if response is None:
                raise Exception("Error while sending joint_relative_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.debug("RMI_JOINT_RELATIVE_JREP successful")
                    if request_successful
                    else LOGGER.error(f"RMI_JOINT_RELATIVE_JREP failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_joint_relative_JRep(): {e}")

    def rmi_linear_motion_JRep(
        self, jointAngles: dict, speed_type: str, speed, term_type: str, term_value, optionals: dict = None
    ):
        time.sleep(self.TIME_BUFFER)
        try:
            optionals = optionals or {}
            expected_keys_jointAngles = {"J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"}
            assert (
                set(jointAngles.keys()) == expected_keys_jointAngles
            ), "the input position isn't correctly written ; position : " + str(jointAngles)
            expected_speed_type = ["mmSec", "InchMin", "Time", "mSec"]
            assert speed_type in expected_speed_type, "speed_type isn't correctly written ; speed_type : " + speed_type
            expected_term_type = ["FINE", "CNT", "CR"]
            assert term_type in expected_term_type, "term_type isn't correctly written ; term_type : " + term_type
            assert 1 <= term_value and term_value <= 100, "term_value is out of range"

            sequence_id = self.SEQUENCE_ID
            self.SEQUENCE_ID += 1
            linear_motion_packet = {
                "Instruction": "FRC_LinearMotionJRep",
                "SequenceID": sequence_id,
                "JointAngle": jointAngles,
                "SpeedType": speed_type,
                "Speed": speed,
                "TermType": term_type,
                "TermValue": term_value,
            }
            linear_motion_packet = linear_motion_packet | optionals  # merge two dicts
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
                    LOGGER.debug("RMI_LINEAR_MOTION_JREP successful")
                    if request_successful
                    else LOGGER.error(f"RMI_LINEAR_MOTION_JREP failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_linear_motion_JRep(): {e}")

    def rmi_linear_relative_JRep(
        self, jointAngles: dict, speed_type: str, speed, term_type: str, term_value, optionals: dict = None
    ):
        time.sleep(self.TIME_BUFFER)
        try:
            optionals = optionals or {}
            expected_keys_jointAngles = {"J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"}
            assert (
                set(jointAngles.keys()) == expected_keys_jointAngles
            ), "the input position isn't correctly written ; position : " + str(jointAngles)
            expected_speed_type = ["mmSec", "InchMin", "Time", "mSec"]
            assert speed_type in expected_speed_type, "speed_type isn't correctly written ; speed_type : " + speed_type
            expected_term_type = ["FINE", "CNT", "CR"]
            assert term_type in expected_term_type, "term_type isn't correctly written ; term_type : " + term_type
            assert 1 <= term_value and term_value <= 100, "term_value is out of range"

            sequence_id = self.SEQUENCE_ID
            self.SEQUENCE_ID += 1
            linear_relative_packet = {
                "Instruction": "FRC_LinearRelativeJRep",
                "SequenceID": sequence_id,
                "JointAngle": jointAngles,
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
                    LOGGER.debug("RMI_LINEAR_RELATIVE_JREP successful")
                    if request_successful
                    else LOGGER.error(f"RMI_LINEAR_RELATIVE_JREP failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_linear_relative_JRep(): {e}")

    def rmi_spline_motion(
        self, config: dict, position: dict, speed_type: str, speed, term_type: str, term_value, optionals: dict = None
    ):
        time.sleep(self.TIME_BUFFER)
        try:
            # Requires the Spline Motion (R904) option. A spline move needs at least one
            # following motion instruction before the controller executes it.
            optionals = optionals or {}
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
            spline_motion_packet = {
                "Instruction": "FRC_SplineMotion",
                "SequenceID": sequence_id,
                "Configuration": config,
                "Position": position,
                "SpeedType": speed_type,
                "Speed": speed,
                "TermType": term_type,
                "TermValue": term_value,
            }
            spline_motion_packet = spline_motion_packet | optionals  # merge two dicts
            response = self.send_message(packet=spline_motion_packet)
            if response is None:
                raise Exception("Error while sending spline_motion_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.debug("RMI_SPLINE_MOTION successful")
                    if request_successful
                    else LOGGER.error(f"RMI_SPLINE_MOTION failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_spline_motion(): {e}")

    def rmi_spline_motion_JRep(
        self, jointAngles: dict, speed_type: str, speed, term_type: str, term_value, optionals: dict = None
    ):
        time.sleep(self.TIME_BUFFER)
        try:
            optionals = optionals or {}
            expected_keys_jointAngles = {"J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"}
            assert (
                set(jointAngles.keys()) == expected_keys_jointAngles
            ), "the input position isn't correctly written ; position : " + str(jointAngles)
            expected_speed_type = ["mmSec", "InchMin", "Time", "mSec"]
            assert speed_type in expected_speed_type, "speed_type isn't correctly written ; speed_type : " + speed_type
            expected_term_type = ["FINE", "CNT", "CR"]
            assert term_type in expected_term_type, "term_type isn't correctly written ; term_type : " + term_type
            assert 1 <= term_value and term_value <= 100, "term_value is out of range"

            sequence_id = self.SEQUENCE_ID
            self.SEQUENCE_ID += 1
            spline_motion_packet = {
                "Instruction": "FRC_SplineMotionJRep",
                "SequenceID": sequence_id,
                "JointAngle": jointAngles,
                "SpeedType": speed_type,
                "Speed": speed,
                "TermType": term_type,
                "TermValue": term_value,
            }
            spline_motion_packet = spline_motion_packet | optionals  # merge two dicts
            response = self.send_message(packet=spline_motion_packet)
            if response is None:
                raise Exception("Error while sending spline_motion_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.debug("RMI_SPLINE_MOTION_JREP successful")
                    if request_successful
                    else LOGGER.error(f"RMI_SPLINE_MOTION_JREP failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_spline_motion_JRep(): {e}")

    def rmi_set_output_port(self, output_type: str, port_number: int, set_type: str, set_value):
        time.sleep(self.TIME_BUFFER)
        try:
            expected_output_type = ["AO", "GO", "DO", "RO", "FLAG"]
            assert (
                output_type in expected_output_type
            ), "output_type isn't correctly written ; output_type : " + str(output_type)
            expected_set_type = ["AI", "GI", "DI", "RI", "Register", "Constant", "FLAG"]
            assert set_type in expected_set_type, "set_type isn't correctly written ; set_type : " + str(set_type)
            assert 0 <= port_number, "port_number is out of range"
            sequence_id = self.SEQUENCE_ID
            self.SEQUENCE_ID += 1
            set_output_port_packet = {
                "Instruction": "FRC_SetOutputPort",
                "SequenceID": sequence_id,
                "OutputType": output_type,
                "PortNumber": port_number,
                "SetType": set_type,
                "SetValue": set_value,
            }
            response = self.send_message(packet=set_output_port_packet)
            if response is None:
                raise Exception("Error while sending set_output_port_packet")
            error_id = response.get("ErrorID", None)
            if error_id is None:
                raise Exception("Error while fetching ErrorID")
            else:
                request_successful = error_id == 0
                error_str = self.get_error_string(error_id)
                (
                    LOGGER.debug("RMI_SET_OUTPUT_PORT successful")
                    if request_successful
                    else LOGGER.error(f"RMI_SET_OUTPUT_PORT failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_set_output_port(): {e}")

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
                    LOGGER.debug("RMI_WRITE_D_OUT successful")
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
                    LOGGER.debug("RMI_READ_D_IN successful")
                    if request_successful
                    else LOGGER.error(f"RMI_READ_D_IN failed: {error_str}")
                )
                return request_successful, response
        except AssertionError as ae:
            LOGGER.error(ae)
        except Exception as e:
            LOGGER.error(f"error rmi_read_d_in(): {e}")


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
