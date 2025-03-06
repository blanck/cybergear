#!/usr/bin/python3

import math
import can
import time
from struct import pack, unpack
from typing import Optional, Tuple, Union

class CyberGear:
    # Command constants
    CMD_POSITION = 1
    CMD_RESPONSE = 2
    CMD_ENABLE = 3
    CMD_RESET = 4
    CMD_SET_MECH_POSITION_TO_ZERO = 6
    CMD_CHANGE_CAN_ID = 7
    CMD_RAM_READ = 17
    CMD_RAM_WRITE = 18
    CMD_GET_MOTOR_FAIL = 21

    # Address constants
    ADDR_RUN_MODE = 0x7005
    ADDR_IQ_REF = 0x7006
    ADDR_SPEED_REF = 0x700A
    ADDR_LIMIT_TORQUE = 0x700B
    ADDR_CURRENT_KP = 0x7010
    ADDR_CURRENT_KI = 0x7011
    ADDR_CURRENT_FILTER_GAIN = 0x7014
    ADDR_LOC_REF = 0x7016
    ADDR_LIMIT_SPEED = 0x7017
    ADDR_LIMIT_CURRENT = 0x7018
    ADDR_MECH_POS = 0x7019
    ADDR_IQF = 0x701A
    ADDR_MECH_VEL = 0x701B
    ADDR_VBUS = 0x701C
    ADDR_ROTATION = 0x701D
    ADDR_LOC_KP = 0x701E
    ADDR_SPD_KP = 0x701F
    ADDR_SPD_KI = 0x7020

    # Mode constants
    MODE_MOTION = 0x00
    MODE_POSITION = 0x01
    MODE_SPEED = 0x02
    MODE_CURRENT = 0x03

    # Parameter limits
    P_MIN = -12.5
    P_MAX = 12.5
    V_MIN = -30.0
    V_MAX = 30.0
    KP_MIN = 0.0
    KP_MAX = 500.0
    KI_MIN = 0.0
    KI_MAX = 5.0
    KD_MIN = 0.0
    KD_MAX = 5.0
    T_MIN = -12.0
    T_MAX = 12.0
    IQ_MIN = -27.0
    IQ_MAX = 27.0
    CURRENT_FILTER_GAIN_MIN = 0.0
    CURRENT_FILTER_GAIN_MAX = 1.0

    IQ_REF_MAX = 23.0
    IQ_REF_MIN = -23.0
    SPD_REF_MAX = 30.0
    SPD_REF_MIN = -30.0
    LIMIT_TORQUE_MAX = 12.0
    LIMIT_TORQUE_MIN = 0.0
    CUR_KP_MAX = 200.0
    CUR_KP_MIN = 0.0
    CUR_KI_MAX = 200.0
    CUR_KI_MIN = 0.0
    LOC_KP_MAX = 200.0
    LOC_KP_MIN = 0.0
    SPD_KP_MAX = 200.0
    SPD_KP_MIN = 0.0
    LIMIT_SPD_MAX = 30.0
    LIMIT_SPD_MIN = 0.0
    LIMIT_CURRENT_MAX = 27.0
    LIMIT_CURRENT_MIN = 0.0

    # Default values
    DEFAULT_CURRENT_KP = 0.065
    DEFAULT_CURRENT_KI = 0.065
    DEFAULT_CURRENT_FILTER_GAIN = 0.0
    DEFAULT_POSITION_KP = 30.0
    DEFAULT_VELOCITY_KP = 2.0
    DEFAULT_VELOCITY_KI = 0.002
    DEFAULT_VELOCITY_LIMIT = 2.0
    DEFAULT_CURRENT_LIMIT = 1.0
    DEFAULT_TORQUE_LIMIT = 12.0

    # Response codes
    RET_CYBERGEAR_OK = 0x00
    RET_CYBERGEAR_MSG_NOT_AVAIL = 0x01
    RET_CYBERGEAR_INVALID_CAN_ID = 0x02
    RET_CYBERGEAR_INVALID_PACKET = 0x03

    # Timing
    CYBERGEAR_RESPONSE_TIME_USEC = 250

    # Direction constants
    CW = 1
    CCW = -1

    def __init__(self, interface: str = 'slcan', channel: str = '/dev/ttyUSB_CAN', bitrate: int = 1000000, debug: bool = False):
        """
        Initialize the CyberGear motor controller.

        Parameters:
        -----------
        interface : str, optional
            The CAN interface to use. Examples:
            - 'socketcan' for Raspberry Pi (channel='can0')
            - 'slcan' for Cannable (channel='/dev/ttyUSB0')
            - 'slcan' for Serial CAN (channel='/dev/tty.usbserial-130')
        channel : str, optional
            The CAN channel to use. Examples:
            - 'can0' for Raspberry Pi
            - '/dev/ttyUSB0' for Cannable
            - '/dev/tty.usbserial-130' for Serial CAN
        bitrate : int, optional
            The CAN bus bitrate (default: 1000000).
        debug : bool, optional
            Enable debug mode to print CAN messages (default: False).
        """
        self.__canbus = can.interface.Bus(interface=interface, channel=channel, bitrate=bitrate, extended=True)
        self.debug = debug

    def init_motor(self, motor_id: int):
        """
        Initialize a motor instance.

        Parameters:
        -----------
        motor_id : int
            The CAN ID of the motor.

        Returns:
        --------
        Motor
            An instance of the Motor class.
        """
        return Motor(self.__canbus, motor_id, self.debug)

    def close(self):
        """Shutdown the CAN bus."""
        self.__canbus.shutdown()


class Motor:
    def __init__(self, canbus: can.Bus, motor_id: int, debug: bool = False):
        """
        Initialize a motor instance.

        Parameters:
        -----------
        canbus : can.Bus
            The shared CAN bus interface.
        motor_id : int
            The CAN ID of the motor.
        debug : bool, optional
            Enable debug mode to print CAN messages (default: False).
        """
        self.__canbus = canbus
        self.motor_id = motor_id
        self.debug = debug
        self.reset_motor()
        
    def __send_receive(self, cmd: int, id_opt: int = 0, data: bytes = bytes([0] * 8)) -> Optional[can.Message]:
        """Send a CAN message and receive a response."""
        if self.motor_id <= 0x7f:
            try:
                # Construct CAN message ID as per Arduino code
                msg_id = (cmd & 0x1F) << 24 | (id_opt & 0xFFFF) << 8 | (self.motor_id & 0xFF)
                msg = can.Message(arbitration_id=msg_id, data=data, is_extended_id=True)
                if self.debug:
                    print(f'TX: id={msg.arbitration_id:08x} data=', ':'.join(f'{x:02x}' for x in msg.data))
                self.__canbus.send(msg)
                r = self.__canbus.recv(timeout=CyberGear.CYBERGEAR_RESPONSE_TIME_USEC / 1000000)
                if r is not None:
                    if self.debug:
                        print(f'RX: id={r.arbitration_id:08X} data=', ':'.join(f'{x:02x}' for x in r.data))
                    return r
            except Exception as e:
                if self.debug:
                    print(f"Error: {e}")
        return None

    def __verify_response_packet(self, response: can.Message) -> bool:
        """
        Verify that the response packet is valid.

        Parameters:
        -----------
        response : can.Message
            The CAN message received from the motor.

        Returns:
        --------
        bool
            True if the packet is valid, False otherwise.
        """
        try:
            # Extract packet type from the CAN message ID
            packet_type = (response.arbitration_id >> 24) & 0x3F

            # Check if the packet type is CMD_RESPONSE
            if packet_type != CyberGear.CMD_RESPONSE:
                if self.debug:
                    print(f"Invalid packet type: {packet_type}")
                return False

            return True
        except Exception as e:
            if self.debug:
                print(f"Error verifying response packet: {e}")
            return False

    def __process_motor_packet(self, data: bytes) -> Tuple[float, float, float, float]:
        """
        Process the motor status packet and extract position, velocity, effort, and temperature.

        Parameters:
        -----------
        data : bytes
            The data payload of the CAN message.

        Returns:
        --------
        Tuple[float, float, float, float]
            A tuple containing position, velocity, effort, and temperature.
        """
        if len(data) != 8:
            raise ValueError(f"Unexpected data length: {len(data)} bytes")

        # Unpack raw data from the response
        raw_position = data[0] << 8 | data[1]  # Combine bytes 0 and 1 for position
        raw_velocity = data[2] << 8 | data[3]  # Combine bytes 2 and 3 for velocity
        raw_effort = data[4] << 8 | data[5]    # Combine bytes 4 and 5 for effort
        raw_temperature = data[6] << 8 | data[7]  # Combine bytes 6 and 7 for temperature

        # Convert raw values to floats
        position = self.uint_to_float(raw_position, CyberGear.P_MIN, CyberGear.P_MAX)  # Position in radians
        velocity = self.uint_to_float(raw_velocity, CyberGear.V_MIN, CyberGear.V_MAX)  # Velocity in rad/s
        effort = self.uint_to_float(raw_effort, CyberGear.T_MIN, CyberGear.T_MAX)  # Effort in Nm
        temperature = self.uint_to_float(raw_temperature, 0, 6553.5)  # Temperature in degrees Celsius

        return position, velocity, effort, temperature

    def uint_to_float(self, x: int, min_val: float, max_val: float) -> float:
        """
        Convert a uint16 value to a float within a specified range.

        Parameters:
        -----------
        x : int
            The uint16 value to convert.
        min_val : float
            The minimum value of the range.
        max_val : float
            The maximum value of the range.

        Returns:
        --------
        float
            The converted float value.
        """
        return min_val + (max_val - min_val) * (x / 65535.0)

    def float_to_uint(self, x: float, min_val: float, max_val: float, bits: int) -> int:
        """
        Convert a float to an unsigned integer within a specified range.

        Parameters:
        -----------
        x : float
            The float value to convert.
        min_val : float
            The minimum value of the range.
        max_val : float
            The maximum value of the range.
        bits : int
            The number of bits for the output integer.

        Returns:
        --------
        int
            The converted unsigned integer.
        """
        span = max_val - min_val
        offset = min_val
        return int(((x - offset) / span) * ((1 << bits) - 1))

    def write_param(self, index: int, data, width: str = 'B') -> bool:
        """
        Write a parameter to the motor.

        Parameters:
        -----------
        index : int
            The address/index of the parameter.
        data : int or float
            The value to write.
        width : str, optional
            The data type of the parameter. Options: 'B', 'h', 'H', 'l', 'L', 'f'.
        """
        if width == 'B':
            data_bytes = pack('<HxxBxxx', index, data)
        elif width == 'h':
            data_bytes = pack('<Hxxhxx', index, data)
        elif width == 'H':
            data_bytes = pack('<HxxHxx', index, data)
        elif width == 'l':
            data_bytes = pack('<Hxxl', index, data)
        elif width == 'L':
            data_bytes = pack('<HxxL', index, data)
        elif width == 'f':
            data_bytes = pack('<Hxxf', index, data)
        else:
            raise ValueError(f"Unsupported width: {width}")

        r = self.__send_receive(CyberGear.CMD_RAM_WRITE, data=data_bytes)
        return r is not None

    def set_run_mode(self, mode: int) -> bool:
        """Set the run mode for the motor."""
        return self.write_param(CyberGear.ADDR_RUN_MODE, mode, width='B')

    def set_speed_limit(self, limit: float) -> bool:
        """Set the speed limit for the motor."""
        return self.write_param(CyberGear.ADDR_LIMIT_SPEED, limit, width='f')

    def set_torque_limit(self, limit: float) -> bool:
        """Set the torque limit for the motor."""
        return self.write_param(CyberGear.ADDR_LIMIT_TORQUE, limit, width='f')

    def set_current_limit(self, limit: float) -> bool:
        """Set the current limit for the motor."""
        return self.write_param(CyberGear.ADDR_LIMIT_CURRENT, limit, width='f')

    def set_position_control_gain(self, kp: float) -> bool:
        """Set the position control gain for the motor."""
        return self.write_param(CyberGear.ADDR_LOC_KP, kp, width='f')

    def set_velocity_control_gain(self, kp: float, ki: float) -> bool:
        """Set the velocity control gain for the motor."""
        return self.write_param(CyberGear.ADDR_SPD_KP, kp, width='f') and \
               self.write_param(CyberGear.ADDR_SPD_KI, ki, width='f')

    def set_current_control_param(self, kp: float, ki: float, gain: float) -> bool:
        """Set the current control parameters for the motor."""
        return self.write_param(CyberGear.ADDR_CURRENT_KP, kp, width='f') and \
               self.write_param(CyberGear.ADDR_CURRENT_KI, ki, width='f') and \
               self.write_param(CyberGear.ADDR_CURRENT_FILTER_GAIN, gain, width='f')

    def enable_motor(self) -> bool:
        """Enable the motor."""
        return self.__send_receive(CyberGear.CMD_ENABLE, data=bytes([0] * 8)) is not None

    def reset_motor(self) -> bool:
        """Reset the motor."""
        return self.__send_receive(CyberGear.CMD_RESET, data=bytes([0] * 8)) is not None

    def stop_motor(self, fault: bool = False) -> bool:
        """Stop the motor."""
        data = bytes([1 if fault else 0] + [0] * 7)
        return self.__send_receive(CyberGear.CMD_RESET, data=data) is not None

    def send_position_command(self, position: float) -> bool:
        """Send a position command to the motor."""
        return self.write_param(CyberGear.ADDR_LOC_REF, position, width='f')

    def send_speed_command(self, speed: float) -> bool:
        """Send a speed command to the motor."""
        return self.write_param(CyberGear.ADDR_SPEED_REF, speed, width='f')

    def send_current_command(self, current: float) -> bool:
        """Send a current command to the motor."""
        return self.write_param(CyberGear.ADDR_IQ_REF, current, width='f')

    def send_motion_command(self, position: float, speed: float, torque: float, kp: float, kd: float) -> bool:
        """
        Send a motion command to the motor.

        Parameters:
        -----------
        position : float
            Target position (radians).
        speed : float
            Target speed (rad/sec).
        torque : float
            Target torque (Nm).
        kp : float
            Motion control kp.
        kd : float
            Motion control kd.

        Returns:
        --------
        bool
            True if the command was sent successfully, False otherwise.
        """
        # Convert floats to 16-bit unsigned integers
        pos_int = self.float_to_uint(position, CyberGear.P_MIN, CyberGear.P_MAX, 16)
        speed_int = self.float_to_uint(speed, CyberGear.V_MIN, CyberGear.V_MAX, 16)
        kp_int = self.float_to_uint(kp, CyberGear.KP_MIN, CyberGear.KP_MAX, 16)
        kd_int = self.float_to_uint(kd, CyberGear.KD_MIN, CyberGear.KD_MAX, 16)
        torque_int = self.float_to_uint(torque, CyberGear.T_MIN, CyberGear.T_MAX, 16)

        # Pack the data into bytes
        data = bytes([
            (pos_int >> 8) & 0xFF,  # Position high byte
            pos_int & 0xFF,         # Position low byte
            (speed_int >> 8) & 0xFF,  # Speed high byte
            speed_int & 0xFF,       # Speed low byte
            (kp_int >> 8) & 0xFF,   # Kp high byte
            kp_int & 0xFF,          # Kp low byte
            (kd_int >> 8) & 0xFF,   # Kd high byte
            kd_int & 0xFF           # Kd low byte
        ])

        # Send the motion command
        return self.__send_receive(CyberGear.CMD_POSITION, id_opt=torque_int, data=data) is not None

    def set_mech_position_to_zero(self) -> bool:
        """Set the mechanical position to zero for the motor."""
        data = bytes([1] + [0] * 7)
        return self.__send_receive(CyberGear.CMD_SET_MECH_POSITION_TO_ZERO, data=data) is not None

    def get_motor_status(self, motor_ids: Optional[list[int]] = None, timeout: float = 0.5) -> Union[Optional[Tuple[int, float, float, float, float]], list[Tuple[int, float, float, float, float]]]:
        """
        Get the status of the motor(s).

        Parameters:
        -----------
        motor_ids : Optional[list[int]]
            A list of motor IDs to query. If None, returns the status for the current motor.
        timeout : float, optional
            The maximum time to wait for responses (default: 0.5 seconds).

        Returns:
        --------
        Union[Optional[Tuple[int, float, float, float, float]], list[Tuple[int, float, float, float, float]]]
            - If `motor_ids` is None: Returns a tuple containing motor ID, position, velocity, effort, and temperature for the current motor.
            - If `motor_ids` is provided: Returns a list of tuples, each containing motor ID, position, velocity, effort, and temperature.
        """

        if motor_ids is None:
            # Single-motor mode: Return status for the current motor
            start_time = time.time()
            while time.time() - start_time < timeout:
                response = self.__canbus.recv(timeout=timeout)
                if response is None:
                    if self.debug:
                        print(f"No response received from motor {self.motor_id:02X}.")
                    return None

                # Verify the response packet and check if it's for the current motor
                if not self.__verify_response_packet(response):
                    continue  # Skip invalid packets

                # Extract motor ID from the CAN message ID
                motor_can_id = (response.arbitration_id >> 8) & 0xFF

                # Skip if the response is not for the current motor
                if motor_can_id != self.motor_id:
                    continue

                # Process the motor status data
                try:
                    position, velocity, effort, temperature = self.__process_motor_packet(response.data)
                    return self.motor_id, position, velocity, effort, temperature
                except Exception as e:
                    if self.debug:
                        print(f"Error processing motor packet for motor {self.motor_id:02X}: {e}")
                    return None

            if self.debug:
                print(f"Timeout while waiting for response from motor {self.motor_id:02X}.")
            return None
        else:
            # Multi-motor mode: Return status for all specified motors
            status_dict = {}  # Dictionary to store motor statuses

            # Send status requests to all motors
            for motor_id in motor_ids:
                self.__send_receive(CyberGear.CMD_RESPONSE, id_opt=motor_id)

            # Process all incoming packets
            start_time = time.time()
            while time.time() - start_time < timeout:  # Listen for responses for 0.5 seconds
                response = self.__canbus.recv(timeout=CyberGear.CYBERGEAR_RESPONSE_TIME_USEC / 1000000)  # Wait for a response
                if response is None:
                    continue

                # Verify the response packet
                if not self.__verify_response_packet(response):
                    continue
                
                # Exit if no more data
                if not response.data:
                    break

                # Extract motor ID from the CAN message ID
                motor_can_id = (response.arbitration_id >> 8) & 0xFF

                # Process the motor status data
                try:
                    position, velocity, effort, temperature = self.__process_motor_packet(response.data)
                    status_dict[motor_can_id] = (motor_can_id, position, velocity, effort, temperature)
                except Exception as e:
                    if self.debug:
                        print(f"Error processing motor packet for motor {motor_can_id:02X}: {e}")
                # Exit early if we've received responses from all requested motors
                if all(motor_id in status_dict for motor_id in motor_ids):
                    break

            # Return statuses for all requested motors
            status_list = []
            for motor_id in motor_ids:
                if motor_id in status_dict:
                    status_list.append(status_dict[motor_id])
                else:
                    if self.debug:
                        print(f"No response received from motor {motor_id:02X}.")
            return status_list


if __name__ == "__main__":
    # Example usage: Switch between speed mode and position mode
    cg = CyberGear(interface='slcan', channel='/dev/ttyUSB_CAN', debug=False)
    motor_1 = cg.init_motor(0x7F)
    #motor_2 = cg.init_motor(0x7D)

    try:
        # Run motor 1 in speed mode for 1 second
        print("Running motor 1 in speed mode...")
        motor_1.set_run_mode(cg.MODE_SPEED)
        motor_1.set_speed_limit(4.0)
        motor_1.set_current_control_param(0.075, 0.075, 0.0)
        motor_1.enable_motor()
        motor_1.send_speed_command(6.0)
        time.sleep(1)
        motor_1.stop_motor()

        # Run motor 1 in position mode
        print("Running motor 1 in position mode...")
        motor_1.set_run_mode(cg.MODE_POSITION)
        motor_1.set_position_control_gain(cg.DEFAULT_POSITION_KP)
        motor_1.set_speed_limit(2.0)  # Speed limit for position mode
        motor_1.set_torque_limit(5.0)  # Torque limit for position mode
        motor_1.enable_motor()

        # Send a position command (1.57 radians = 90 degrees)
        target_position = 1.57
        motor_1.send_position_command(target_position)
        print(f"Motor 1 moving to position: {target_position} radians")
        time.sleep(2)

        # Stop the motors
        motor_1.stop_motor()
        #motor_2.stop_motor()
        print("Motors stopped.")

    finally:
        # Close the CAN bus
        cg.close()
        print("CAN bus closed.")