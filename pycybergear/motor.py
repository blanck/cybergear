# src/pycybergear/motor.py

import time
import logging
from struct import pack, unpack
from typing import Optional, Dict, Callable, Union, TYPE_CHECKING
import can

from .constants import CommandType, ParamAddr, RunMode, MotorErrorFlags
from .exceptions import MotorError

# Type hint only needed during static analysis
if TYPE_CHECKING:
    from .controller import CyberGearController

log = logging.getLogger(__name__)

class Motor:
    # Physical limits from documentation
    P_MIN, P_MAX = -12.5, 12.5 # rad
    V_MIN, V_MAX = -30.0, 30.0 # rad/s
    T_MIN, T_MAX = -12.0, 12.0 # Nm
    KP_MIN, KP_MAX = 0.0, 500.0
    KD_MIN, KD_MAX = 0.0, 5.0
    IQ_MAX_PEAK = 23.0 # Peak current (Amps) for Iq ref commands
    IQ_MAX_LIMIT = 27.0 # Limit parameter setting max
    TORQUE_CONSTANT = 0.87 # Nm / Arms

    def __init__(self, motor_id: int, master_can_id: int,
                 send_function: Callable[[int, bytes], bool],
                 controller: 'CyberGearController'):
        """Initializes a Motor instance. Should be created via CyberGearController.add_motor()."""
        self._motor_id: int = motor_id
        self._master_can_id: int = master_can_id
        self._send: Callable[[int, bytes], bool] = send_function
        self._controller: 'CyberGearController' = controller
        self._debug: bool = controller.debug # Inherit debug state

        # --- Motor State Variables ---
        self._position: float = 0.0
        self._velocity: float = 0.0
        self._effort: float = 0.0
        self._temperature: float = 0.0
        self._last_update_time: Optional[float] = None
        self._enabled: bool = False
        self._current_mode: Optional[RunMode] = None
        self._error_flags: MotorErrorFlags = MotorErrorFlags.NONE
        self._mcu_id: Optional[bytes] = None
        self._read_params: Dict[int, Union[int, float]] = {}
        self._pending_reads: Dict[int, str] = {} # Track pending reads: {index: format_code}

    # --- Properties for State Access ---
    @property
    def motor_id(self) -> int: return self._motor_id
    @property
    def position(self) -> float: return self._position
    @property
    def velocity(self) -> float: return self._velocity
    @property
    def effort(self) -> float: return self._effort
    @property
    def temperature(self) -> float: return self._temperature
    @property
    def is_enabled(self) -> bool: return self._enabled
    @property
    def current_mode(self) -> Optional[RunMode]: return self._current_mode
    @property
    def last_update_time(self) -> Optional[float]: return self._last_update_time
    @property
    def mcu_id(self) -> Optional[bytes]: return self._mcu_id
    @property
    def is_error(self) -> bool: return self._error_flags != MotorErrorFlags.NONE

    def get_error_description(self) -> str:
        """Provides a string describing the current error flags."""
        if not self.is_error: return "Status Normal"
        errors = [flag.name for flag in MotorErrorFlags if flag != MotorErrorFlags.NONE and flag in self._error_flags]
        return "Status Abnormal: " + ", ".join(errors)

    def get_read_parameter(self, address: ParamAddr) -> Optional[Union[int, float]]:
        """Gets the last received value for a parameter requested via read_parameter."""
        return self._read_params.get(address.value)

    # --- Internal Methods ---
    def _construct_arbitration_id(self, cmd_type: CommandType, id_opt: int = 0) -> int:
        """Constructs the 29-bit CAN Arbitration ID."""
        return ( (cmd_type.value & 0x1F) << 24 | (id_opt & 0xFFFF) << 8 | (self._motor_id & 0xFF) )

    def _handle_can_frame(self, msg: can.Message, comm_type: CommandType):
        """Parses relevant incoming CAN frame, updates state."""
        data = msg.data
        data_len = len(data)

        if comm_type == CommandType.MOTOR_FEEDBACK:
            if data_len == 8:
                try:
                    raw_pos = data[0] << 8 | data[1]
                    raw_vel = data[2] << 8 | data[3]
                    raw_eff = data[4] << 8 | data[5]
                    raw_temp = data[6] << 8 | data[7] # Temp*10

                    self._position = self._uint_to_float(raw_pos, self.P_MIN, self.P_MAX)
                    self._velocity = self._uint_to_float(raw_vel, self.V_MIN, self.V_MAX)
                    self._effort = self._uint_to_float(raw_eff, self.T_MIN, self.T_MAX)
                    self._temperature = float(raw_temp) * 0.1 # Apply scaling
                    self._last_update_time = time.monotonic()

                    # Parse error/status from ID field (bits 21-16 of arb ID)
                    id_opt_feedback = (msg.arbitration_id >> 8) & 0xFFFF
                    status_bits = (id_opt_feedback >> 8) & 0xFF # High byte holds status/errors

                    current_errors = MotorErrorFlags.NONE
                    if status_bits & (1 << 0): current_errors |= MotorErrorFlags.MOTOR_VOLTAGE_LOW # bit 16
                    if status_bits & (1 << 1): current_errors |= MotorErrorFlags.MOTOR_OVERCURRENT   # bit 17
                    if status_bits & (1 << 2): current_errors |= MotorErrorFlags.MOTOR_OVERTEMP      # bit 18
                    if status_bits & (1 << 3): current_errors |= MotorErrorFlags.ENCODER_MAG_FAULT   # bit 19
                    if status_bits & (1 << 4): current_errors |= MotorErrorFlags.ENCODER_HALL_FAULT  # bit 20
                    if status_bits & (1 << 5): current_errors |= MotorErrorFlags.MOTOR_NOT_CALIBRATED# bit 21
                    self._error_flags = current_errors

                    mode_val = (status_bits >> 6) & 0x03 # bits 22-23
                    try: self._current_mode = RunMode(mode_val)
                    except ValueError: self._current_mode = None

                    # Update enabled state based on mode (Mode 0 is reset/disabled state)
                    self._enabled = (mode_val != 0)

                except Exception as e: log.error(f"Motor {self._motor_id}: Error parsing feedback frame: {e}", exc_info=self._debug)
            else: log.warning(f"Motor {self._motor_id}: Received feedback frame with unexpected length {data_len}")

        elif comm_type == CommandType.PARAM_READ:
             if data_len >= 8: # Expect Index(2), 00(2), Value(4)
                 try:
                     index_read = data[0] | (data[1] << 8)
                     format_code = self._pending_reads.pop(index_read, 'f') # Get expected format

                     value = None
                     payload = data[4:8] # Assuming value is always in last 4 bytes
                     if format_code == 'f': value = unpack('<f', payload)[0]
                     elif format_code == 'l': value = unpack('<l', payload)[0]
                     elif format_code == 'L': value = unpack('<L', payload)[0]
                     elif format_code == 'H': value = unpack('<H', payload[:2])[0]
                     elif format_code == 'h': value = unpack('<h', payload[:2])[0]
                     elif format_code == 'B': value = unpack('<B', payload[:1])[0]
                     elif format_code == 'b': value = unpack('<b', payload[:1])[0]

                     if value is not None:
                         self._read_params[index_read] = value
                         self._last_update_time = time.monotonic()
                         if self._debug: log.debug(f"Motor {self._motor_id}: Read Param Index=0x{index_read:X}, Value={value}")
                     else: log.warning(f"Motor {self._motor_id}: Unsupported format '{format_code}' for param 0x{index_read:X}")

                 except Exception as e: log.error(f"Motor {self._motor_id}: Error parsing Param Read response: {e}", exc_info=self._debug)
             else: log.warning(f"Motor {self._motor_id}: Received short Param Read response len={data_len}")

        elif comm_type == CommandType.GET_MCU_ID:
             if data_len == 8:
                  self._mcu_id = bytes(data)
                  self._last_update_time = time.monotonic()
                  if self._debug: log.debug(f"Motor {self._motor_id}: Received MCU ID: {self._mcu_id.hex(':')}")
             else: log.warning(f"Motor {self._motor_id}: Received MCU ID response len={data_len}")

        elif comm_type == CommandType.GET_MOTOR_ERROR:
             log.warning(f"Motor {self._motor_id}: Received Dedicated Error Frame. Data: {data.hex(':')}")
             current_errors = self._error_flags # Combine with feedback flags
             if data_len >= 8: # Need at least fault(4) + warning(4)
                 fault_val = unpack('<I', data[0:4])[0] # Fault bits 0-31
                 warning_val = unpack('<I', data[4:8])[0] # Warning bits 0-31

                 if fault_val & (1 << 0): current_errors |= MotorErrorFlags.FAULT_MOTOR_OVERTEMP
                 if fault_val & (1 << 1): current_errors |= MotorErrorFlags.FAULT_DRV_CHIP
                 if fault_val & (1 << 2): current_errors |= MotorErrorFlags.FAULT_VOLTAGE_LOW
                 if fault_val & (1 << 3): current_errors |= MotorErrorFlags.FAULT_VOLTAGE_HIGH
                 if fault_val & (1 << 4): current_errors |= MotorErrorFlags.FAULT_CUR_SAMPLE_B
                 if fault_val & (1 << 5): current_errors |= MotorErrorFlags.FAULT_CUR_SAMPLE_C
                 if fault_val & (1 << 7): current_errors |= MotorErrorFlags.FAULT_ENCODER_NCAL
                 if (fault_val >> 8) & 0xFF: current_errors |= MotorErrorFlags.FAULT_OVERLOAD
                 if fault_val & (1 << 16): current_errors |= MotorErrorFlags.FAULT_CUR_SAMPLE_A

                 if warning_val & (1 << 0): current_errors |= MotorErrorFlags.WARN_MOTOR_OVERTEMP
                 # Add other warning parsings if needed
             self._error_flags = current_errors
             self._last_update_time = time.monotonic()

    # --- Utilities ---
    def _uint_to_float(self, x: int, min_val: float, max_val: float, bits: int = 16) -> float:
        """Converts uint to float based on range and bit width."""
        span = max_val - min_val; max_uint = (1 << bits) - 1
        if span == 0: return min_val
        x = max(0, min(x, max_uint)) # Clamp input uint value
        return min_val + span * (x / max_uint)

    def _float_to_uint(self, x: float, min_val: float, max_val: float, bits: int) -> int:
        """Converts float to uint based on range and bit width."""
        span = max_val - min_val; max_uint = (1 << bits) - 1
        if span == 0: return 0
        x = max(min_val, min(x, max_val)) # Clamp input float value
        val = int(((x - min_val) / span) * max_uint)
        return max(0, min(val, max_uint)) # Clamp result

    # --- Parameter Writing Helper ---
    def _write_param(self, address: ParamAddr, value: Union[int, float], width_code: str) -> bool:
        """Internal helper to send a RAM Write command."""
        try:
            addr_val = address.value
            if not isinstance(addr_val, int): raise TypeError("Address must be integer")
            # Check if writing to non-RAM requires save_parameters() - user responsibility
            if addr_val < 0x7000 and self._debug:
                 log.debug(f"Motor {self._motor_id}: Writing to potentially non-volatile address {address.name}. Use save_parameters() to persist.")

            # Pack data payload (Little Endian address and value)
            if width_code == 'B':   data_bytes = pack('<HxxBxxx', addr_val, int(value))
            elif width_code == 'h': data_bytes = pack('<Hxxhxx', addr_val, int(value))
            elif width_code == 'H': data_bytes = pack('<HxxHxx', addr_val, int(value))
            elif width_code == 'l': data_bytes = pack('<Hxxl', addr_val, int(value))
            elif width_code == 'L': data_bytes = pack('<HxxL', addr_val, int(value))
            elif width_code == 'f': data_bytes = pack('<Hxxf', addr_val, float(value))
            else: raise ValueError(f"Unsupported width code: {width_code}")

            # Construct and send command
            id_opt_write = self._master_can_id
            arbitration_id = self._construct_arbitration_id(CommandType.PARAM_WRITE, id_opt_write)
            return self._send(arbitration_id, data_bytes)
        except Exception as e: log.error(f"Motor {self._motor_id}: Error preparing write_param {address.name}: {e}"); return False

    # --- Public Command Methods ---
    def enable_motor(self) -> bool:
        """Enables the motor."""
        log.info(f"Motor {self._motor_id}: Enabling.")
        arbitration_id = self._construct_arbitration_id(CommandType.ENABLE_MOTOR, self._master_can_id)
        success = self._send(arbitration_id, bytes(8))
        if success: self._enabled = True # Optimistic state update
        return success

    def reset_motor(self) -> bool:
        """Disables the motor. Also referred to as Stop."""
        log.info(f"Motor {self._motor_id}: Disabling/Resetting.")
        arbitration_id = self._construct_arbitration_id(CommandType.DISABLE_MOTOR, self._master_can_id)
        success = self._send(arbitration_id, bytes(8))
        if success: self._enabled = False # Optimistic state update
        return success

    def stop_motor(self) -> bool:
        """Convenience method for reset_motor."""
        return self.reset_motor()

    def clear_error(self) -> bool:
        """Clears motor fault status."""
        log.info(f"Motor {self._motor_id}: Clearing errors.")
        arbitration_id = self._construct_arbitration_id(CommandType.DISABLE_MOTOR, self._master_can_id)
        data = bytes([0x01] + [0] * 7)
        success = self._send(arbitration_id, data)
        if success: self._error_flags = MotorErrorFlags.NONE # Optimistic clear
        return success

    def set_run_mode(self, mode: RunMode) -> bool:
        """Sets the motor's operating mode."""
        log.info(f"Motor {self._motor_id}: Setting run mode to {mode.name}.")
        success = self._write_param(ParamAddr.RUN_MODE, mode.value, 'B')
        # Mode state is updated via feedback, not optimistically here
        return success

    def send_speed_command(self, speed: float) -> bool:
        """Sends a speed command (rad/s) (requires Speed Mode)."""
        log.debug(f"Motor {self._motor_id}: Setting speed to {speed:.2f} rad/s.")
        return self._write_param(ParamAddr.SPEED_REF, speed, 'f')

    def send_position_command(self, position: float) -> bool:
        """Sends a position command (radians) (requires Position Mode)."""
        log.debug(f"Motor {self._motor_id}: Setting position to {position:.2f} rad.")
        return self._write_param(ParamAddr.LOC_REF, position, 'f')

    def send_current_command(self, current: float) -> bool:
        """Sets target current Iq reference (Amperes) (requires Current Mode)."""
        log.debug(f"Motor {self._motor_id}: Setting target current to {current:.2f} A.")
        # Clamp to motor peak current capability
        iq_ref = max(-self.IQ_MAX_PEAK, min(current, self.IQ_MAX_PEAK))
        return self._write_param(ParamAddr.IQ_REF, iq_ref, 'f')

    def set_torque(self, torque: float) -> bool:
        """Sets target torque (Nm) (requires Current Mode). Converts Nm to Iq reference."""
        log.debug(f"Motor {self._motor_id}: Setting target torque to {torque:.2f} Nm.")
        if not self.TORQUE_CONSTANT: log.error("Torque constant is zero."); return False
        # Convert Nm to Iq reference, clamp based on max peak current
        iq_ref = torque / self.TORQUE_CONSTANT
        iq_ref_clamped = max(-self.IQ_MAX_PEAK, min(iq_ref, self.IQ_MAX_PEAK))
        if abs(iq_ref - iq_ref_clamped) > 0.01: log.warning(f"Requested torque {torque:.2f}Nm clamped to Iq={iq_ref_clamped:.2f}A")
        return self._write_param(ParamAddr.IQ_REF, iq_ref_clamped, 'f')

    # --- Parameter Setters ---
    def set_speed_limit(self, limit: float) -> bool: # rad/s
         log.debug(f"Motor {self._motor_id}: Setting speed limit to {limit:.2f} rad/s.")
         return self._write_param(ParamAddr.LIMIT_SPEED, limit, 'f')
    def set_torque_limit(self, limit: float) -> bool: # Nm
         log.debug(f"Motor {self._motor_id}: Setting torque limit to {limit:.2f} Nm.")
         return self._write_param(ParamAddr.LIMIT_TORQUE, max(0.0, min(limit, self.T_MAX)), 'f')
    def set_current_limit(self, limit: float) -> bool: # Amps
         log.debug(f"Motor {self._motor_id}: Setting current limit to {limit:.2f} A.")
         return self._write_param(ParamAddr.LIMIT_CURRENT, max(0.0, min(limit, self.IQ_MAX_LIMIT)), 'f')
    def set_position_control_gain(self, kp: float) -> bool:
         log.debug(f"Motor {self._motor_id}: Setting Pos Kp to {kp:.3f}.")
         return self._write_param(ParamAddr.LOC_KP, kp, 'f')
    def set_velocity_control_gain(self, kp: float, ki: float) -> bool:
         log.debug(f"Motor {self._motor_id}: Setting Speed Kp={kp:.3f}, Ki={ki:.3f}.")
         ok1 = self._write_param(ParamAddr.SPD_KP, kp, 'f')
         time.sleep(0.005); ok2 = self._write_param(ParamAddr.SPD_KI, ki, 'f') # Allow small delay between param writes
         return ok1 and ok2
    def set_current_control_param(self, kp: float, ki: float, filter_gain: float) -> bool:
        log.debug(f"Motor {self._motor_id}: Setting Current Kp={kp:.3f}, Ki={ki:.3f}, Filter={filter_gain:.3f}.")
        ok1=self._write_param(ParamAddr.CURRENT_KP, kp, 'f'); time.sleep(0.005)
        ok2=self._write_param(ParamAddr.CURRENT_KI, ki, 'f'); time.sleep(0.005)
        ok3=self._write_param(ParamAddr.CURRENT_FILTER_GAIN, filter_gain, 'f')
        return ok1 and ok2 and ok3

    # --- Other Commands ---
    def set_mech_position_zero(self) -> bool:
        """Sets current mechanical position as zero offset."""
        log.info(f"Motor {self._motor_id}: Setting mechanical position to zero.")
        arbitration_id = self._construct_arbitration_id(CommandType.SET_MECH_POS_ZERO, self._master_can_id)
        return self._send(arbitration_id, bytes([0x01] + [0] * 7))

    def change_motor_can_id(self, new_can_id: int) -> bool:
        """Requests the motor change its CAN ID. Controller state needs update."""
        new_id_masked = new_can_id & 0xFF
        if not (0 <= new_id_masked <= 127): log.error(f"Invalid new CAN ID {new_id_masked}."); return False
        log.warning(f"Motor {self._motor_id}: Attempting change ID to {new_id_masked}. Controller/object state needs manual update.")
        id_opt_change = (new_id_masked << 8) | self._master_can_id
        arbitration_id = self._construct_arbitration_id(CommandType.SET_CAN_ID, id_opt_change)
        return self._send(arbitration_id, bytes([0x01] + [0] * 7))

    def send_motion_command(self, position: float, velocity: float, torque_ff: float, kp: float, kd: float) -> bool:
        """Sends combined motion command. Uses Big Endian data, Torque in id_opt."""
        log.debug(f"Motor {self._motor_id}: Motion Cmd: P={position:.2f} V={velocity:.2f} Tff={torque_ff:.2f} Kp={kp:.1f} Kd={kd:.2f}")
        try:
            tff_int = self._float_to_uint(torque_ff, self.T_MIN, self.T_MAX, 16)
            id_opt_motion = tff_int # Torque feedforward goes into id_opt (bits 23-8)

            pos_int = self._float_to_uint(position, self.P_MIN, self.P_MAX, 16)
            vel_int = self._float_to_uint(velocity, self.V_MIN, self.V_MAX, 16)
            kp_int = self._float_to_uint(kp, self.KP_MIN, self.KP_MAX, 16)
            kd_int = self._float_to_uint(kd, self.KD_MIN, self.KD_MAX, 16)

            # Pack data: Pos, Vel, Kp, Kd (Big Endian format based on C example/docs)
            data = bytes([
                (pos_int >> 8) & 0xFF, pos_int & 0xFF,
                (vel_int >> 8) & 0xFF, vel_int & 0xFF,
                (kp_int >> 8) & 0xFF, kp_int & 0xFF,
                (kd_int >> 8) & 0xFF, kd_int & 0xFF,
            ])
            arbitration_id = self._construct_arbitration_id(CommandType.MOTION_COMMAND, id_opt_motion)
            return self._send(arbitration_id, data)
        except Exception as e: log.error(f"Motor {self._motor_id}: Error sending motion command: {e}"); return False

    # --- Parameter Read/Save/ID Methods ---
    def read_parameter(self, address: ParamAddr, data_type_code: str = 'f') -> bool:
        """Requests reading a parameter. Check value later with get_read_parameter()."""
        log.debug(f"Motor {self._motor_id}: Requesting read param {address.name} (Type: {data_type_code})")
        try:
            addr_val = address.value
            if not isinstance(addr_val, int): raise TypeError("Address must be integer")

            cmd_type = CommandType.PARAM_READ
            # Parameter index goes into data bytes 0-1 (Little Endian)
            req_data = bytes([addr_val & 0xFF, (addr_val >> 8) & 0xFF] + [0]*6)
            self._pending_reads[addr_val] = data_type_code # Track expected format for response

            id_opt_read = self._master_can_id # Master ID goes in id_opt low byte
            arbitration_id = self._construct_arbitration_id(cmd_type, id_opt_read)
            return self._send(arbitration_id, req_data)
        except Exception as e: log.error(f"Motor {self._motor_id}: Error preparing read_parameter: {e}"); return False

    def save_parameters(self) -> bool:
        """Saves parameters from RAM to Flash."""
        log.info(f"Motor {self._motor_id}: Sending Save Parameters command.")
        # id_opt high byte = 0x02 for save command according to C examples
        id_opt_save = (0x02 << 8) | self._master_can_id
        arbitration_id = self._construct_arbitration_id(CommandType.FLASH_COMMAND, id_opt_save)
        return self._send(arbitration_id, bytes(8)) # Data unused

    def request_mcu_id(self) -> bool:
        """Requests the motor's MCU ID. Check value later with mcu_id property."""
        log.debug(f"Motor {self._motor_id}: Requesting MCU ID.")
        arbitration_id = self._construct_arbitration_id(CommandType.GET_MCU_ID, self._master_can_id)
        return self._send(arbitration_id, bytes(8)) # Data unused