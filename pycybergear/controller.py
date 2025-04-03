# src/pycybergear/controller.py

import can
import time
import threading
import logging
from typing import Optional, Dict, List

# Use relative imports within the package
from .constants import CommandType
from .exceptions import CyberGearError, CANError
from .motor import Motor

log = logging.getLogger(__name__)

class CyberGearController:
    """Manages CAN bus communication and CyberGear motor instances."""

    # Constants can be accessed via Motor class as well
    P_MIN, P_MAX = Motor.P_MIN, Motor.P_MAX
    V_MIN, V_MAX = Motor.V_MIN, Motor.V_MAX
    T_MIN, T_MAX = Motor.T_MIN, Motor.T_MAX

    def __init__(self, interface: str = 'slcan', channel: str = '/dev/ttyACM0',
                 bitrate: int = 1000000, master_can_id: int = 0, debug: bool = False):
        """
        Initializes the CAN bus controller.

        Args:
            interface: python-can interface type (e.g., 'socketcan', 'slcan', 'pcan').
            channel: python-can channel identifier (e.g., 'can0', '/dev/ttyACM0', 'PCAN_USBBUS1').
            bitrate: CAN bus bitrate (e.g., 1000000).
            master_can_id: The CAN ID used by this controller instance (0-255).
            debug: Enable detailed debug logging.
        """
        self._bus_interface = interface
        self._bus_channel = channel
        self._bus_bitrate = bitrate
        self.master_can_id = master_can_id & 0xFF
        self.debug = debug
        if self.debug:
            # Set logger level for this module if debug is enabled for the controller
            # Avoids changing global logging level
            logging.getLogger('pycybergear').setLevel(logging.DEBUG)

        self._bus: Optional[can.BusABC] = None
        self._motors: Dict[int, Motor] = {}
        self._can_lock = threading.Lock() # Lock for shared bus access (send/recv in ping) and _motors dict
        self._notifier: Optional[can.Notifier] = None
        self._is_running = False
        log.info(f"CyberGearController initialized (Master ID: {self.master_can_id})")

    def _message_received(self, msg: can.Message):
        """Callback function for can.Notifier. Routes messages to Motor instances."""
        if not msg.is_extended_id: return # Ignore standard frames
        if self.debug: log.debug(f'RX Raw: id=0x{msg.arbitration_id:08X} data={msg.data.hex(":")}')

        comm_type_val = (msg.arbitration_id >> 24) & 0x1F
        try: comm_type = CommandType(comm_type_val)
        except ValueError:
             if self.debug: log.debug(f"RX: Unknown comm type {comm_type_val}")
             return

        # Determine which motor this message is relevant to based on documented response ID structure
        responding_motor_id = -1
        if comm_type in [CommandType.MOTOR_FEEDBACK, CommandType.DISABLE_MOTOR, CommandType.ENABLE_MOTOR, CommandType.PARAM_WRITE]:
             # These responses confirm action on target, ID is likely Target ID
             responding_motor_id = msg.arbitration_id & 0xFF # Bits 7-0
        elif comm_type in [CommandType.GET_MCU_ID, CommandType.GET_MOTOR_ERROR, CommandType.PARAM_READ]:
             # These responses likely contain the sender's ID (the motor) in bits 15-8
             responding_motor_id = (msg.arbitration_id >> 8) & 0xFF # Bits 15-8
        # Add logic for other response types if their ID structure differs

        if responding_motor_id == -1:
            if self.debug: log.debug(f"RX: Could not determine motor ID for type {comm_type.name}, ArbID=0x{msg.arbitration_id:08X}")
            return

        # Route frame to the correct motor instance if it exists
        # Locking only needed if motors can be added/removed while listener runs
        # with self._can_lock:
        motor = self._motors.get(responding_motor_id)

        if motor:
            try: motor._handle_can_frame(msg, comm_type)
            except Exception as e: log.error(f"Error processing frame for motor {responding_motor_id}: {e}", exc_info=self.debug)
        elif self.debug:
            log.debug(f"RX: Received frame for uninitialized motor ID {responding_motor_id}")

    def _send_can_message(self, arbitration_id: int, data: bytes) -> bool:
        """Internal method to send a single CAN message."""
        if not self._is_running or not self._bus: log.error("CAN bus not running."); return False
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        success = False
        with self._can_lock: # Ensure sending is thread-safe
            try:
                self._bus.send(msg)
                if self.debug: log.debug(f'TX: id=0x{msg.arbitration_id:08X} data={msg.data.hex(":")}')
                success = True
            except can.CanError as e: log.error(f"CAN Send Error (ID: 0x{arbitration_id:08X}): {e}")
            except Exception as e: log.error(f"Unexpected CAN Send Error (ID: 0x{arbitration_id:08X}): {e}", exc_info=self.debug)
        return success

    def add_motor(self, motor_id: int) -> Motor:
        """
        Creates (if needed) and returns a Motor instance for the given CAN ID.

        Args:
            motor_id: The CAN ID of the motor (0-127 recommended).

        Returns:
            The corresponding Motor instance.
        """
        motor_id_masked = motor_id & 0xFF
        if not (0 <= motor_id_masked <= 127): log.warning(f"Motor ID {motor_id_masked} outside typical 0-127 range.")

        with self._can_lock: # Protect dictionary modification
            if motor_id_masked in self._motors:
                if self.debug: log.debug(f"Returning existing instance for motor {motor_id_masked}")
                return self._motors[motor_id_masked]

            log.info(f"Creating new instance for motor ID {motor_id_masked}")
            motor = Motor(
                motor_id=motor_id_masked,
                master_can_id=self.master_can_id,
                send_function=self._send_can_message, # Inject send method
                controller=self
            )
            self._motors[motor_id_masked] = motor
            return motor

    def get_motor(self, motor_id: int) -> Optional[Motor]:
        """Gets a previously added motor instance, returns None if not found."""
        with self._can_lock:
            return self._motors.get(motor_id & 0xFF)

    def start(self):
        """Connects to the CAN bus and starts the background listener thread."""
        if self._is_running:
            log.warning("CAN controller already running.")
            return

        # Ensure resources are clean before starting
        if self._bus or self._notifier:
            log.warning("Attempting start on potentially unclean state. Forcing close first.")
            self.close()

        try:
            log.info(f"Connecting to CAN bus: {self._bus_interface} / {self._bus_channel} @ {self._bus_bitrate} bps...")
            self._bus = can.interface.Bus(
                interface=self._bus_interface,
                channel=self._bus_channel,
                bitrate=self._bus_bitrate,
                receive_own_messages=False
            )
            log.debug("Bus instance created.")

            self._notifier = can.Notifier(self._bus, [self._message_received], timeout=0.1)
            self._is_running = True
            log.info("CAN Controller started successfully (Bus & Notifier running).")

            # Allow a brief moment for backend/notifier stabilization
            time.sleep(0.05)

        except can.CanError as e:
            if self._notifier: self._notifier.stop(); self._notifier = None
            if self._bus: self._bus.shutdown(); self._bus = None
            self._is_running = False
            log.error(f"Failed to connect/start CAN bus or notifier: {e}")
            raise CANError(f"Failed to connect/start CAN bus or notifier: {e}") from e
        except Exception as e:
            if self._notifier: self._notifier.stop(); self._notifier = None
            if self._bus: self._bus.shutdown(); self._bus = None
            self._is_running = False
            log.error(f"Failed to start CAN controller: {e}", exc_info=self.debug)
            raise CyberGearError(f"Failed to start CAN controller: {e}") from e


    def close(self):
        """Stops the listener thread and shuts down the CAN bus connection."""
        log.info("Shutting down CAN Controller...")
        self._is_running = False

        active_notifier = self._notifier
        if active_notifier:
            log.debug("Stopping CAN Notifier thread (timeout=1.5s)...")
            start_stop = time.monotonic()
            try:
                active_notifier.stop(timeout=1.5)
                stop_duration = time.monotonic() - start_stop
                if stop_duration >= 1.49: # Check if timeout was likely hit
                     log.warning("CAN Notifier stop may have timed out.")
                else:
                     log.info(f"CAN Notifier stopped successfully in {stop_duration:.2f}s.")
            except Exception as e:
                 log.error(f"Error stopping CAN Notifier: {e}", exc_info=self.debug)
            finally:
                 self._notifier = None

        active_bus = self._bus
        if active_bus:
            bus_instance_name = active_bus.__class__.__name__
            log.debug(f"Shutting down CAN bus instance ({bus_instance_name})...")
            try:
                active_bus.shutdown()
                log.info("CAN bus shut down.")
            except Exception as e:
                log.error(f"Error during CAN bus shutdown: {e}", exc_info=self.debug)
            finally:
                self._bus = None

        # Internal delay for potential OS/driver resource release lag
        internal_delay = 0.2
        if self.debug: log.debug(f"Adding internal delay of {internal_delay}s for resource release.")
        time.sleep(internal_delay)

        with self._can_lock:
            self._motors.clear()
            log.debug("Internal motor dictionary cleared.")

        log.info("Controller shutdown complete.")


    def ping_motor(self, motor_id_to_ping: int, timeout: float = 0.1) -> bool:
        """Synchronously pings a motor ID and waits for a Type 0 response."""
        if not self._is_running: raise CANError("Controller not started.")
        target_id = motor_id_to_ping & 0xFF
        cmd_type = CommandType.GET_MCU_ID
        id_opt = self.master_can_id
        arbitration_id = (cmd_type.value & 0x1F) << 24 | (id_opt & 0xFFFF) << 8 | target_id

        if not self._send_can_message(arbitration_id, bytes(8)): return False

        start_time = time.time()
        while time.time() - start_time < timeout:
            # Temporarily use direct recv for synchronous ping response check
            with self._can_lock: response = self._bus.recv(timeout=0.01)
            if response and response.is_extended_id:
                 response_cmd_type_val = (response.arbitration_id >> 24) & 0x1F
                 responding_id_in_opt = (response.arbitration_id >> 8) & 0xFF
                 if response_cmd_type_val == cmd_type.value and responding_id_in_opt == target_id:
                     if self.debug: log.debug(f"Ping SUCCESS for ID {target_id}")
                     return True
        if self.debug: log.debug(f"Ping TIMEOUT for ID {target_id}")
        return False

    def scan_for_motors(self, scan_range: range = range(1, 128), ping_timeout: float = 0.05) -> List[int]:
        """Scans for motors using synchronous ping."""
        if not self._is_running: raise CANError("Controller not started.")
        detected_ids_int = []
        print(f"Scanning IDs {scan_range.start} to {scan_range.stop - 1}: ", end='', flush=True)
        for motor_id_to_scan in scan_range:
            print(".", end='', flush=True)
            # Shorten timeout for scan pings
            if self.ping_motor(motor_id_to_scan, timeout=ping_timeout):
                detected_ids_int.append(motor_id_to_scan)
        print(" Done.")
        sorted_detected_ids_int = sorted(detected_ids_int)
        detected_ids_hex = [f"0x{motor_id:02x}" for motor_id in sorted_detected_ids_int]
        if not detected_ids_hex: print("No motors detected.")
        else: print(f"Detected: {detected_ids_hex}")
        return sorted_detected_ids_int

    def __enter__(self): self.start(); return self
    def __exit__(self, exc_type, exc_val, exc_tb): self.close()