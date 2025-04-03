# PyCyberGear: Xiaomi CyberGear Motor Controller for Python

[![PyPI version](https://badge.fury.io/py/pycybergear.svg)](https://badge.fury.io/py/pycybergear) <!-- Optional: Add if you publish -->
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) <!-- Optional: Update license -->

This repository contains a Python library for controlling Xiaomi CyberGear motors over a CAN bus using the excellent `python-can` package. It provides an object-oriented interface to send commands (position, speed, current/torque), configure parameters, and monitor motor status asynchronously.

**Disclaimer**: This is an **unofficial library** and is not affiliated with or endorsed by Xiaomi. Use at your own risk. It was developed referencing the communication protocol derived from sources like the [M5 Stack CyberGear library](https://github.com/project-sternbergia/cybergear_m5) and available documentation.

## Features

- Object-oriented interface (`CyberGearController`, `Motor`).
- Asynchronous CAN message handling via `python-can` Notifier.
- Control modes: Position, Speed, Current (Torque).
- Combined Motion Command support (`send_motion_command`).
- Parameter reading (`read_parameter`) and writing (`write_param`).
- Parameter saving to flash (`save_parameters`).
- Motor enabling (`enable_motor`), disabling (`reset_motor`/`stop_motor`), zeroing (`set_mech_position_zero`), ID changing (`change_motor_can_id`).
- Error flag reporting (`is_error`, `get_error_description`) and clearing (`clear_error`).
- CAN ID scanning (`scan_for_motors`, `ping_motor`).
- Uses standard Python `logging` module.
- Designed for `python-can` compatible interfaces (SocketCAN, slcan, PCAN, Kvaser, etc.).

## Installation

Requires Python 3.8 or higher.

1.  **Install `python-can` and backend dependencies:**

    ```bash
    pip install "python-can[slcan,socketcan]" # Install desired backends (e.g., slcan, socketcan)
    # Or: pip install python-can # Install base, ensure backend drivers are present
    ```

2.  **Install PyCyberGear:**

    - **From PyPI (Recommended, if published):**
      ```bash
      pip install pycybergear # Or your chosen package name
      ```
    - **Directly from GitHub (Latest Development):**
      ```bash
      pip install git+https://github.com/your_username/pycybergear.git # *** UPDATE URL ***
      ```
    - **From Local Clone:**
      ```bash
      git clone https://github.com/your_username/pycybergear.git # *** UPDATE URL ***
      cd pycybergear
      pip install .
      ```

## Library Structure

- **`CyberGearController`**: Manages the CAN bus connection (using `python-can`), starts/stops communication, discovers motors (`scan_for_motors`), and creates/provides access to `Motor` objects (`add_motor`, `get_motor`). Use this as the main entry point.
- **`Motor`**: Represents a single CyberGear motor. Holds the motor's state (position, velocity, temperature, errors, etc.) updated asynchronously. Provides methods to send commands (`enable_motor`, `send_speed_command`, etc.) and configure parameters (`set_run_mode`, `write_param`, etc.).
- **Enums**: (`RunMode`, `ParamAddr`, `MotorErrorFlags`) for constants.
- **Exceptions**: Custom exceptions for error handling.

## Basic Usage Workflow

The typical workflow is:

1.  Create a `CyberGearController` instance (ideally using a `with` statement for automatic cleanup).
2.  Start the controller (automatic with `with`, or call `controller.start()`).
3.  Identify target motor IDs (either known beforehand or using `controller.scan_for_motors()`).
4.  Get the specific `Motor` object instance using `controller.add_motor(motor_id)`. `add_motor` can be called multiple times for the same ID; it will return the existing instance.
5.  Call command methods directly on the `Motor` object (e.g., `motor.enable_motor()`, `motor.send_speed_command(5.0)`).
6.  Access motor state via properties on the `Motor` object (e.g., `motor.position`, `motor.velocity`, `motor.is_error`). State is updated in the background.
7.  Stop the controller (automatic with `with`, or call `controller.close()`).

## Examples

```python
import time
import logging
from pycybergear import CyberGearController, Motor, RunMode, CyberGearError, ParamAddr

# Configure logging level for more details (e.g., DEBUG for TX/RX)
logging.basicConfig(level=logging.INFO)
log = logging.getLogger(__name__)

# Replace with your actual CAN interface details
CAN_INTERFACE = 'slcan'
CAN_CHANNEL = '/dev/ttyACM0' # Example for slcan adapter
# CAN_INTERFACE = 'socketcan'
# CAN_CHANNEL = 'can0'      # Example for socketcan

MASTER_ID = 0 # Host CAN ID

try:
    # Use context manager for automatic start/stop/cleanup
    with CyberGearController(interface=CAN_INTERFACE, channel=CAN_CHANNEL,
                             master_can_id=MASTER_ID, debug=False) as controller:

        log.info("Scanning for motors (IDs 1-5)...")
        # Reduce timeout for faster scans if bus is reliable
        detected_ids = controller.scan_for_motors(scan_range=range(1, 6), ping_timeout=0.05)

        if not detected_ids:
            log.warning("No motors detected.")
            exit()

        # --- Get motor object for the first detected ID ---
        motor_id = detected_ids[0]
        log.info(f"Adding/Retrieving motor object for ID {motor_id:#04x}")
        motor1 = controller.add_motor(motor_id)

        # --- Enable and Check Status ---
        log.info("Enabling motor...")
        if not motor1.enable_motor():
            raise MotorError("Failed to enable motor")
        time.sleep(0.5) # Allow time for motor to enable and report status

        if motor1.is_error:
            log.error(f"Motor started with error: {motor1.get_error_description()}")
            log.info("Attempting to clear error...")
            motor1.clear_error()
            time.sleep(0.1)
            if not motor1.enable_motor(): # Retry enable after clear
                 raise MotorError("Failed to enable motor after clearing error")
            time.sleep(0.5)

        log.info(f"Motor Enabled. Current State: Pos={motor1.position:.2f} Vel={motor1.velocity:.2f} Mode={motor1.current_mode}")

        # --- Speed Control Example ---
        log.info("Setting Speed Mode...")
        motor1.set_run_mode(RunMode.SPEED)
        time.sleep(0.05) # Short delay after mode change command

        log.info("Setting speed to 3.0 rad/s")
        motor1.send_speed_command(3.0)
        start_time = time.time()
        while time.time() - start_time < 2.0: # Run for 2 seconds
             # Status is updated in the background, just read properties
             log.info(f" Status: V={motor1.velocity:.2f} T={motor1.effort:.2f} Err={motor1.is_error}")
             if motor1.is_error: break # Stop on error
             time.sleep(0.1)

        log.info("Setting speed to 0.0 rad/s")
        motor1.send_speed_command(0.0)
        time.sleep(0.5) # Wait to slow down

        # --- Parameter Read Example ---
        log.info("Reading LIMIT_TORQUE parameter...")
        if motor1.read_parameter(ParamAddr.LIMIT_TORQUE, 'f'):
            time.sleep(0.1) # Wait for response
            limit = motor1.get_read_parameter(ParamAddr.LIMIT_TORQUE)
            log.info(f"Read LIMIT_TORQUE = {limit}")
        else:
            log.warning("Failed to send read parameter command.")

        # --- Disable ---
        log.info("Disabling motor...")
        motor1.reset_motor() # Or motor1.reset_motor()
        time.sleep(0.1)

except CyberGearError as e:
    log.error(f"Operation failed: {e}")
except KeyboardInterrupt:
    log.info("Operation interrupted by user.")
except Exception as e:
    log.error(f"An unexpected error occurred: {e}", exc_info=True)

log.info("Program finished.")

```

## Connecting the Motor

This library uses `python-can`, so you need a compatible backend driver and interface configuration.

### 1. `socketcan` on Linux/Raspberry Pi

Configure your `can0` (or other) interface, e.g., in `/etc/network/interfaces.d/can0`:

```
allow-hotplug can0
iface can0 can static
    bitrate 1000000 # Match your motor bitrate
    up /sbin/ip link set $IFACE type can restart-ms 100
    # Optional: Increase queue length
    # up /sbin/ifconfig $IFACE txqueuelen 1000
```

Bring it up: `sudo ip link set can0 up type can bitrate 1000000`
Initialize: `CyberGearController(interface='socketcan', channel='can0', ...)`

### 2. `slcan` Adapters (e.g., CANable, USB2CAN)

Many USB-to-CAN adapters emulate the SLCAN (Serial Line CAN) ASCII protocol.

- Find the serial port device name (e.g., `/dev/ttyACM0`, `/dev/ttyUSB0`, `COM3`).
- Ensure no other process (like `slcand`) is using the port. `python-can` handles the protocol directly.
- Initialize: `CyberGearController(interface='slcan', channel='/dev/ttyACM0', bitrate=1000000, ...)`

### 3. Other Interfaces (PCAN, Kvaser, Vector, etc.)

- Install the necessary drivers and `python-can` backend support for your specific hardware.
- Consult the `python-can` documentation for the correct `interface` and `channel` names. Example: `CyberGearController(interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000, ...)`

## API Reference

_(Optional: Consider adding more detailed API documentation here or linking to generated docs using Sphinx)_

### `CyberGearController` Methods

- `__init__(...)`: Initialize controller and CAN settings.
- `start()`: Connect to CAN bus and start listener (used by `__enter__`).
- `close()`: Stop listener and shutdown bus (used by `__exit__`).
- `add_motor(motor_id)`: Get/create a `Motor` instance for a given ID.
- `get_motor(motor_id)`: Get an existing `Motor` instance.
- `scan_for_motors(...)`: Scan a range of CAN IDs.
- `ping_motor(...)`: Ping a single CAN ID.

### `Motor` Properties (Read-Only State)

- `motor_id`: Motor's CAN ID.
- `position`: Last reported position (radians).
- `velocity`: Last reported velocity (rad/s).
- `effort`: Last reported effort/torque (Nm).
- `temperature`: Last reported temperature (Celsius).
- `is_enabled`: Boolean indicating likely enabled state.
- `current_mode`: Last known `RunMode` Enum.
- `last_update_time`: `time.monotonic()` timestamp of last feedback.
- `is_error`: Boolean indicating if any error flags are set.
- `get_error_description()`: String detailing active errors.
- `get_read_parameter(address)`: Gets value from last successful parameter read.

### `Motor` Methods (Commands)

- `enable_motor()`
- `disable()` / `reset_motor()` # Send Type 4, data 0
- `stop_motor()` # Convenience wrapper for reset_motor() maybe? Or add separate logic if needed.
- `clear_error()` # Send Type 4, data 1
- `set_run_mode(mode: RunMode)`
- `send_speed_command(speed: float)`
- `send_position_command(position: float)`
- `send_current_command(current: float)` # Sends IQ_REF (Amps)
- `set_torque(torque: float)` # Calculates IQ_REF from torque
- `set_speed_limit(limit: float)`
- `set_torque_limit(limit: float)`
- `set_current_limit(limit: float)`
- `set_position_control_gain(kp: float)` # Alias for set_position_kp
- `set_velocity_control_gain(kp: float, ki: float)` # Alias for set_speed_gains
- `set_current_control_param(kp: float, ki: float, filter_gain: float)` # Alias for set_current_gains
- `set_mech_position_zero()`
- `change_motor_can_id(new_can_id: int)`
- `send_motion_command(position: float, velocity: float, torque_ff: float, kp: float, kd: float)`
- `read_parameter(address: ParamAddr, data_type_code: str)`
- `write_param(address: ParamAddr, value: Union[int, float], width_code: str)` # Usually call specific setters instead
- `save_parameters()`

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions, bug reports, and feature requests are welcome! Please open an issue or submit a pull request on the GitHub repository.

## Acknowledgments

- **python-can**: Core dependency for CAN communication.
- **M5 Stack CyberGear Library**: Referenced for protocol understanding.
