# Xiaomi CyberGear Motor Controller for Python

[![PyPI version](https://badge.fury.io/py/pycybergear.svg)](https://badge.fury.io/py/pycybergear) <!-- Optional: Add if you publish -->
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) <!-- Optional: Update license -->

This repository contains a Python library for controlling Xiaomi CyberGear motors over a CAN bus using the `python-can` package. It provides a straightforward interface to initialize the connection, discover motors, and send commands to individual motors for position, speed, or current/torque control, as well as reading back status information.

**Disclaimer**: This is an **unofficial library** and is not affiliated with or endorsed by Xiaomi. Use at your own risk. Protocol information was derived from sources like the [M5 Stack CyberGear library](https://github.com/project-sternbergia/cybergear_m5) and available [documentation](https://github.com/belovictor/cybergear-docs).

## Installation

Requires Python 3 and `python-can`.

1.  **Install `python-can` and backend dependencies:**

    ```bash
    pip install python-can
    ```

2.  **Install:**

    - **From PyPI (Recommended, if published):**
      ```bash
      pip install pycybergear
      ```
    - **Directly from GitHub (Latest Development):**
      ```bash
      pip install git+https://github.com/blanck/cybergear.git
      ```
    - **From Local Clone:**
      ```bash
      git clone https://github.com/blanck/cybergear.git
      cd cybergear
      pip install .
      ```

## Library Structure

The library uses two main classes:

1.  **`CyberGear`**: This class acts as the **controller**. You instantiate it once to establish the connection to your CAN bus interface (`slcan`, `socketcan`, etc.). It manages the overall communication channel and provides methods to discover connected motors (`scan_for_motors`, `ping_motor`) and to create `Motor` objects (`init_motor`) for interaction. Remember to call `close()` on this object when done, or use it as a context manager (`with CyberGear(...) as cg:`).

2.  **`Motor`**: This class represents a **single physical CyberGear motor**. You obtain instances of this class using `CyberGear.init_motor(motor_id)`. Each `Motor` object is tied to a specific CAN ID and holds the last known status (position, velocity, etc.) for that motor, updated via calls to `CyberGear.process_packet()`. You call methods directly on the `Motor` object to send commands (e.g., `motor.enable_motor()`, `motor.send_speed_command(5.0)`) or configure parameters for that specific motor.

## API Reference

_Note: Command methods generally return `True` if the CAN message was sent successfully by `python-can`, and `False` on immediate send errors. This does not guarantee the motor received or executed the command._

### `CyberGear` Class

Handles the CAN bus connection and motor discovery/initialization.

- `__init__(self, interface: str = 'slcan', channel: str = ..., bitrate: int = 1000000, master_can_id: int = 0, debug: bool = False)`

  - Initializes the controller and connects to the specified CAN interface.
  - `master_can_id`: The CAN ID used by this controller (host), defaults to 0.

  ```python
  from cybergear import CyberGear
  # Example: Connect via slcan adapter on COM3 with Master ID 0
  cg = CyberGear(interface='slcan', channel='COM3', master_can_id=0)
  motor1 = cg.init_motor(0x7F)
  # Rest of code
  ```

  - Or use the 'with' statement for automatic cleanup:

  ```python
  from cybergear import CyberGear
  # Example: Connect socketcan on linux/Raspberry Pi
  with CyberGear(interface='socketcan', channel='can0') as cg:
    motor1 = cg.init_motor(0x7F)
    # Rest of code
  ```

- `init_motor(self, motor_id: int) -> Motor`

  - Creates or retrieves a `Motor` object instance for the specified CAN ID, using the controller's `master_can_id`. Returns the `Motor` object.

  ```python
  # Assumes cg is an initialized CyberGear object
  # Initialize motor with default ID 0x7F
  motor1 = cg.init_motor(0x7F)
  ```

- `ping_motor(self, motor_id_to_ping: int, timeout: float = 0.1)`

  - Sends a specific request (Get MCU ID) to a motor ID and waits for a valid response. Returns `True` if responsive, `False` otherwise.

  ```python
  # Assumes cg is an initialized CyberGear object
  if cg.ping_motor(0x7F):
      print("Motor 0x7F is responding.")
  else:
      print("Motor 0x7F did not respond.")
  ```

- `scan_for_motors(self, scan_range: range = range(1, 128), ping_timeout: float = 0.05) -> List[int]`

  - Pings all IDs in the specified range and returns a list of integer IDs that responded.
  - Prints progress dots and a summary of detected IDs (as hex). Returns a list of integers.

  ```python
  # Assumes cg is an initialized CyberGear object
  detected_ids = cg.scan_for_motors(scan_range=range(125, 128), ping_timeout=0.02)
  # Output might look like:
  # Scanning IDs 125 to 127: ... Done.
  # Detected: ['0x7f']
  # motor1 = cg.init_motor(detected_ids[0])
  ```

- `process_packet(self) -> bool`

  - Reads one available CAN message from the bus and updates the corresponding `Motor` object's status if the message is recognized feedback for an initialized motor.
  - **Important:** Call this frequently in your main loop to keep motor status (`motor.position`, `motor.velocity`, etc.) up-to-date. See **Example 5** below for usage in a control loop.
  - Returns `True` if a relevant status update was processed, `False` otherwise.

- `get_motor_status(self, motor_id: int) -> Optional[Tuple]`

  - Returns the _last known_ status tuple for the specified motor ID stored within its `Motor` object: `(motor_id, position, velocity, effort, temperature)`.
  - Returns `None` if the motor ID hasn't been initialized.
  - **Note:** Relies on recent calls to `process_packet()` for updated values.

  ```python
  # Assumes cg is an initialized CyberGear object
  status = cg.get_motor_status(0x7F)
  if status:
      mid, pos, vel, eff, temp = status
      print(f"Motor {mid:#04x} Pos: {pos:.2f} rad")
  ```

- `close(self)`:
  - Shuts down the CAN bus connection. Call this when finished, or use the `with` statement.
  ```python
  # Assumes cg is an initialized CyberGear object
  cg.close()
  ```

### `Motor` Class

Represents and controls a single motor. Get instances via `CyberGear.init_motor()`.

- `enable_motor(self)`

  - Sends the command to enable motor power output.

  ```python
  # Assumes motor1 = cg.init_motor(0x7F)
  motor1.enable_motor()
  ```

- `reset_motor(self)`

  - Sends the command to disable motor power output (also referred to as stop).

  ```python
  # Assumes motor1 is an initialized Motor object
  motor1.reset_motor()
  ```

- `stop_motor(self)`

  - Alias for `reset_motor()`. Sends the command to disable motor power output.

  ```python
  # Assumes motor1 is an initialized Motor object
  motor1.stop_motor()
  ```

- `set_run_mode(self, mode: int)`

  - Sets the active control mode. Use defined constants (accessed via `CyberGear` class or imported directly):
    - `CyberGear.MODE_MOTION` (0): Control motion mode.
    - `CyberGear.MODE_POSITION` (1): Control target angle (radians).
    - `CyberGear.MODE_SPEED` (2): Control target velocity (rad/s).
    - `CyberGear.MODE_CURRENT` (3): Control target Iq current (Amperes) / torque.

  ```python
  # Assumes motor1 is an initialized Motor object
  # Import CyberGear or access constant through initialized cg object
  from cybergear import CyberGear
  motor1.set_run_mode(CyberGear.MODE_SPEED)
  # Or: motor1.set_run_mode(cg.MODE_SPEED)
  ```

- `send_speed_command(self, speed: float)`

  - Sends a target speed command (radians/second). Requires Speed Mode (`MODE_SPEED`).

  ```python
  # Assumes motor1 is an initialized Motor object
  motor1.send_speed_command(10.0) # 10 rad/s
  ```

- `send_position_command(self, position: float)`

  - Sends a target position command (radians). Requires Position Mode (`MODE_POSITION`).

  ```python
  # Assumes motor1 is an initialized Motor object
  # Go to approx 90 degrees
  motor1.send_position_command(1.57)
  ```

- `send_current_command(self, current: float)`

  - Sends a target Iq current command (Amperes). Requires Current Mode (`MODE_CURRENT`).

  ```python
  # Assumes motor1 is an initialized Motor object
  motor1.send_current_command(1.5) # Target 1.5A Iq
  ```

- `set_torque(self, torque: float)`

  - Sends a target torque command (Nm) by calculating the equivalent Iq current. Requires Current Mode (`MODE_CURRENT`).

  ```python
  # Assumes motor1 is an initialized Motor object
  motor1.set_torque(0.5) # Target 0.5 Nm
  ```

- `send_motion_command(self, position: float, velocity: float, torque: float, kp: float, kd: float)`

  - Sends a combined trajectory command (radians, rad/s target velocity, Nm torque feedforward, Kp gain, Kd gain). Requires a specific mode (often Mode 0, verify documentation).

  ```python
  # Assumes motor1 is an initialized Motor object
  # Example: Move towards 0 rad with velocity 1 rad/s, 0.1 Nm feedforward torque, specific gains
  motor1.send_motion_command(position=0.0, velocity=1.0, torque=0.1, kp=5.0, kd=0.1)
  ```

- `set_speed_limit(self, limit: float)`

  - Sets the speed limit parameter (radians/second). Primarily affects Position Mode moves.

  ```python
  # Assumes motor1 is an initialized Motor object
  motor1.set_speed_limit(5.0) # Limit position moves to 5 rad/s
  ```

- `set_torque_limit(self, limit: float)`

  - Sets the torque limit parameter (Nm).

  ```python
  # Assumes motor1 is an initialized Motor object
  motor1.set_torque_limit(8.0) # Limit torque output to 8 Nm
  ```

- `set_current_limit(self, limit: float)`

  - Sets the phase current limit parameter (Amperes).

  ```python
  # Assumes motor1 is an initialized Motor object
  motor1.set_current_limit(20.0) # Limit phase current to 20 A
  ```

- `set_position_control_gain(self, kp: float)`

  - Sets the Kp gain for the position controller.

  ```python
  # Assumes motor1 is an initialized Motor object
  motor1.set_position_control_gain(35.0)
  ```

- `set_velocity_control_gain(self, kp: float, ki: float)`

  - Sets the Kp and Ki gains for the velocity controller.

  ```python
  # Assumes motor1 is an initialized Motor object
  motor1.set_velocity_control_gain(kp=2.5, ki=0.05)
  ```

- `set_current_control_param(self, kp: float, ki: float, filter_gain: float)`

  - Sets the Kp, Ki, and filter gain for the current controller.

  ```python
  # Assumes motor1 is an initialized Motor object
  motor1.set_current_control_param(kp=0.07, ki=0.07, filter_gain=0.1)
  ```

- `set_mech_position_to_zero(self)`

  - Defines the current motor position as the mechanical zero offset (this setting is lost on power down).

  ```python
  # Assumes motor1 is an initialized Motor object
  motor1.set_mech_position_to_zero()
  ```

- `change_motor_can_id(self, new_can_id: int)`

  - Sends the command to permanently change the motor's CAN ID. Use with caution! The `Motor` object becomes invalid after this.

  ```python
  # Example: Change default motor ID (0x7F) to 0x7A
  motor1 = cg.init_motor(0x7F)
  motor1.change_motor_can_id(0x7A)
  # IMPORTANT: After this, the 'motor1' object is no longer valid.
  # You would need to cg.init_motor(0x7A) to communicate further.
  ```

- `get_motor_status(self)`
  - Returns the last known status tuple for _this_ motor instance: `(position, velocity, effort, temperature)`.
  - **Note:** Relies on recent calls to `CyberGear.process_packet()` for updated values.
  ```python
  # Assumes motor1 = cg.init_motor(0x7F)
  # Assumes cg.process_packet() is called frequently elsewhere
  pos, vel, eff, temp = motor1.get_motor_status()
  print(f"Current Pos: {pos:.2f}")
  ```

## Examples

### Example 1: Running the Motor in Speed Mode

```python
from cybergear import CyberGear
import time

MOTOR_ID = 0x7F

cg = CyberGear(interface='slcan', channel='/dev/ttyACM0', debug=False)
motor1 = cg.init_motor(MOTOR_ID)

try:
    print(f"Setting motor {MOTOR_ID:#04x} to speed mode...")
    motor1.set_run_mode(CyberGear.MODE_SPEED) # Access constants via class
    motor1.enable_motor()
    time.sleep(0.1)

    print("Sending speed command (6.0 rad/s)...")
    motor1.send_speed_command(6.0)
    start_time = time.time()
    while time.time() - start_time < 2.0:
        if cg.process_packet():
             status = cg.get_motor_status(MOTOR_ID) # Get status via controller
             if status:
                 print(f" Status: Vel={status[2]:.2f} rad/s, Effort={status[3]:.2f} Nm")
        time.sleep(0.02)

    print("Stopping motor...")
    motor1.stop_motor()

finally:
    print("Closing CAN bus...")
    cg.close()
```

### Example 2: Running the Motor in Position Mode

```python
from cybergear import CyberGear
import time

MOTOR_ID = 0x7F

cg = CyberGear(interface='slcan', channel='/dev/ttyACM0', debug=False)
motor1 = cg.init_motor(MOTOR_ID)

try:
    print(f"Setting motor {MOTOR_ID:#04x} to position mode...")
    motor1.set_run_mode(CyberGear.MODE_POSITION)
    motor1.set_position_control_gain(30.0)
    motor1.set_speed_limit(2.0)
    motor1.enable_motor()
    time.sleep(0.1)

    target_position = 1.57
    print(f"Sending position command ({target_position} rad)...")
    motor1.send_position_command(target_position)

    print("Waiting for move (polling status)...")
    wait_start_time = time.time()
    while time.time() - wait_start_time < 5.0:
        motor1.send_position_command(target_position)
        if cg.process_packet():
            status = cg.get_motor_status(MOTOR_ID)
            if status:
                print(f" Status: Pos={status[1]:.2f} rad, Vel={status[2]:.2f} rad/s")
                if abs(status[1] - target_position) < 0.05 and abs(status[2]) < 0.1:
                    print("Target position likely reached.")
                    break
        time.sleep(0.05)

    print("Stopping motor...")
    motor1.stop_motor()

finally:
    print("Closing CAN bus...")
    cg.close()
```

### Example 3: Reading Motor Status (Polling)

```python
from cybergear import CyberGear
import time

MOTOR_ID = 0x7F

cg = CyberGear(interface='slcan', channel='/dev/ttyACM0', debug=False)
motor1 = cg.init_motor(MOTOR_ID)

try:
    print("Enabling motor...")
    motor1.enable_motor()
    time.sleep(0.1)

    print("Polling status for 3 seconds...")
    start_time = time.time()
    while time.time() - start_time < 3.0:
        motor1.enable_motor() # We need to send some command to receive status packet
        if cg.process_packet():
            status = cg.get_motor_status(MOTOR_ID)
            if status:
                print(f" Motor Status: Pos={status[1]:.2f}, Vel={status[2]:.2f}, Eff={status[3]:.2f}, Temp={status[4]:.1f}")
        time.sleep(0.1)

    print("Stopping motor...")
    motor1.stop_motor()

finally:
    print("Closing CAN bus...")
    cg.close()
```

### Example 4: Sending a Motion Command

```python
from cybergear import CyberGear
import time

MOTOR_ID = 0x7F

cg = CyberGear(interface='slcan', channel='/dev/ttyACM0', debug=False)
motor1 = cg.init_motor(MOTOR_ID)

try:
    print("Setting up for motion control...")
    motor1.set_run_mode(CyberGear.MODE_MOTION)
    print("Enabling motor...")
    motor1.enable_motor()
    time.sleep(0.1)

    print("Monitoring status during motion command...")
    start_time = time.time()
    while time.time() - start_time < 3.0:
        motor1.send_motion_command(position=1.57, speed=10.0, torque=0.01, kp=0.1, kd=0.1)
        if cg.process_packet():
            status = cg.get_motor_status(MOTOR_ID)
            if status:
                print(f" Status: Pos={status[1]:.2f}, Vel={status[2]:.2f}, Eff={status[3]:.2f}")
        time.sleep(0.05)

    print("Stopping motor...")
    motor1.stop_motor()

finally:
    print("Closing CAN bus...")
    cg.close()
```

### Example 5: Driving a Wheel Fixed Distance (Illustrates `process_packet` Loop)

```python
from cybergear import CyberGear
import time
import math

MOTOR_ID = 0x7F
WHEEL_DIAMETER_CM = 21.0
# Calibrate this value based on your setup!
# It's the change in motor.position value for one full output revolution.
# Default range is -12.5 to +12.5 (25.0 units).
# 25.0 units is 4 turns, so COUNTS_PER_OUTPUT_REV = 25.0 / 4.0 = 6.25
COUNTS_PER_OUTPUT_REV = 6.25

WHEEL_CIRCUMFERENCE_CM = math.pi * WHEEL_DIAMETER_CM
CM_PER_COUNT = WHEEL_CIRCUMFERENCE_CM / COUNTS_PER_OUTPUT_REV

cg = CyberGear(interface='slcan', channel='/dev/ttyACM0', debug=False)
motor1 = cg.init_motor(MOTOR_ID)

try:
    print("Setting up for distance control...")
    motor1.set_run_mode(CyberGear.MODE_SPEED)
    motor1.enable_motor()
    motor1.set_mech_position_to_zero()
    time.sleep(0.2)

    target_distance_cm = 50.0
    total_distance_cm = 0.0
    status_init = cg.get_motor_status(MOTOR_ID)
    if not status_init: raise Exception("Failed to get initial motor status")
    last_position_rad = status_init[1] # Position is index 1

    print(f"Moving forward {target_distance_cm} cm...")
    MAX_SPEED_RAD_S = 6.0

    while total_distance_cm < target_distance_cm:
        # Process incoming CAN messages to update motor status
        cg.process_packet()

        # Get latest status from the controller's cache
        status_now = cg.get_motor_status(MOTOR_ID)
        current_position_rad = status_now[1] # Position index

        # Calculate distance delta, handling wrap-around
        delta_rad = current_position_rad - last_position_rad
        pos_range = CyberGear.P_MAX - CyberGear.P_MIN
        if delta_rad > pos_range / 2: delta_rad -= pos_range
        elif delta_rad < -pos_range / 2: delta_rad += pos_range

        distance_moved_cm = delta_rad * CM_PER_COUNT
        total_distance_cm += distance_moved_cm
        last_position_rad = current_position_rad

        # Basic P-control for speed
        remaining_distance = target_distance_cm - total_distance_cm
        speed_command = max(0.1, min(MAX_SPEED_RAD_S, remaining_distance * 0.5))
        motor1.send_speed_command(speed_command)

        # Optional printout
        # print(f" Target: {target_distance_cm:.1f}cm | Current: {total_distance_cm:.1f}cm | Speed Cmd: {speed_command:.2f}rad/s | Pos: {current_position_rad:.2f}")

        time.sleep(0.01)

    print("Target distance reached.")
    motor1.stop_motor()

finally:
    print("Closing CAN bus...")
    cg.close()

```

## Connecting the Motor Depending on the CAN Driver

### 1. **Using `can0` on Linux/Raspberry Pi**

If you're using a Raspberry Pi or a Linux machine with a CAN interface (e.g., MCP2515 or similar), you can use the `socketcan` interface. Here's how to set it up:

#### Step 1: Enable the CAN Interface

First, ensure that the CAN interface is enabled and configured. You can do this by adding the following lines to `/etc/network/interfaces`:

```bash
auto can0
iface can0 can static
    bitrate 1000000
    up ifconfig $IFACE txqueuelen 1000
```

Then, bring up the interface:

```bash
sudo ifconfig can0 up
```

#### Step 2: Use the `socketcan` Interface in Python

When initializing the `CyberGear` class, specify the `interface` as `'socketcan'` and the `channel` as `'can0'`:

```python
cg = CyberGear(interface='socketcan', channel='can0', bitrate=1000000, debug=False)
motor = cg.init_motor(0x7F)
```

### 2. **Using a USB-to-CAN Adapter (e.g., CANable)**

If you're using a USB-to-CAN adapter like the CANable, you can use the `slcan` interface.

#### Step 1: Install `slcand`

Ensure that `slcand` is installed and running. You can start it with:

```bash
sudo slcand -o -s8 -t hw -S 1000000 /dev/ttyUSB0
sudo ifconfig slcan0 up
```

Replace `/dev/ttyUSB0` with the appropriate device name for your adapter.

#### Step 2: Use the `slcan` Interface in Python

When initializing the `CyberGear` class, specify the `interface` as `'slcan'` and the `channel` as the device name (e.g., `/dev/ttyUSB0`):

```python
cg = CyberGear(interface='slcan', channel='/dev/ttyUSB0', bitrate=1000000, debug=False)
motor = cg.init_motor(0x7F)
```

### 3. **Using a Serial CAN Adapter (e.g., Waveshare)**

If you're using a Serial CAN adapter like the Waveshare, you can also use the `slcan` interface.

#### Step 1: Configure the Serial CAN Adapter

Ensure that the adapter is properly connected and configured. You may need to set the baud rate and other parameters using the adapter's configuration tool.

#### Step 2: Use the `slcan` Interface in Python

When initializing the `CyberGear` class, specify the `interface` as `'slcan'` and the `channel` as the serial port (e.g., `/dev/ttyUSB_CAN`):

```python
cg = CyberGear(interface='slcan', channel='/dev/ttyUSB_CAN', bitrate=1000000, debug=False)
motor = cg.init_motor(0x7F)
```

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request if you have any improvements or bug fixes.

## Acknowledgments

This library was developed with the help of the following resources:

- **[M5 Stack CyberGear Library](https://github.com/project-sternbergia/cybergear_m5)**: The CAN protocol and motor control logic were heavily inspired by this library.
- **[python-can](https://python-can.readthedocs.io/)**: For providing a robust CAN bus interface for Python.

Special thanks to the open-source community for their contributions and support.

---

For more information, please refer to the [Xiaomi CyberGear website](https://www.mi.com/cybergear), [Unofficial CyberGear documentation](https://github.com/belovictor/cybergear-docs) or open an issue in this repository.
