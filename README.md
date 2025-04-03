# Xiaomi CyberGear Motor Controller

This repository contains a Python library for controlling Xiaomi CyberGear motors over a CAN bus. The library provides a simple interface to send commands to the motor, such as setting the position, speed, or torque, and reading the motor's status (position, velocity, effort, and temperature).

**Disclaimer**: This is an **unofficial library** and is not affiliated with or endorsed by Xiaomi. It was developed with the help of the [M5 Stack CyberGear library](https://github.com/project-sternbergia/cybergear_m5), which served as a reference for the CAN protocol and motor control logic.

## Installation

To use this library, you need to have Python 3 installed along with the `python-can` package. You can install the required package using pip:

```bash
pip install python-can
```

You can install this library using `pip` or by cloning the repository directly from GitHub.

### Option 1: Install via `pip`

If the library is published on PyPI, you can install it using:

```bash
pip install pycybergear
```

### Option 2: Install via `git`

To install the latest version directly from the GitHub repository, use:

```bash
pip install git+https://github.com/blanck/cybergear.git
```

## Library Structure

The library consists of two main classes:

1. **CyberGear**: This class initializes the CAN bus and provides a method to initialize a motor instance.
2. **Motor**: This class represents a single motor and provides methods to control the motor, such as setting the run mode, sending position/speed/torque commands, and reading the motor's status.

### Key Methods

#### CyberGear Class

- `__init__(self, interface: str = 'slcan', channel: str = '/dev/ttyUSB_CAN', bitrate: int = 1000000, debug: bool = False)`: Initializes the CAN bus.
- `init_motor(self, motor_id: int, master_can_id: int)`: Initializes a motor instance with the specified CAN ID.
- `close(self)`: Shuts down the CAN bus.

#### Motor Class

- `set_run_mode(self, mode: int)`: Sets the run mode of the motor (e.g., position, speed, current).
- `set_speed_limit(self, limit: float)`: Sets the speed limit for the motor.
- `set_torque_limit(self, limit: float)`: Sets the torque limit for the motor.
- `set_current_limit(self, limit: float)`: Sets the current limit for the motor.
- `set_position_control_gain(self, kp: float)`: Sets the position control gain (Kp) for the motor.
- `set_velocity_control_gain(self, kp: float, ki: float)`: Sets the velocity control gains (Kp and Ki) for the motor.
- `set_current_control_param(self, kp: float, ki: float, gain: float)`: Sets the current control parameters (Kp, Ki, and filter gain) for the motor.
- `enable_motor(self)`: Enables the motor.
- `reset_motor(self)`: Resets the motor.
- `stop_motor(self, fault: bool = False)`: Stops the motor.
- `send_position_command(self, position: float)`: Sends a position command to the motor.
- `send_speed_command(self, speed: float)`: Sends a speed command to the motor.
- `send_current_command(self, current: float)`: Sends a current command to the motor.
- `send_motion_command(self, position: float, speed: float, torque: float, kp: float, kd: float)`: Sends a motion command to the motor (position, speed, torque, Kp, and Kd).
- `set_mech_position_to_zero(self)`: Sets the mechanical position to zero.
- `get_motor_status(self, motor_ids: Optional[list[int]] = None, timeout: float = 0.5)`: Retrieves the status of the motor(s) (position, velocity, effort, and temperature).
- `change_motor_can_id(self, new_can_id: int)`: Change CAN ID of the motor

## Examples

### Example 1: Running the Motor in Speed Mode

```python
from cybergear import CyberGear
import time

# Initialize the CAN bus and motor
cg = CyberGear(interface='slcan', channel='/dev/ttyUSB_CAN', debug=False)
motor = cg.init_motor(0x7F)

try:
    # Set the motor to speed mode
    motor.set_run_mode(CyberGear.MODE_SPEED)
    motor.set_speed_limit(4.0)  # Set speed limit to 4.0 rad/s
    motor.set_current_control_param(0.075, 0.075, 0.0)  # Set current control parameters
    motor.enable_motor()  # Enable the motor

    # Send a speed command (6.0 rad/s)
    motor.send_speed_command(6.0)
    time.sleep(1)  # Run for 1 second

    # Stop the motor
    motor.stop_motor()

finally:
    # Close the CAN bus
    cg.close()
```

### Example 2: Running the Motor in Position Mode

```python
from cybergear import CyberGear
import time

# Initialize the CAN bus and motor
cg = CyberGear(interface='slcan', channel='/dev/ttyUSB_CAN', debug=False)
motor = cg.init_motor(0x7F)

try:
    # Set the motor to position mode
    motor.set_run_mode(CyberGear.MODE_POSITION)
    motor.set_position_control_gain(30.0)  # Set position control gain (Kp)
    motor.set_speed_limit(2.0)  # Set speed limit for position mode
    motor.set_torque_limit(5.0)  # Set torque limit for position mode
    motor.enable_motor()  # Enable the motor

    # Send a position command (1.57 radians = 90 degrees)
    motor.send_position_command(1.57)
    time.sleep(2)  # Wait for the motor to reach the target position

    # Stop the motor
    motor.stop_motor()

finally:
    # Close the CAN bus
    cg.close()
```

### Example 3: Reading Motor Status

```python
from cybergear import CyberGear
import time

# Initialize the CAN bus and motor
cg = CyberGear(interface='slcan', channel='/dev/ttyUSB_CAN', debug=False)
motor = cg.init_motor(0x7F)

try:
    # Enable the motor
    motor.enable_motor()

    # Read the motor status
    status = motor.get_motor_status()
    if status:
        motor_id, position, velocity, effort, temperature = status
        print(f"Motor ID: {motor_id}")
        print(f"Position: {position} rad")
        print(f"Velocity: {velocity} rad/s")
        print(f"Effort: {effort} Nm")
        print(f"Temperature: {temperature} °C")

finally:
    # Close the CAN bus
    cg.close()
```

### Example 4: Sending a Motion Command

```python
from cybergear import CyberGear
import time

# Initialize the CAN bus and motor
cg = CyberGear(interface='slcan', channel='/dev/ttyUSB_CAN', debug=False)
motor = cg.init_motor(0x7F)

try:
    # Enable the motor
    motor.enable_motor()

    # Send a motion command (position, speed, torque, Kp, Kd)
    motor.send_motion_command(position=1.57, speed=2.0, torque=1.0, kp=100.0, kd=1.0)
    time.sleep(2)  # Wait for the motor to execute the command

    # Stop the motor
    motor.stop_motor()

finally:
    # Close the CAN bus
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

For more information, please refer to the [Xiaomi CyberGear website](https://www.mi.com/cybergear), [Xiaomi CyberGear documentation](https://github.com/belovictor/cybergear-docs) or open an issue in this repository.
