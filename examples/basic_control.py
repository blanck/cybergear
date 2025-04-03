# examples/basic_control.py

import time
import logging
from pycybergear import CyberGearController, Motor, RunMode, CyberGearError

# --- Configuration ---
# Adjust these for your setup
CAN_INTERFACE = 'slcan'
CAN_CHANNEL = '/dev/tty.usbmodem206933BB55311' # Linux/Mac for USB adapter
# CAN_CHANNEL = 'COM3'      # Windows example
BITRATE = 1000000
MOTOR_ID = 0x7C             # Target motor CAN ID (change if needed)
MASTER_ID = 0               # Controller's CAN ID

# Configure logging (optional, set level for more details)
logging.basicConfig(level=logging.INFO)
log = logging.getLogger(__name__)
# To see detailed CAN traffic, set pycybergear's logger level:
# logging.getLogger('pycybergear').setLevel(logging.DEBUG)
# --- End Configuration ---

def run_example():
    log.info("Starting CyberGear Basic Control Example")
    controller = None # Define outside try block for cleanup
    try:
        # Initialize controller using 'with' for automatic start/close
        with CyberGearController(interface=CAN_INTERFACE, channel=CAN_CHANNEL,
                                 bitrate=BITRATE, master_can_id=MASTER_ID, debug=False) as controller:

            log.info("Pinging target motor...")
            if not controller.ping_motor(MOTOR_ID, timeout=0.1):
                 log.error(f"Motor {MOTOR_ID:#04x} did not respond to ping. Exiting.")
                 return

            log.info("Adding motor object...")
            motor = controller.add_motor(MOTOR_ID)

            log.info("Enabling motor...")
            if not motor.enable_motor():
                log.error("Failed to enable motor.")
                return # Exit if enable fails
            time.sleep(0.5) # Wait for potential status update after enable

            log.info(f"Motor State After Enable: Mode={motor.current_mode} Err={motor.is_error}")
            if motor.is_error:
                 log.warning(f"Motor has errors: {motor.get_error_description()}")
                 log.info("Attempting to clear errors...")
                 motor.clear_error()
                 time.sleep(0.1)
                 # Re-enable after clearing? Only if needed.
                 # motor.enable_motor()
                 # time.sleep(0.5)

            # --- Speed Mode Example ---
            log.info("Setting Speed Mode")
            if not motor.set_run_mode(RunMode.SPEED):
                log.error("Failed to set speed mode.")
            else:
                time.sleep(0.1) # Allow mode change
                log.info("Running at 4.0 rad/s")
                motor.send_speed_command(4.0)
                time.sleep(2.0) # Run for 2 seconds
                log.info("Running at -2.0 rad/s")
                motor.send_speed_command(-2.0)
                time.sleep(1.5)
                log.info("Stopping speed command (set to 0)")
                motor.send_speed_command(0.0)
                time.sleep(0.5)

            # --- Position Mode Example ---
            log.info("Setting Position Mode")
            if not motor.set_run_mode(RunMode.POSITION):
                log.error("Failed to set position mode")
            else:
                # Set limits/gains appropriate for position mode
                motor.set_speed_limit(3.0) # rad/s max speed for position moves
                motor.set_torque_limit(5.0) # Nm limit
                # Use default Kp or set your own:
                # motor.set_position_control_gain(30.0) # Set Kp if needed
                time.sleep(0.1)

                target_pos_1 = 1.57 # ~90 degrees
                log.info(f"Moving to position {target_pos_1:.2f} rad")
                motor.send_position_command(target_pos_1)
                time.sleep(2.0) # Wait for move to complete
                log.info(f" Current position: {motor.position:.2f}")

                target_pos_2 = 0.0
                log.info(f"Moving to position {target_pos_2:.2f} rad")
                motor.send_position_command(target_pos_2)
                time.sleep(2.0)
                log.info(f" Current position: {motor.position:.2f}")

            # --- Stopping Motor ---
            log.info("Stopping motor...")
            motor.reset_motor() # Or motor.reset_motor()
            time.sleep(0.1)
            log.info("Motor stopped.")

    except CyberGearError as e:
        log.error(f"CyberGear operation failed: {e}")
    except KeyboardInterrupt:
        log.info("Operation interrupted by user.")
    except Exception as e:
        # Log unexpected errors with traceback
        log.error(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        # The 'with' statement handles controller.close() automatically
        log.info("Example finished.")

if __name__ == "__main__":
    run_example()