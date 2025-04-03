# examples/parameter_handling.py

import time
import logging
from pycybergear import CyberGearController, Motor, RunMode, CyberGearError, ParamAddr

# --- Configuration ---
CAN_INTERFACE = 'slcan'
CAN_CHANNEL = '/dev/ttyACM0'
BITRATE = 1000000
MOTOR_ID = 0x7C # Target motor CAN ID
MASTER_ID = 0

logging.basicConfig(level=logging.INFO)
log = logging.getLogger(__name__)
# Set pycybergear level to DEBUG to see TX/RX for param read/write
# logging.getLogger('pycybergear').setLevel(logging.DEBUG)
# --- End Configuration ---

def run_param_example():
    log.info("Starting CyberGear Parameter Handling Example")
    try:
        with CyberGearController(interface=CAN_INTERFACE, channel=CAN_CHANNEL,
                                 bitrate=BITRATE, master_can_id=MASTER_ID, debug=False) as controller:

            log.info("Checking for motor...")
            if not controller.ping_motor(MOTOR_ID):
                 log.error(f"Motor {MOTOR_ID:#04x} not found.")
                 return

            motor = controller.add_motor(MOTOR_ID)

            # --- Read Parameter Example ---
            param_to_read = ParamAddr.LIMIT_TORQUE
            log.info(f"Requesting parameter {param_to_read.name} (0x{param_to_read.value:X})...")
            # Specify expected data type code ('f' for float32, 'H' for uint16, 'B' for uint8 etc.)
            if not motor.read_parameter(param_to_read, 'f'):
                 log.error("Failed to send read command.")
            else:
                # Wait for the background listener to receive and process the response
                read_value = None
                wait_start = time.monotonic()
                while time.monotonic() - wait_start < 0.5: # Wait up to 0.5 seconds
                    read_value = motor.get_read_parameter(param_to_read)
                    if read_value is not None:
                        break
                    time.sleep(0.02) # Poll periodically

                if read_value is not None:
                    log.info(f"Read {param_to_read.name} = {read_value}")
                else:
                    log.warning(f"Did not receive response for {param_to_read.name} within timeout.")


            # --- Write Parameter Example ---
            param_to_write = ParamAddr.LIMIT_SPEED # Speed limit in rad/s
            new_limit = 3.5
            log.info(f"Writing {param_to_write.name} = {new_limit} rad/s...")
            if not motor.set_speed_limit(new_limit): # Use specific setter if available
                 log.error("Failed to write speed limit.")
            else:
                 log.info("Speed limit write command sent.")
                 # Optionally re-read to verify (add delay first)
                 # time.sleep(0.1)
                 # motor.read_parameter(param_to_write, 'f')
                 # ... wait and check get_read_parameter ...


            # --- Save Parameters Example ---
            # Be careful: This writes current RAM parameters to persistent flash memory!
            # log.info("Saving parameters to Flash...")
            # if not motor.save_parameters():
            #     log.error("Failed to send save parameters command.")
            # else:
            #     log.info("Save parameters command sent.")
            # time.sleep(0.5) # Allow time for save operation


    except CyberGearError as e:
        log.error(f"Operation failed: {e}")
    except KeyboardInterrupt:
        log.info("Operation interrupted.")
    except Exception as e:
        log.error(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        log.info("Parameter example finished.")


if __name__ == "__main__":
    run_param_example()