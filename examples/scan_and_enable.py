# examples/scan_and_enable.py

import time
import logging
from pycybergear import CyberGearController, Motor, RunMode, CyberGearError

# --- Configuration ---
CAN_INTERFACE = 'slcan'
CAN_CHANNEL = '/dev/ttyACM0'
BITRATE = 1000000
MASTER_ID = 0

# Configure logging
logging.basicConfig(level=logging.INFO)
log = logging.getLogger(__name__)
# --- End Configuration ---

def run_scan_example():
    log.info("Starting CyberGear Scan Example")
    try:
        with CyberGearController(interface=CAN_INTERFACE, channel=CAN_CHANNEL,
                                 bitrate=BITRATE, master_can_id=MASTER_ID, debug=False) as controller:

            log.info("Scanning for motors (IDs 1-127)...")
            # Use a slightly longer timeout per ping for scan robustness if needed
            detected_ids = controller.scan_for_motors(scan_range=range(1, 128), ping_timeout=0.05)

            if not detected_ids:
                log.warning("No motors found.")
                return

            log.info(f"Found {len(detected_ids)} motors.")

            # Initialize and enable each detected motor
            motors = {}
            for motor_id in detected_ids:
                 try:
                     motor = controller.add_motor(motor_id)
                     motors[motor_id] = motor
                     log.info(f"Initializing and enabling motor {motor_id:#04x}...")
                     if not motor.enable_motor():
                         log.error(f"Failed to enable motor {motor_id:#04x}")
                     else:
                         log.info(f"Motor {motor_id:#04x} enabled.")
                 except CyberGearError as e:
                      log.error(f"Error initializing/enabling motor {motor_id:#04x}: {e}")

            time.sleep(1.0) # Keep motors enabled for a second

            # Print status of enabled motors
            log.info("Current status of detected/enabled motors:")
            for motor_id, motor in motors.items():
                 if motor.is_enabled: # Check if enable was successful
                     log.info(f" Motor {motor_id:#04x}: Pos={motor.position:.2f} Vel={motor.velocity:.2f} Err={motor.is_error}")
                 else:
                     log.info(f" Motor {motor_id:#04x}: Was not enabled successfully.")


            # Disable all motors
            log.info("Disabling all detected motors...")
            for motor_id, motor in motors.items():
                log.info(f" Disabling {motor_id:#04x}...")
                motor.reset_motor()

            time.sleep(0.5)
            log.info("Motors disabled.")

    except CyberGearError as e:
        log.error(f"CyberGear operation failed: {e}")
    except KeyboardInterrupt:
        log.info("Operation interrupted.")
    except Exception as e:
        log.error(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        log.info("Scan example finished.")

if __name__ == "__main__":
    run_scan_example()