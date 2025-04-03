# src/pycybergear/constants.py

from enum import Enum, Flag

class CommandType(Enum):
    """CyberGear CAN Command/Communication Types (Bits 28-24 of Arbitration ID)"""
    GET_MCU_ID = 0
    MOTION_COMMAND = 1
    MOTOR_FEEDBACK = 2
    ENABLE_MOTOR = 3
    DISABLE_MOTOR = 4 # Also used for clearing errors
    SET_MECH_POS_ZERO = 6
    SET_CAN_ID = 7
    FLASH_COMMAND = 8 # Used for Save/Restore Params
    PARAM_READ = 17 # Read RAM Parameter (>=0x7000)
    PARAM_WRITE = 18 # Write RAM Parameter (>=0x7000)
    GET_MOTOR_ERROR = 21 # Dedicated error frame

class ParamAddr(Enum):
    """CyberGear Parameter Addresses (Index for Read/Write commands)"""
    # From original constants list / common usage
    RUN_MODE = 0x7005
    IQ_REF = 0x7006
    SPEED_REF = 0x700A
    LIMIT_TORQUE = 0x700B
    CURRENT_KP = 0x7010
    CURRENT_KI = 0x7011
    CURRENT_FILTER_GAIN = 0x7014
    LOC_REF = 0x7016
    LIMIT_SPEED = 0x7017
    LIMIT_CURRENT = 0x7018
    LOC_KP = 0x701E
    SPD_KP = 0x701F
    SPD_KI = 0x7020
    # Read Only addresses from docs (0x7000 range)
    MECH_POS = 0x7019
    IQF = 0x701A
    MECH_VEL = 0x701B
    VBUS = 0x701C
    ROTATION = 0x701D
    # Writable addresses from docs (0x2000 range)
    MECH_OFFSET = 0x2005
    MECH_POS_INIT = 0x2006
    CAN_ID = 0x200a
    CAN_MASTER_ID = 0x200b
    CAN_TIMEOUT = 0x200c
    MOTOR_OVERTEMP = 0x200d
    OVERTEMP_TIME = 0x200e
    GEAR_RATIO = 0x200f
    TQ_CALI_TYPE = 0x2010
    CUR_FILT_GAIN = 0x2011 # Duplicates 0x7014?
    CUR_KP = 0x2012 # Duplicates 0x7010?
    CUR_KI = 0x2013 # Duplicates 0x7011?
    SPD_FILT_GAIN = 0x2017
    LIMIT_CUR = 0x2019 # Duplicates 0x7018?
    # Add others from pages 13-16 if needed

class RunMode(Enum):
    POSITION = 0x01
    SPEED = 0x02
    CURRENT = 0x03
    # Mode 0 is mentioned in docs as "Motion Control Mode", used for MOTION_COMMAND?

class MotorErrorFlags(Flag):
    """Flags representing motor error status bits (derived from documentation)."""
    NONE = 0
    # Bits 16-21 from Type 2 Feedback ID (shifted)
    MOTOR_VOLTAGE_LOW = 1 << 16
    MOTOR_OVERCURRENT = 1 << 17
    MOTOR_OVERTEMP = 1 << 18
    ENCODER_MAG_FAULT = 1 << 19
    ENCODER_HALL_FAULT = 1 << 20
    MOTOR_NOT_CALIBRATED = 1 << 21
    # Bits 0-31 from Type 21 Fault Frame Data
    FAULT_MOTOR_OVERTEMP = 1 << 0
    FAULT_DRV_CHIP = 1 << 1
    FAULT_VOLTAGE_LOW = 1 << 2 # Duplicates MOTOR_VOLTAGE_LOW?
    FAULT_VOLTAGE_HIGH = 1 << 3
    FAULT_CUR_SAMPLE_B = 1 << 4
    FAULT_CUR_SAMPLE_C = 1 << 5
    FAULT_ENCODER_NCAL = 1 << 7 # Duplicates MOTOR_NOT_CALIBRATED?
    FAULT_OVERLOAD = 1 << 8 # Assuming occupies bits 8-15
    FAULT_CUR_SAMPLE_A = 1 << 16 # Duplicates MOTOR_OVERCURRENT?
    # Bits 32+ from Type 21 Warning Frame Data (Byte 4-7)
    WARN_MOTOR_OVERTEMP = 1 << 32
    # Add other warning bits if needed

# Default values from original constants, useful for reference
DEFAULT_CURRENT_KP = 0.065
DEFAULT_CURRENT_KI = 0.065
DEFAULT_CURRENT_FILTER_GAIN = 0.0
DEFAULT_POSITION_KP = 30.0
DEFAULT_VELOCITY_KP = 2.0
DEFAULT_VELOCITY_KI = 0.002
DEFAULT_VELOCITY_LIMIT = 2.0 # rad/s
DEFAULT_CURRENT_LIMIT = 1.0 # Amps? Might be Iq_ref limit. Docs say 23A.
DEFAULT_TORQUE_LIMIT = 12.0 # Nm