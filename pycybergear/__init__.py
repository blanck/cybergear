# src/pycybergear/__init__.py

import logging

# Configure root logger for the library package
# Users can further configure logging if needed
log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler()) # Avoids "No handler found" warnings if user doesn't configure logging

# Import main classes and enums for easier access
# e.g., from pycybergear import CyberGearController, Motor, RunMode
from .controller import CyberGearController
from .motor import Motor
from .constants import RunMode, ParamAddr, MotorErrorFlags, CommandType
from .exceptions import CyberGearError, MotorError, CANError, ConfigurationError, TimeoutError

# Define package version (consider using importlib.metadata in Python 3.8+)
__version__ = "1.0.0" 

# Define what 'from pycybergear import *' imports (optional but good practice)
__all__ = [
    "CyberGearController",
    "Motor",
    "RunMode",
    "ParamAddr",
    "MotorErrorFlags",
    "CommandType",
    "CyberGearError",
    "MotorError",
    "CANError",
    "ConfigurationError",
    "TimeoutError",
    "__version__"
]