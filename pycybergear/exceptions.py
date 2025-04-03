# src/pycybergear/exceptions.py

class CyberGearError(Exception):
    """Base exception for pycybergear library errors."""
    pass

class MotorError(CyberGearError):
    """Exception related to a specific motor operation or state."""
    pass

class CANError(CyberGearError):
    """Exception related to CAN bus communication failures."""
    pass

class ConfigurationError(CyberGearError):
    """Exception for configuration-related issues."""
    pass

class TimeoutError(CyberGearError):
    """Exception for operations timing out (e.g., parameter reads)."""
    pass