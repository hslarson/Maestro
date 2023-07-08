import inspect
import logging
logger = logging.getLogger(__name__)

# Decorator for verifying device-specific capabilities
def DeviceSpecific(method):
    def wrapper(*args, **kwargs):
        arg_names = inspect.getfullargspec(method).args
        self = args[arg_names.index('self')]

         # Check for channel out of bounds
        if ('chan' in arg_names):
            if args[arg_names.index('chan')] >= len(self.channels):
                logger.error("Channel out of bounds. "+str(args[arg_names.index('chan')])+" > "+str(len(self.channels)-1))
                return

        # Check for pin type errors
        if method.__name__ in {"digitalWrite", "digitalRead", "analogRead", "setSpeed", "setAccel"}:
            chan_num = args[arg_names.index('chan')]
            channel = self.channels[chan_num]

            if (method.__name__ == "digitalRead"):
                if channel.mode != "Input" and chan_num < 12:
                    logger.warning("Channels 0-11 only support analog input")
                    logger.warning(f"Channel {chan_num} is configured as {channel.mode}")

            elif (method.__name__ == "analogRead"):
                if channel.mode != "Output" and chan_num > 11:
                    logger.warning("Channels 12-23 only support digital input")
                    logger.warning(f"Channel {chan_num} is configured as {channel.mode}")

            elif (method.__name__ == "digitalWrite"):
                if channel.mode != "Output":
                    logger.warning("digitalWrite is intended for use with Output channels")
                    logger.warning(f"Channel {chan_num} is configured as {channel.mode}")

            elif channel.mode != "Servo":
                logger.warning(method.__name__ + " has no effect for non-servo channels")
                logger.warning(f"Channel {chan_num} is configured as {channel.mode}")

        # Mini-maestro-only functions
        if len(self.channels) == 6:
            if method.__name__ == "setPWM":
                logger.warning("Micro Maestro has no PWM channel, try using pseudoPWM")
                return

            elif method.__name__ == "setMultipleTargets":
                logger.warning("Micro maestro doesn't support setMultipleTargets")
                return

            elif method.__name__ == "getMovingState":
                logger.warning("Micro maestro doesn't support getMovingState")
                return

        # Call function
        method(*args, **kwargs)

    return wrapper
