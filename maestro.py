import serial
import xmltodict
from Device import DeviceSpecific
from Channel import Channel

from sys import version_info
PY2 = version_info[0] == 2   #Running Python 2.x?


class Maestro:
    DeviceSpecific = DeviceSpecific # Import decorator

    # When connected via USB, the Maestro creates two virtual serial ports /dev/ttyACM0 for commands and /dev/ttyACM1 for communications.
    # Be sure the Maestro is configured for "USB Dual Port" serial mode. "USB Chained Mode" may work as well, but hasn't been tested.
    def __init__(self, settings_file, ttyStr='/dev/ttyACM0'):

        # Load settings file
        with open(settings_file, 'r') as f:
            settings = xmltodict.parse(f.read())["UscSettings"]
            f.close()

        # Handle different serial modes????

        # Serial Mode
        self.usb = serial.Serial(
            ttyStr,
            baudrate = int(settings["FixedBaudRate"])
            # ,timeout = int(settings["SerialTimeout"]) if int(settings["SerialTimeout"]) > 0 else None
        )

        # Save CRC preference
        self.crc_enabled = (settings["EnableCrc"] == "true")

        # Save device number
        # Command lead-in and device number are sent for each Pololu serial command.
        self.device = int(settings["SerialDeviceNumber"])
        self.PololuCmd = chr(0xaa) + chr(self.device)

        # Save period and period multiplier (mini maestro only)
        if "@MiniMaestroServoPeriod" in settings["Channels"]:
            self.period = int(settings["Channels"]["@MiniMaestroServoPeriod"]) # Units: 0.25us
            self.multiplier = int(settings["Channels"]["@ServoMultiplier"])
        else:
            self.period = 80000 # !!! is period set for micro maestro??

        # Save individual channel settings
        self.channels = []
        for chan in settings["Channels"]["Channel"]:
            self.channels.append(Channel(chan))


    # Cleanup by closing USB serial port
    def close(self):
        self.usb.close()


    # Separate a value into high and low bytes
    def split(self, val):
        lsb = val & 0x7f # 7 bits for least significant byte
        msb = (val >> 7) & 0x7f # shift 7 and take next 7 bits for msb
        return chr(lsb) + chr(msb)


    # Calculate CRC-7 polynomial for a string of hex bytes
    # Send the crc byte after the cmd byte. If this CRC byte is incorrect,
    # the Maestro will set the 'Serial CRC' error bit in the error register and ignore the command.
    def crc7(cmd):
        cmd = [ord(c) for c in cmd] + [0] # Append zeros
        remainder = cmd[0] # Initialize remainder

        # Iterate through bytes, then bits of each byte
        for octet in cmd[1:]:
            for pos in range(8):
                remainder ^= (0x91 if (remainder % 2) else 0x00) # Perform xor. CRC Polynomial = 0x91
                remainder >>= 1 # Right shift
                remainder += 0x80 if (octet & (1 << pos)) else 0x00 # Append new msb

        return chr(remainder & 0x7fff) # always clear msb (may be redundant)


    # Send a Pololu command out the serial port
    def sendCmd(self, cmd):
        cmdStr = self.PololuCmd + cmd
        if self.crc_enabled:
            cmdStr += self.crc7(cmdStr) # Perform crc calculation

        self.usb.write(cmdStr if PY2 else bytes(cmdStr,'latin-1'))


    # Set channel to a specified target value
    # Target values will be constrained within Min and Max range
    # For servos, target represents the pulse width in of quarter-microseconds
    @DeviceSpecific
    def setTarget(self, chan, target):
        # Record Target value
        self.channels[chan].update_target(target)

        cmd = chr(0x04) + chr(chan) + self.split(target)
        self.sendCmd(cmd)


    # Set a contiguous block of targets (e.g. channel 4, 5, and 6)
    # Not available with Micro Maestro.
    @DeviceSpecific
    def setMultipleTargets(self, first_chan, *args): # args: target 1, target 2, ...
        # Send setup bytes
        cmd = chr(0x1f) + chr(len(args)) + chr(first_chan)

        # Send targets
        for n, target in enumerate(args):
            # Record Target value
            self.channels[first_chan + n].update_target(target)

            # Send target
            cmd += self.split(target)

        self.sendCmd(cmd)


    # Used for digital IO
    HIGH = 0x3fff
    LOW  = 0x0000

    # If the channel is configured as a digital output,
    # values less than 6000 tell the Maestro to drive the line low,
    # while values of 6000 or greater tell the Maestro to drive the line high.
    @DeviceSpecific
    def digitalWrite(self, chan, val):
        self.setTarget(chan, val)


    # Set speed of channel
    # Speed of 0 is unrestricted.
    # Period (T)  | Speed units
    # ----------- | -----------------
    # T = 20 ms   | (0.25 μs)/(10 ms)
    # T = 3–19 ms | (0.25 μs)/T
    # T > 20 ms   | (0.25 μs)/(T/2)
    @DeviceSpecific
    def setSpeed(self, chan, speed):
        cmd = chr(0x07) + chr(chan) + self.split(speed)
        self.sendCmd(cmd)


    # Set acceleration of channel
    # This provide soft starts and finishes when servo moves to target position.
    # Valid values are from 0 to 255. 0 = unrestricted, 1 = slowest start.
    # Period (T)  | Acceleration units
    # ----------- | -------------------------
    # T = 20 ms   | (0.25 μs)/(10 ms)/(80 ms)
    # T = 3–19 ms | (0.25 μs)/T/(8T)
    # T > 20 ms   | (0.25 μs)/(T/2)/(4T)
    @DeviceSpecific
    def setAccel(self, chan, accel):
        cmd = chr(0x09) + chr(chan) + self.split(accel)
        self.sendCmd(cmd)


    # Configure the maestro's PWM channel
    # Freq units=Hz, duty=[0,1]
    # Not available with Micro Maestro.
    @DeviceSpecific
    def setPWM(self, freq, duty):
        period = round((1/freq)/(1/48e-6)) # Units: 1/48 μs
        on_time = round(period*duty)

        cmd = chr(0x0a) + self.split(on_time) + self.split(period)
        self.sendCmd(cmd)


    # Create a generic pwm signal on a servo channel
    # Min should be set to lowest possible value. Max should be the same as period
    # Behavior may be unreliable at duty cycles close to 0 or 1
    @DeviceSpecific
    def pseudoPWM(self, chan, duty):
        period = self.period * (self.multiplier if self.channels[chan].multiplied else 1)
        self.setTarget(chan, round(duty*period))


    # Get the current position of the device on the specified channel
    # The result is returned in a measure of quarter-microseconds, which mirrors
    # the Target parameter of setTarget.
    # This is not reading the true servo position, but the last target position sent
    # to the servo. If the Speed is set to below the top speed of the servo, then
    # the position result will align well with the actual servo position, assuming
    # it is not stalled or slowed.
    @DeviceSpecific
    def getPosition(self, chan):
        cmd = chr(0x10) + chr(chan)
        self.sendCmd(cmd)

        lsb = ord(self.usb.read())
        msb = ord(self.usb.read())
        return (msb << 8) + lsb


    # If the channel is configured as an input, the position represents the voltage measured on the channel.
    # The inputs on channels 12–23 are digital: their values are either exactly 0 or exactly 1023.
    @DeviceSpecific
    def digitalRead(self, chan):
        return self.HIGH if self.getPosition(chan) == 1023 else self.LOW


    # If the channel is configured as an input, the position represents the voltage measured on the channel.
    # The inputs on channels 0–11 are analog: their values range from 0 to 1023, representing voltages from 0 to 5 V
    @DeviceSpecific
    def analogRead(self, chan):
        print("hello")
        return self.getPosition(chan)


    # Test to see if a servo has reached the set target position.  This only provides
    # useful results if the Speed parameter is set slower than the maximum speed of
    # the servo.  Servo range must be defined first using setRange. See setRange comment.

    # ***Note if target position goes outside of Maestro's allowable range for the
    # channel, then the target can never be reached, so it will appear to always be
    # moving to the target.
    @DeviceSpecific
    def isMoving(self, chan):
        if self.channels[chan].target > 0:
            return self.getPosition(chan) != self.channels[chan].target

        return False


    # Have all servo outputs reached their targets? This is useful only if Speed and/or
    # Acceleration have been set on one or more of the channels. Returns True if moving
    # Not available with Micro Maestro.
    @DeviceSpecific
    def getMovingState(self):
        cmd = chr(0x13)
        self.sendCmd(cmd)
        return self.usb.read() != chr(0)


    # Get errors
    # Response: error bits 0-7, error bits 8-15
    # Bit 0: Serial signal error
    # Bit 1: Serial overrun error
    # Bit 2: Serial buffer full
    # Bit 3: Serial CRC error
    # Bit 4: Serial protocol error
    # Bit 5: Serial timeout
    # Bit 6: Script stack error
    # Bit 7: Script call stack error
    # Bit 8: Script program counter error
    def getErrors(self):
        cmd = chr(0x21)
        self.sendCmd(cmd)

        lsb = ord(self.usb.read())
        msb = ord(self.usb.read())
        return (msb << 8) + lsb


    # Go home
    def goHome(self):
        # Update targets
        for chan in self.channels:
            if chan.home_mode == "Go to": #!!!!!!!!!!! not tested
                chan.target = chan.home

        cmd = chr(0x22)
        self.sendCmd(cmd)


    # Run a Maestro Script subroutine in the currently active script. Scripts can
    # have multiple subroutines, which get numbered sequentially from 0 on up. Code your
    # Maestro subroutine to either infinitely loop, or just end (return is not valid).
    def runScriptSub(self, subNumber, param=None):
        # Pass a parameter to script
        if param != None: cmd = chr(0x28) + chr(subNumber) + self.split(param)

        # Run script with no parameter
        else: cmd = chr(0x27) + chr(subNumber)

        self.sendCmd(cmd)


    # Stop the current Maestro Script
    def stopScript(self):
        cmd = chr(0x24)
        self.sendCmd(cmd)


    # Get script status
    # Response: 0x00 if the script is running, 0x01 if the script is stopped
    def isScriptRunning(self):
        cmd = chr(0x2E)
        self.sendCmd(cmd)
        return True if ord(self.usb.read()) == 0x00 else False
