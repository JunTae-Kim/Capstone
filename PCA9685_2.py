__author__ = 'ataylor'

from smbus import SMBus

from math import floor
from time import sleep
import sys

# The default address, used unless the address jumpers are bridged
DEFAULT_ADDRESS = 0x40
# The default value on modern Raspberry Pis
DEFAULT_I2C_BUS = 1

# Registers:
MODE1 = 0x00
MODE2 = 0x01
PRE_SCALE = 0xFE

# Base address which all LEDs are referenced from
# Each LED is 4 addresses:
#  On H
#  On L
#  Off H
#  Off L
LED_BASE = 0x06

# Internal oscillator frequency
OSC = 25000000
# Stock frequency of 60Hz
FREQ = 60

class pca9865_servo(object):

    def __init__(self, i2c_address=DEFAULT_ADDRESS, i2c_bus=DEFAULT_I2C_BUS):

        # Create a bus object for the given bus
        self.bus = SMBus(i2c_bus)

        # Use the supplied target address
        self.address = i2c_address

        # Turn all the outputs off
        self._set_all_off()

        # Send the unit to sleep so we can set the oscillator
        self._write_bytes(MODE1, 0x10)

        # Wait for the unit to stabilise after sending it to sleep
        sleep(0.1)

        # We will be working at 60Hz
        prescale_value = int(floor(OSC / (4096*FREQ)) - 1)
        # This has to be set whilst the unit is in sleep mode
        self._write_bytes(PRE_SCALE, prescale_value)

        # The oscillator takes a while to stabilise
        sleep(0.1)

        # Set default values in MODE1, bring it out of sleep
        self._write_bytes(MODE1, 0x00)
        # Set the outputs to open-dram
        self._write_bytes(MODE2, 0x04)

    # Set all outputs to off, handy for resetting
    def _set_all_off(self):
        self._write_bytes(0xFA, 0)
        self._write_bytes(0xFB, 0)
        self._write_bytes(0xFC, 0)
        self._write_bytes(0xFD, 0)

    def _write_bytes(self, register, value):
        self.bus.write_byte_data(self.address, int(register), int(value))

    def set_servo_value(self, channel, value):
        # We just set off the on times, the rest is spent being off
        self._write_bytes(LED_BASE+2+4*channel, value & 0xFF)
        self._write_bytes(LED_BASE+3+4*channel, value >> 8)
        pass


servo_controller = pca9865_servo()

# Loop indefinitely, waiting for input from the node-red javascript functions
# Input is: <channel> <value>
# where value is +- from the midpoint
while True:

    try:
        input = raw_input()
        # Closing the connection
        if 'close' in input:
            sys.exit(0)

        # Split the input into two parts ...
        (s_channel, s_position) = input.split(" ", 1)

        # ... parse both into integers
        channel = int(s_channel)
        position = int(s_position)

        print ("Channel: %s, Position %s" % (channel, position))

        # Configure min and max servo pulse lengths
        servo_min = 150  # Min pulse length out of 4096
        servo_max = 600  # Max pulse length out of 4096

        mid_point = servo_max - servo_min
        servo_position = mid_point + position

        servo_controller.set_servo_value(channel, servo_position)

    except ValueError:
        # Bad input string
        print ("Bad input")
    except (EOFError, SystemExit):
        # Do any cleanup
        sys.exit(0)

