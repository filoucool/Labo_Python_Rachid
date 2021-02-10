""" packet sample
 +----+----+--+------+-----------+----------+---+-----------+---------+
    |0XFF|0XFF|ID|LENGTH|INSTRUCTION|PARAMETER1|...|PARAMETER N|CHECK SUM|
    +----+----+--+------+-----------+----------+---+-----------+---------+
"""

import serial
import sys
import time
import math
import argparse

try:
    import RPi.GPIO as gpio
except:
    pass

PING = 0x01
READ_DATA = 0x02
WRITE_DATA = 0x03
REG_WRITE = 0x04
ACTION = 0x05
RESET = 0x06
SYNC_WRITE = 0x83

INSTRUCTIONS = (PING, READ_DATA, WRITE_DATA, REG_WRITE, ACTION, RESET,
                SYNC_WRITE)


MAX_NUM_PARAMS = 255 - 6 

NUMBER_OF_PARAMETERS = {
    PING:{
        'min': 0,
        'max': 0
    },
    READ_DATA:{
        'min': 2,
        'max': 2
    },
    WRITE_DATA:{
        'min': 2,
        'max': MAX_NUM_PARAMS
    },
    REG_WRITE:{
        'min': 2,
        'max': MAX_NUM_PARAMS
    },
    ACTION:{
        'min': 0,
        'max': 0
    },
    RESET:{
        'min': 0,
        'max': 0
    },
    SYNC_WRITE:{
        'min': 4,
        'max': MAX_NUM_PARAMS
    }
}

BROADCAST_ID = 0xfe
PACKET_HEADER = bytes((0xff, 0xff))
MODEL_NUMBER = 0x00
VERSION_OF_FIRMWARE = 0x02
ID = 0x03
BAUD_RATE = 0x04
RETURN_DELAY_TIME = 0x05
CW_ANGLE_LIMIT = 0x06
CCW_ANGLE_LIMIT = 0x08
HIGHEST_LIMIT_TEMPERATURE = 0x0b
LOWEST_LIMIT_VOLTAGE = 0x0c
HIGHEST_LIMIT_VOLTAGE = 0x0d
MAX_TORQUE = 0x0e
STATUS_RETURN_LEVEL = 0x10
ALARM_LED = 0x11
ALARM_SHUTDOWN = 0x12
DOWN_CALIBRATION = 0x14
UP_CALIBRATION = 0x16
TORQUE_ENABLE = 0x18
LED = 0x19
CW_COMPLIENCE_MARGIN = 0x1a
CCW_COMPLIENCE_MARGIN = 0x1b
CW_COMPLIENCE_SLOPE = 0x1c
CCW_COMPLIENCE_SLOPE = 0x1d
GOAL_POSITION = 0x1e
MOVING_SPEED = 0x20
TORQUE_LIMIT = 0x22
PRESENT_POSITION = 0x24
PRESENT_SPEED = 0x26
PRESENT_LOAD = 0x28
PRESENT_VOLTAGE = 0x2a
PRESENT_TEMPERATURE = 0x2b
REGISTRED_INSTRUCTION = 0x2c
MOVING = 0x2e
LOCK = 0x2f
PUNCH = 0x30


def packets(self, id, instruction, parameters=None): #construction du byte array

        if parameters is None:
            parameters = bytes()
        else:
            parameters = bytes(tuple(parameters))

        #ajoute les staring bytes
        self._bytes = bytearray((0xff, 0xff))

       #vérifie si le id du moteur est valide
        if 0x00 <= id <= 0xfe:
            self._bytes.append(id)
        
        #ajoute le byte de lenght
        self._bytes.append(len(parameters) + 2)

       #ajoute les instructions
        if instruction in INSTRUCTIONS:
            self._bytes.append(instruction)

        #ajoute les parameters
        nb_param_min = NUMBER_OF_PARAMETERS[self.instruction]['min']
        nb_param_max = NUMBER_OF_PARAMETERS[self.instruction]['max']

        #vérifie le nombre de parameters
        if nb_param_min <= len(parameters) <= nb_param_max:
            self._bytes.extend(parameters)
    

        #checksum calculator
        computed_checksum = compute_checksum(self._bytes[2:])
        self._bytes.append(computed_checksum)

def compute_checksum(byte_seq):
    
    byte_seq = bytes(tuple(byte_seq))

    # vérifie la longueur des bytes
    if len(byte_seq) < 3:
        raise ValueError("erreur un minimum de 3 bytes est requis")

    # vérifie le id du moteur
    if not (0x00 <= byte_seq[0] <= 0xfe):
        msg = "mauvais id de moteur"
        raise ValueError(msg)

    # vérifie la longueur du byte
    if byte_seq[1] != (len(byte_seq) - 1):
        raise ValueError('erreur un minimum de 3 bytes est requis')

    checksum = ~sum(byte_seq) & 0xff

    return checksum


@property
def instruction(self):
        return self._bytes[4]


def set_id(self, dynamixel_id, new_id):
        self.write_data(dynamixel_id, ID, new_id)


def set_baud_rate(self, dynamixel_id, baudrate, unit="kbps"):

        if unit == "bps":
            baudrate = int(round(2000000. / baudrate)) - 1
        elif unit == "kbps":
            baudrate = int(round(2000. / baudrate)) - 1

        self.write_data(dynamixel_id, BAUD_RATE, baudrate)


def set_return_delay_time(self, dynamixel_id, return_delay_time, unit="us"):

        if unit in ("us", "usec", "microseconds"):
            return_delay_time = int(round(return_delay_time / 2.))

        self.write_data(dynamixel_id, RETURN_DELAY_TIME, return_delay_time)


def set_cw_angle_limit(self, dynamixel_id, angle_limit, degrees=False):

        if degrees:
            angle_limit = degrees_to_dxl_angle(angle_limit)

        params = int_to_little_endian_bytes(angle_limit)
        self.write_data(dynamixel_id, CW_ANGLE_LIMIT, params)


def set_ccw_angle_limit(self, dynamixel_id, angle_limit, degrees=False):
        if degrees:
            angle_limit = degrees_to_dxl_angle(angle_limit)

        params = int_to_little_endian_bytes(angle_limit)
        self.write_data(dynamixel_id, CCW_ANGLE_LIMIT, params)


def set_speed(self, dynamixel_id, speed):

        params = int_to_little_endian_bytes(speed)
        self.write_data(dynamixel_id, MOVING_SPEED, params)


def degrees_to_dxl_angle(angle_degrees):
    dxl_angle = math.floor((angle_degrees + 150.0) / 300. * 1023.)
    return dxl_angle

"""
Bytes
---------------------------------------------------------------------------------------------------------------------------------------------------------------
"""

def int_to_little_endian_bytes(integer):
    if not isinstance(integer, int):
        msg = "An integer in range(0x00, 0xffff) is required (got {})."
        raise TypeError(msg.format(type(integer)))

    # Check the argument value
    if not (0 <= integer <= 0xffff):
        msg = "An integer in range(0x00, 0xffff) is required (got {})."
        raise ValueError(msg.format(integer))

    hex_string = '%04x' % integer
    hex_tuple = (int(hex_string[2:4], 16), int(hex_string[0:2], 16))

    return hex_tuple


def little_endian_bytes_to_int(little_endian_byte_seq):

    little_endian_byte_seq = bytes(tuple(little_endian_byte_seq))

    if len(little_endian_byte_seq) != 2:
        raise ValueError("A sequence of two bytes is required.")

    integer = little_endian_byte_seq[1] * 0x100 + little_endian_byte_seq[0]

    return integer


def pretty_hex_str(byte_seq, separator=","):
    if isinstance(byte_seq, int):
        byte_seq = bytes((byte_seq, ))
    else:
        byte_seq = bytes(byte_seq)

    return separator.join(['%02x' % byte for byte in byte_seq])


def dxl_angle_to_degrees(dxl_angle):
    angle_degrees = round(dxl_angle / 1023. * 300. - 150.0, 1)
    return angle_degrees


"""
Arg Parser
---------------------------------------------------------------------------------------------------------------------------------------------------------------
"""
def common_argument_parser(desc, id_arg=True, id_arg_mandatory=False):
    parser = argparse.ArgumentParser(description=desc)

    if id_arg:
        if id_arg_mandatory:
            parser.add_argument("--dynamixel_id",
                                "-i",
                                help=ID_HELP_STR,
                                metavar="INTEGER",
                                type=int,
                                required=True,
                                default=BROADCAST_ID)
        else:
            parser.add_argument("--dynamixel_id",
                                "-i",
                                help=ID_HELP_STR,
                                metavar="INTEGER",
                                type=int,
                                default=BROADCAST_ID)

    parser.add_argument("--baudrate",
                        "-b",
                        help=BAUD_RATE_HELP_STR,
                        metavar="INTEGER",
                        type=int,
                        default=57600)

    parser.add_argument("--timeout",
                        "-t",
                        help=TIMEOUT_HELP_STR,
                        metavar="FLOAT",
                        type=float,
                        default=0.1)

    parser.add_argument("--port",
                        "-p",
                        help=PORT_HELP_STR,
                        metavar="STRING",
                        default="/dev/ttyUSB0")

    parser.add_argument("--rpi",
                        help=RPI_HELP_STR,
                        action="store_true")

    return parser

ID_HELP_STR = ("The unique ID of a Dynamixel unit to work with "
               "(254 is a broadcasting ID)")

BAUD_RATE_HELP_STR = "The baud rate speed (e.g. 57600)"

TIMEOUT_HELP_STR = "The timeout value for the connection"

PORT_HELP_STR = ("The serial device to connect with "
                 "(e.g. '/dev/ttyUSB0' for Unix users "
                 "or 'COM1' for Windows users)")

RPI_HELP_STR = "Use Raspberry Pi GPIO to connect Dynamixels"


"""
Packet
------------------------------------------------------------------------------------------------------------------------------------------------------------
""" 

def __init__(self, dynamixel_id, data):

        # Check the data bytes.
        # "TypeError" and "ValueError" are raised by the "bytes" constructor if
        # necessary.
        if isinstance(data, int):
            data = bytes((data, )) # convert integers to a sequence
        else:
            data = bytes(data)

        # Add the header bytes.
        self._bytes = bytearray((0xff, 0xff))

        # Check and add the Dynamixel ID byte.
        # "TypeError" and "ValueError" are raised by the "bytearray.append()"
        # if necessary.
        if 0x00 <= dynamixel_id <= 0xfe:
            self._bytes.append(dynamixel_id)
        else:
            msg = "Wrong dynamixel_id: {:#x} (should be in range(0x00, 0xfe))."
            raise ValueError(msg.format(dynamixel_id))

        # Add the length byte.
        self._bytes.append(len(data) + 1)

        # Add the data bytes.
        self._bytes.extend(data)

        # Add the checksum byte.
        computed_checksum = compute_checksum(self._bytes[2:])
        self._bytes.append(computed_checksum)

		
@property
def header(self):
        return self._bytes[0:2]
@property
def id(self):
        return self._bytes[2]
@property
def length(self):
        return self._bytes[3]
@property
def parameters(self):
        return self._bytes[5:-1]
@property
def data(self):
        return self._bytes[4:-1]
@property
def checksum(self):
        return self._bytes[-1]


"""
Connection
--------------------------------------------------------------------------------------------------------------------------------------------------------------
"""

