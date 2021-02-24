""" packet sample
 +----+----+--+------+-----------+----------+---+-----------+---------+
    |0XFF|0XFF|ID|LENGTH|INSTRUCTION|PARAMETER1|...|PARAMETER N|CHECK SUM|
    +----+----+--+------+-----------+----------+---+-----------+---------+
"""

import sys
import time
import math
import argparse
from serial import Serial
from serial import SerialException

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
    PING: {
        'min': 0,
        'max': 0
    },
    READ_DATA: {
        'min': 2,
        'max': 2
    },
    WRITE_DATA: {
        'min': 2,
        'max': MAX_NUM_PARAMS
    },
    REG_WRITE: {
        'min': 2,
        'max': MAX_NUM_PARAMS
    },
    ACTION: {
        'min': 0,
        'max': 0
    },
    RESET: {
        'min': 0,
        'max': 0
    },
    SYNC_WRITE: {
        'min': 4,
        'max': MAX_NUM_PARAMS
    }
}

BROADCAST_ID = 0xfe
PACKET_HEADER = bytes((0xff, 0xff))
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


def packets(self, id, instruction, parameters=None):  # construction du byte array

    if parameters is None:
        parameters = bytes()
    else:
        parameters = bytes(tuple(parameters))

    # ajoute les staring bytes
    self._bytes = bytearray((0xff, 0xff))

   # vérifie si le id du moteur est valide
    if 0x00 <= id <= 0xfe:
        self._bytes.append(id)

    # ajoute le byte de lenght
    self._bytes.append(len(parameters) + 2)

   # ajoute les instructions
    if instruction in INSTRUCTIONS:
        self._bytes.append(instruction)

    # ajoute les parameters
    nb_param_min = NUMBER_OF_PARAMETERS[self.instruction]['min']
    nb_param_max = NUMBER_OF_PARAMETERS[self.instruction]['max']

    # vérifie le nombre de parameters
    if nb_param_min <= len(parameters) <= nb_param_max:
        self._bytes.extend(parameters)

    # checksum calculator
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
            parser.add_argument("--id",
                                "-i",
                                help=ID_HELP_STR,
                                metavar="INTEGER",
                                type=int,
                                required=True,
                                default=BROADCAST_ID)
        else:
            parser.add_argument("--id",
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


def initiate(self, id, data):
    if isinstance(data, int):
        data = bytes((data, ))
    else:
        data = bytes(data)

    self._bytes = bytearray((0xff, 0xff))
    if 0x00 <= id <= 0xfe:
        self._bytes.append(id)
    else:
        msg = "Wrong id: {:#x} (should be in range(0x00, 0xfe))."
        raise ValueError(msg.format(id))

    self._bytes.append(len(data) + 1)
    self._bytes.extend(data)
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
Packets
--------------------------------------------------------------------------------------------------------------------------------------------------------------
"""


class Packet(object):

    def __init__(self, id, data):
        if isinstance(data, int):
            data = bytes((data, ))
        else:
            data = bytes(data)

        self._bytes = bytearray((0xff, 0xff))

        if 0x00 <= id <= 0xfe:
            self._bytes.append(id)
        else:
            msg = "Wrong id: {:#x} (should be in range(0x00, 0xfe))."
            raise ValueError(msg.format(id))

        self._bytes.append(len(data) + 1)
        self._bytes.extend(data)
        computed_checksum = compute_checksum(self._bytes[2:])
        self._bytes.append(computed_checksum)

    def to_byte_array(self):
        return bytearray(self._bytes)
    def to_bytes(self):
        return bytes(self._bytes)
    def to_integer_tuple(self):
        return tuple(self._bytes)

    def to_printable_string(self):
        packet_str = ' '.join(['%02x' % byte for byte in self._bytes])
        return packet_str

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
Instruction packets
-----------------------------------------------------------------------------------------------------------------------------------------------------------------
"""


class InstructionPacket(Packet):

    def __init__(self, id, instruction, parameters=None):
        if parameters is None:
            parameters = bytes()
        else:
            parameters = bytes(tuple(parameters))

        self._bytes = bytearray((0xff, 0xff))
        if 0x00 <= id <= 0xfe:
            self._bytes.append(id)
        else:
            if isinstance(id, int):
                msg = ("Wrong id value, "
                       "an integer in range(0x00, 0xfe) is required.")
                raise ValueError(msg)
            else:
                raise TypeError("Wrong id type (integer required).")

        self._bytes.append(len(parameters) + 2)
        if instruction in INSTRUCTIONS:
            self._bytes.append(instruction)
        else:
            if isinstance(instruction, int):
                msg = "Wrong instruction, should be in ({})."
                instructions_str = pretty_hex_str(INSTRUCTIONS)
                raise ValueError(msg.format(instructions_str))
            else:
                raise TypeError("Wrong instruction type (integer required).")

        nb_param_min = NUMBER_OF_PARAMETERS[self.instruction]['min']
        nb_param_max = NUMBER_OF_PARAMETERS[self.instruction]['max']

        if nb_param_min <= len(parameters) <= nb_param_max:
            self._bytes.extend(parameters)
        else:
            msg = ("Wrong number of parameters: {} parameters "
                   "(min expected={}; max expected={}).")
            nb_param = len(parameters)
            raise ValueError(msg.format(nb_param, nb_param_min, nb_param_max))

        computed_checksum = compute_checksum(self._bytes[2:])
        self._bytes.append(computed_checksum)

    @property
    def instruction(self):
        return self._bytes[4]


"""
Exception classes
-----------------------------------------------------------------------------------------------------------------------------------------------------------
"""

class StatusPacketError(Exception):
    pass
class InstructionError(StatusPacketError):
    pass
class OverloadError(StatusPacketError):
    pass
class InstructionChecksumError(StatusPacketError):
    pass
class StatusChecksumError(StatusPacketError):
    pass
class RangeError(StatusPacketError):
    pass
class OverheatingError(StatusPacketError):
    pass
class AngleLimitError(StatusPacketError):
    pass
class InputVoltageError(StatusPacketError):
    pass

"""
Status Packet
-------------------------------------------------------------------------------------------------------------------------------------------------------------
"""


class StatusPacket(Packet):

    def __init__(self, packet):
        self._bytes = bytes(tuple(packet))
        if len(self._bytes) < 6:
            raise ValueError("Incomplete packet.")
        if bytes(self.header) != b'\xff\xff':
            raise ValueError("Wrong header (should be b'\xff\xff').")

        if self.length != len(self._bytes) - 4:
            raise ValueError('Wrong length byte.')

        computed_checksum = compute_checksum(self._bytes[2:-1])
        if computed_checksum != self.checksum:
            raise StatusChecksumError('Wrong checksum.')

        if self.instruction_error:
            raise InstructionError()

        if self.overload_error:
            raise OverloadError()

        if self.checksum_error:
            raise InstructionChecksumError()

        if self.range_error:
            raise RangeError()

        if self.overheating_error:
            raise OverheatingError()

        if self.angle_limit_error:
            raise AngleLimitError()

        if self.input_voltage_error:
            raise InputVoltageError()

        if not(0x00 <= self.id <= 0xfd):
            msg = "Wrong id, a value in range (0, 0xFD) is required."
            raise ValueError(msg)


    @property
    def error(self):
        return self._bytes[4]

    @property
    def instruction_error(self):
        return bool(self.error & (1 << 6))

    @property
    def overload_error(self):
        return bool(self.error & (1 << 5))

    @property
    def checksum_error(self):
        return bool(self.error & (1 << 4))

    @property
    def range_error(self):
        return bool(self.error & (1 << 3))

    @property
    def overheating_error(self):
        return bool(self.error & (1 << 2))

    @property
    def angle_limit_error(self):
        return bool(self.error & (1 << 1))

    @property
    def input_voltage_error(self):
        return bool(self.error & (1 << 0))


"""
Connection
--------------------------------------------------------------------------------------------------------------------------------------------------------------
"""


def __init__(self, port='/dev/ttyUSB0', baudrate=1000000, timeout=0.1,
             waiting_time=0.02, rpi_gpio=False):

    self.rpi_gpio = False

    if rpi_gpio:
        if "RPi" in sys.modules:
            self.rpi_gpio = True
        else:
            raise Exception("RPi.GPIO cannot be imported")

    self.waiting_time = waiting_time

    self.port = port
    self.baudrate = baudrate
    self.timeout = timeout

    if self.rpi_gpio:
        gpio.setmode(gpio.BCM)
        gpio.setup(18, gpio.OUT)

    self.serial_connection = serial.Serial(port=self.port,
                                           baudrate=self.baudrate,
                                           timeout=self.timeout,
                                           bytesize=serial.EIGHTBITS,
                                           parity=serial.PARITY_NONE,
                                           stopbits=serial.STOPBITS_ONE)


def send(self, instruction_packet):

    if isinstance(instruction_packet, bytes):
        instruction_packet_bytes = instruction_packet
    else:
        instruction_packet_bytes = instruction_packet.to_bytes()

    self.flush()

    if self.rpi_gpio:

        gpio.output(18, gpio.HIGH)
        time.sleep(0.01)

    self.serial_connection.write(instruction_packet_bytes)

    if self.rpi_gpio:
        self.serial_connection.flushOutput()
        time.sleep(0.00017 * len(instruction_packet_bytes))
        if self.rpi_gpio:
            gpio.output(18, gpio.LOW)

        time.sleep(0.004)
    else:
        time.sleep(self.waiting_time)
    num_bytes_available = self.serial_connection.inWaiting()
    status_packet_bytes = self.serial_connection.read(num_bytes_available)

    status_packet = None
    if len(status_packet_bytes) > 0:
        status_packet = StatusPacket(status_packet_bytes)

    return status_packet


def close(self):
    self.serial_connection.close()

    if self.rpi_gpio:
        pass


def flush(self):

    self.serial_connection.flushInput()


def read_data(self, id, address, length):

    instruction = READ_DATA
    params = (address, length)
    inst_packet = InstructionPacket(id, instruction, params)
    status_packet = self.send(inst_packet)

    data_bytes = None
    if status_packet is not None:
        if status_packet.id == id:
            data_bytes = status_packet.parameters
    return data_bytes


def write_data(self, id, address, data):

    bytes_address = bytes((address, ))
    if isinstance(data, int):
        bytes_to_write = bytes((data, ))
    else:
        bytes_to_write = bytes(data)
    instruction = WRITE_DATA
    params = bytes_address + bytes_to_write
    inst_packet = InstructionPacket(id, instruction, params)
    self.send(inst_packet)


def ping(self, id):
    instruction = PING
    inst_packet = InstructionPacket(id, instruction)

    status_packet = self.send(inst_packet)

    is_available = False
    if status_packet is not None:
        if status_packet.id == id:
            is_available = True
    return is_available


def dump_control_table(self, id):
    byte_seq = self.read_data(id, 0, 50)
    return byte_seq


def print_control_table(self, id):

    byte_seq = self.dump_control_table(id)
    control_table_str = pretty_hex_str(byte_seq, ' ')

    print(control_table_str)


def get_control_table_tuple(self, id):

    dxl_id = id

    def angle_to_str(dxl_angle):
        angle_degrees = dxl_angle_to_degrees(dxl_angle)
        angle_str = "{}° ({})".format(angle_degrees, dxl_angle)
        return angle_str

    def abs_angle_to_str(dxl_angle):
        angle_degrees = round(dxl_angle / 1023. * 300., 1)
        angle_str = "{}° ({})".format(angle_degrees, dxl_angle)
        return angle_str

    baud_rate_str = "%s bps" % self.get_baud_rate(dxl_id)
    return_delay_time_str = "%s µs" % self.get_return_delay_time(dxl_id)
    cw_angle_limit_str = angle_to_str(self.get_cw_angle_limit(dxl_id))
    ccw_angle_limit_str = angle_to_str(self.get_ccw_angle_limit(dxl_id))
    max_temperature_str = "%s°C" % self.get_max_temperature(dxl_id)
    min_voltage_str = "%sV" % self.get_min_voltage(dxl_id)
    max_voltage_str = "%sV" % self.get_max_voltage(dxl_id)
    max_torque = self.get_max_torque(dxl_id)

    if max_torque == 0:
        max_torque_str = "0 (free run mode)"
    else:
        max_torque_str = max_torque

    status_return_level = self.get_status_return_level(dxl_id)
    if status_return_level == 0:
        status_return_level_str = "0 (do not respond to any instructions)"
    elif status_return_level == 1:
        status_return_level_str = ("1 (respond only to READ_DATA"
                                       " instructions)")
    elif status_return_level == 2:
        status_return_level_str = "2 (respond to all instructions)"
    else:
        status_return_level_str = "%i (unknown)" % status_return_level

    voltage_alarm_led = self.has_input_voltage_alarm_led(dxl_id)
    angle_limit_alarm_led = self.has_angle_limit_alarm_led(dxl_id)
    overheating_alarm_led = self.has_overheating_alarm_led(dxl_id)
    range_alarm_led = self.has_range_alarm_led(dxl_id)
    checksum_alarm_led = self.has_checksum_alarm_led(dxl_id)
    overload_alarm_led = self.has_overload_alarm_led(dxl_id)
    instruction_alarm_led = self.has_instruction_alarm_led(dxl_id)
    voltage_alarm_led_str = "on" if voltage_alarm_led else "off"
    angle_limit_alarm_led_str = "on" if angle_limit_alarm_led else "off"
    overheating_alarm_led_str = "on" if overheating_alarm_led else "off"
    range_alarm_led_str = "on" if range_alarm_led else "off"
    checksum_alarm_led_str = "on" if checksum_alarm_led else "off"
    overload_alarm_led_str = "on" if overload_alarm_led else "off"
    instruction_alarm_led_str = "on" if instruction_alarm_led else "off"
    voltage_alarm_off = self.has_input_voltage_alarm_shutdown(dxl_id)
    angle_limit_alarm_off = self.has_angle_limit_alarm_shutdown(dxl_id)
    overheating_alarm_off = self.has_overheating_alarm_shutdown(dxl_id)
    range_alarm_off = self.has_range_alarm_shutdown(dxl_id)
    checksum_alarm_off = self.has_checksum_alarm_shutdown(dxl_id)
    overload_alarm_off = self.has_overload_alarm_shutdown(dxl_id)
    instruction_alarm_off = self.has_instruction_alarm_shutdown(dxl_id)
    voltage_alarm_off_str = "on" if voltage_alarm_off else "off"
    angle_limit_alarm_off_str = "on" if angle_limit_alarm_off else "off"
    overheating_alarm_off_str = "on" if overheating_alarm_off else "off"
    range_alarm_off_str = "on" if range_alarm_off else "off"
    checksum_alarm_off_str = "on" if checksum_alarm_off else "off"
    overload_alarm_off_str = "on" if overload_alarm_off else "off"
    instruction_alarm_off_str = "on" if instruction_alarm_off else "off"
    torque_enable_str = "yes" if self.is_torque_enable(dxl_id) else "no"
    led_str = "on" if self.is_led_enabled(dxl_id) else "off"
    cw_compliance_margin = self.get_cw_compliance_margin(dxl_id)
    ccw_compliance_margin = self.get_ccw_compliance_margin(dxl_id)
    cw_compliance_slope = self.get_cw_compliance_slope(dxl_id)
    ccw_compliance_slope = self.get_ccw_compliance_slope(dxl_id)
    cw_compliance_margin_str = abs_angle_to_str(cw_compliance_margin)
    ccw_compliance_margin_str = abs_angle_to_str(ccw_compliance_margin)
    cw_compliance_slope_str = abs_angle_to_str(cw_compliance_slope)
    ccw_compliance_slope_str = abs_angle_to_str(ccw_compliance_slope)
    goal_position_str = angle_to_str(self.get_goal_position(dxl_id))
    position_str = angle_to_str(self.get_present_position(dxl_id))
    voltage_str = "%sV" % self.get_present_voltage(dxl_id)
    temperature_str = "%s°C" % self.get_present_temperature(dxl_id)

    if self.has_registred_instruction(dxl_id):
        registred_inst_str = "yes"
    else:
        registred_inst_str = "no"

        moving_str = "yes" if self.is_moving(dxl_id) else "no"
        locked_str = "yes" if self.is_locked(dxl_id) else "no"

    ctrl_table_tuple = (
        ("firmware_version", self.get_firmware_version(dxl_id)),
        ("id", dxl_id),
        ("baud_rate", baud_rate_str),
        ("return_delay_time", return_delay_time_str),
        ("cw_angle_limit", cw_angle_limit_str),
        ("ccw_angle_limit", ccw_angle_limit_str),
        ("max_temperature", max_temperature_str),
        ("min_voltage", min_voltage_str),
        ("max_voltage", max_voltage_str),
        ("max_torque", max_torque_str),
        ("status_return_level", status_return_level_str),
        ("input_voltage_alarm_led", voltage_alarm_led_str),
        ("angle_limit_alarm_led", angle_limit_alarm_led_str),
        ("overheating_alarm_led", overheating_alarm_led_str),
        ("range_alarm_led", range_alarm_led_str),
        ("checksum_alarm_led", checksum_alarm_led_str),
        ("overload_alarm_led", overload_alarm_led_str),
        ("instruction_alarm_led", instruction_alarm_led_str),
        ("input_voltage_alarm_shutdown", voltage_alarm_off_str),
        ("angle_limit_alarm_shutdown", angle_limit_alarm_off_str),
        ("overheating_alarm_shutdown", overheating_alarm_off_str),
        ("range_alarm_shutdown", range_alarm_off_str),
        ("checksum_alarm_shutdown", checksum_alarm_off_str),
        ("overload_alarm_shutdown", overload_alarm_off_str),
        ("instruction_alarm_shutdown", instruction_alarm_off_str),
        ("down_calibration", self.get_down_calibration(dxl_id)),
        ("up_calibration", self.get_up_calibration(dxl_id)),
        ("torque_enabled", torque_enable_str),
        ("led", led_str),
        ("cw_compliance_margin", cw_compliance_margin_str),
        ("ccw_compliance_margin", ccw_compliance_margin_str),
        ("cw_compliance_slope", cw_compliance_slope_str),
        ("ccw_compliance_slope", ccw_compliance_slope_str),
        ("goal_position", goal_position_str),
        ("moving_speed", self.get_moving_speed(dxl_id)),
        ("torque_limit", self.get_torque_limit(dxl_id)),
        ("present_position", position_str),
        ("present_speed", self.get_present_speed(dxl_id)),
        ("present_load", self.get_present_load(dxl_id)),
        ("present_voltage", voltage_str),
        ("present_temperature", temperature_str),
        ("registred_instruction", registred_inst_str),
        ("moving", moving_str),
        ("locked", locked_str),
        ("punch", self.get_punch(dxl_id)),
    )
    return ctrl_table_tuple


def pretty_print_control_table(self, id):
    ctrl_table_tuple = self.get_control_table_tuple(id)
    for key, value in ctrl_table_tuple:
        print("{:.<29} {}".format(key, value))


def scan(self, id_bytes=None):

    available_ids = bytearray()
    if id_bytes is None:
        id_bytes = bytes(range(0xfe))

    for id in id_bytes:
        if 0 <= id <= 0xfd:
            if self.ping(id):
                available_ids.append(id)
    return available_ids


def get_firmware_version(self, id):
    byte_seq = self.read_data(id, VERSION_OF_FIRMWARE, 1)
    return byte_seq[0]


def get_baud_rate(self, id):
    byte_seq = self.read_data(id, BAUD_RATE, 1)
    raw_value = byte_seq[0]
    baud_rate = 2000000 / (raw_value + 1)
    return round(baud_rate, 1)


def get_return_delay_time(self, id):
    byte_seq = self.read_data(id, RETURN_DELAY_TIME, 1)
    raw_value = byte_seq[0]
    delay_time = 2 * raw_value
    return delay_time


def get_cw_angle_limit(self, id):
    byte_seq = self.read_data(id, CW_ANGLE_LIMIT, 2)
    return little_endian_bytes_to_int(byte_seq)


def get_ccw_angle_limit(self, id):
    byte_seq = self.read_data(id, CCW_ANGLE_LIMIT, 2)
    return little_endian_bytes_to_int(byte_seq)


def get_max_temperature(self, id):
    byte_seq = self.read_data(id, HIGHEST_LIMIT_TEMPERATURE, 1)
    return byte_seq[0]


def get_min_voltage(self, id):
    byte_seq = self.read_data(id, LOWEST_LIMIT_VOLTAGE, 1)
    raw_value = byte_seq[0]

    min_voltage = raw_value / 10.
    return min_voltage


def get_max_voltage(self, id):
    byte_seq = self.read_data(id, HIGHEST_LIMIT_VOLTAGE, 1)
    raw_value = byte_seq[0]

    max_voltage = raw_value / 10.
    return max_voltage


def get_max_torque(self, id):
    byte_seq = self.read_data(id, MAX_TORQUE, 2)
    return little_endian_bytes_to_int(byte_seq)


def get_status_return_level(self, id):
    byte_seq = self.read_data(id, STATUS_RETURN_LEVEL, 1)
    return byte_seq[0]


def has_input_voltage_alarm_led(self, id):
    byte_seq = self.read_data(id, ALARM_LED, 1)
    alarm_led_byte = byte_seq[0]

    return bool(alarm_led_byte & (1 << 0))


def has_angle_limit_alarm_led(self, id):
    byte_seq = self.read_data(id, ALARM_LED, 1)
    alarm_led_byte = byte_seq[0]

    return bool(alarm_led_byte & (1 << 1))


def has_overheating_alarm_led(self, id):
    byte_seq = self.read_data(id, ALARM_LED, 1)
    alarm_led_byte = byte_seq[0]

    return bool(alarm_led_byte & (1 << 2))


def has_range_alarm_led(self, id):
    byte_seq = self.read_data(id, ALARM_LED, 1)
    alarm_led_byte = byte_seq[0]

    return bool(alarm_led_byte & (1 << 3))


def has_checksum_alarm_led(self, id):
    byte_seq = self.read_data(id, ALARM_LED, 1)
    alarm_led_byte = byte_seq[0]

    return bool(alarm_led_byte & (1 << 4))


def has_overload_alarm_led(self, id):
    byte_seq = self.read_data(id, ALARM_LED, 1)
    alarm_led_byte = byte_seq[0]

    return bool(alarm_led_byte & (1 << 5))


def has_instruction_alarm_led(self, id):
    byte_seq = self.read_data(id, ALARM_LED, 1)
    alarm_led_byte = byte_seq[0]

    return bool(alarm_led_byte & (1 << 6))


def has_input_voltage_alarm_shutdown(self, id):
    byte_seq = self.read_data(id, ALARM_SHUTDOWN, 1)
    alarm_shutdown_byte = byte_seq[0]

    return bool(alarm_shutdown_byte & (1 << 0))


def has_angle_limit_alarm_shutdown(self, id):
    byte_seq = self.read_data(id, ALARM_SHUTDOWN, 1)
    alarm_shutdown_byte = byte_seq[0]

    return bool(alarm_shutdown_byte & (1 << 1))


def has_overheating_alarm_shutdown(self, id):
    byte_seq = self.read_data(id, ALARM_SHUTDOWN, 1)
    alarm_shutdown_byte = byte_seq[0]

    return bool(alarm_shutdown_byte & (1 << 2))


def has_range_alarm_shutdown(self, id):
    byte_seq = self.read_data(id, ALARM_SHUTDOWN, 1)
    alarm_shutdown_byte = byte_seq[0]

    return bool(alarm_shutdown_byte & (1 << 3))


def has_checksum_alarm_shutdown(self, id):
    byte_seq = self.read_data(id, ALARM_SHUTDOWN, 1)
    alarm_shutdown_byte = byte_seq[0]

    return bool(alarm_shutdown_byte & (1 << 4))


def has_overload_alarm_shutdown(self, id):
    byte_seq = self.read_data(id, ALARM_SHUTDOWN, 1)
    alarm_shutdown_byte = byte_seq[0]

    return bool(alarm_shutdown_byte & (1 << 5))


def has_instruction_alarm_shutdown(self, id):
    byte_seq = self.read_data(id, ALARM_SHUTDOWN, 1)
    alarm_shutdown_byte = byte_seq[0]

    return bool(alarm_shutdown_byte & (1 << 6))


def get_down_calibration(self, id):
    byte_seq = self.read_data(id, DOWN_CALIBRATION, 2)
    return little_endian_bytes_to_int(byte_seq)


def get_up_calibration(self, id):
    byte_seq = self.read_data(id, UP_CALIBRATION, 2)
    return little_endian_bytes_to_int(byte_seq)


def is_torque_enable(self, id):
    byte_seq = self.read_data(id, TORQUE_ENABLE, 1)
    return byte_seq[0] == 1


def is_led_enabled(self, id):
    byte_seq = self.read_data(id, LED, 1)
    return byte_seq[0] == 1


def get_cw_compliance_margin(self, id):
    byte_seq = self.read_data(id, CW_COMPLIENCE_MARGIN, 1)
    return byte_seq[0]


def get_ccw_compliance_margin(self, id):
    byte_seq = self.read_data(id, CCW_COMPLIENCE_MARGIN, 1)
    return byte_seq[0]


def get_cw_compliance_slope(self, id):
    byte_seq = self.read_data(id, CW_COMPLIENCE_SLOPE, 1)
    return byte_seq[0]


def get_ccw_compliance_slope(self, id):
    byte_seq = self.read_data(id, CCW_COMPLIENCE_SLOPE, 1)
    return byte_seq[0]


def get_goal_position(self, id):
    byte_seq = self.read_data(id, GOAL_POSITION, 2)
    return little_endian_bytes_to_int(byte_seq)


def get_moving_speed(self, id):
    byte_seq = self.read_data(id, MOVING_SPEED, 2)
    return little_endian_bytes_to_int(byte_seq)


def get_torque_limit(self, id):
    byte_seq = self.read_data(id, TORQUE_LIMIT, 2)
    return little_endian_bytes_to_int(byte_seq)


def get_present_position(self, id, degrees=False):
    byte_seq = self.read_data(id, PRESENT_POSITION, 2)
    position = little_endian_bytes_to_int(byte_seq)
    if degrees:
        position = dxl_angle_to_degrees(position)
    return position


def get_present_speed(self, id):
    byte_seq = self.read_data(id, PRESENT_SPEED, 2)
    return little_endian_bytes_to_int(byte_seq)


def get_present_load(self, id):
    byte_seq = bytearray(self.read_data(id, PRESENT_LOAD, 2))
    load_direction = -1 if (byte_seq[1] & (1 << 2)) == 0 else 1
    byte_seq[1] = 0b00000011 & byte_seq[1]
    abs_load = little_endian_bytes_to_int(byte_seq)
    load = load_direction * abs_load
    return load


def get_present_voltage(self, id):
    byte_seq = self.read_data(id, PRESENT_VOLTAGE, 1)
    raw_value = byte_seq[0]
    voltage = raw_value / 10.
    return voltage


def get_present_temperature(self, id):
    byte_seq = self.read_data(id, PRESENT_TEMPERATURE, 1)
    return byte_seq[0]


def has_registred_instruction(self, id):
    byte_seq = self.read_data(id, REGISTRED_INSTRUCTION, 1)
    return byte_seq[0] == 1


def is_moving(self, id):
    byte_seq = self.read_data(id, MOVING, 1)
    return byte_seq[0] == 1


def is_locked(self, id):
    byte_seq = self.read_data(id, LOCK, 1)
    return byte_seq[0] == 1


def get_punch(self, id):
    byte_seq = self.read_data(id, PUNCH, 2)
    return little_endian_bytes_to_int(byte_seq)


def set_id(self, id, new_id):
    self.write_data(id, ID, new_id)


def set_baud_rate(self, id, baudrate, unit="kbps"):

    if unit == "bps":
        baudrate = int(round(2000000. / baudrate)) - 1
    elif unit == "kbps":
        baudrate = int(round(2000. / baudrate)) - 1
    self.write_data(id, BAUD_RATE, baudrate)


def set_return_delay_time(self, id, return_delay_time, unit="us"):

    if unit in ("us", "usec", "microseconds"):
        return_delay_time = int(round(return_delay_time / 2.))
    self.write_data(id, RETURN_DELAY_TIME, return_delay_time)


def set_cw_angle_limit(self, id, angle_limit, degrees=False):

    if degrees:
        angle_limit = degrees_to_dxl_angle(angle_limit)
    params = int_to_little_endian_bytes(angle_limit)
    self.write_data(id, CW_ANGLE_LIMIT, params)


def set_ccw_angle_limit(self, id, angle_limit, degrees=False):

    if degrees:
        angle_limit = degrees_to_dxl_angle(angle_limit)
    params = int_to_little_endian_bytes(angle_limit)
    self.write_data(id, CCW_ANGLE_LIMIT, params)


def set_speed(self, id, speed):

    params = int_to_little_endian_bytes(speed)
    self.write_data(id, MOVING_SPEED, params)


def goto(self, id, position, speed=None, degrees=False):

    if degrees:
        position = degrees_to_dxl_angle(position)
    params = int_to_little_endian_bytes(position)

    if speed is not None:
        params += int_to_little_endian_bytes(speed)
    self.write_data(id, GOAL_POSITION, params)