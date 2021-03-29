# -*- coding : utf-8 -*-

# PyAX-12

# The MIT License
#
# Copyright (c) 2010,2015 Jeremie DECOCK (http://www.jdhp.org)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
This module contain the `Connection` class communicate with Dynamixel units.
"""

__all__ = ['Connection']

import serial
import sys
import time

import packet as pk
import status_packet as sp
import instruction_packet as ip

try:
    import RPi.GPIO as gpio
except:
    pass

import utils

class Connection(object):
    """Create a serial connection with dynamixel actuators.

    :param str port: the serial device to connect with (e.g. '/dev/ttyUSB0'
        for Unix users or 'COM1' for windows users).
    :param int baudrate: the baud rate speed (e.g. 57600).
    :param float timeout: the timeout value for the connection.
    :param float waiting_time: the waiting time (in seconds) between sending
        the instruction packet and the receiving the status packet.
    """

    def __init__(self, port='/dev/ttyUSB0', baudrate=1000000, timeout=0.1,
                 waiting_time=2, rpi_gpio=False):

        self.rpi_gpio = False

        if rpi_gpio:
            if "RPi" in sys.modules:
                self.rpi_gpio = True
            else:
                raise Exception("RPi.GPIO cannot be imported")   # TODO: improve this ?

        self.waiting_time = 1

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        if self.rpi_gpio:
            gpio.setmode(gpio.BCM)
            gpio.setup(18, gpio.OUT)  # Set the direction to output (send data)

        self.serial_connection = serial.Serial(port=self.port,
                                               baudrate=self.baudrate,
                                               timeout=self.timeout,
                                               bytesize=serial.EIGHTBITS,
                                               parity=serial.PARITY_NONE,
                                               stopbits=serial.STOPBITS_ONE)

    def send(self, instruction_packet):
        """Send an instruction packet.

        :param instruction_packet: can be either a `Packet` instance or a
            "bytes" string containing the full instruction packet to be sent to
            Dynamixel units.
        """

        if isinstance(instruction_packet, bytes):
            # instruction_packet is a bytes instance
            instruction_packet_bytes = instruction_packet
        else:
            # instruction_packet is a Packet instance
            instruction_packet_bytes = instruction_packet.to_bytes()

        self.flush()      # TODO: make a (synchronous) flush_in() and flush_out() instead

        # Send the packet #################################

        if self.rpi_gpio:
            # Pin 18 = +3V (DATA status = send data to Dynamixel)
            gpio.output(18, gpio.HIGH)
            time.sleep(0.001)          # TODO

        self.serial_connection.write(instruction_packet_bytes)

        # Sleep a little bit ##############################

        if self.rpi_gpio:
            self.serial_connection.flushOutput()
            time.sleep(0.000035 * len(instruction_packet_bytes))  # TODO: check instead if the output buffer is empty or make the previous flushOutput synchronous

            if self.rpi_gpio:
                # Pin 18 = 0V (DATA status = receive data from Dynamixel)
                gpio.output(18, gpio.LOW)

            time.sleep(0.004)              # TODO: make a while loop with a timeout instead ?
        else:
            time.sleep(self.waiting_time)  # TODO: make a while loop with a timeout instead ?

        # Receive the reply (status packet) ###############

        # WARNING:
        # If you use the USB2Dynamixel device, make sure its switch is set on
        # "TTL" (otherwise status packets won't be readable).

       


    def close(self):
        """Close the serial connection."""

        # TODO: flush ?
        self.serial_connection.close()

        if self.rpi_gpio:
            pass     # TODO: put back gpio to its default setup ?
            #gpio.setmode(gpio.BCM)
            #gpio.setup(18, gpio.OUT)  # Set the direction to output (send data)


    def flush(self):
        """Flush the connection buffers."""

        self.serial_connection.flushInput()
        #self.serial_connection.flushOutput()  # TODO ?


    ## HIGH LEVEL FUNCTIONS ####################################################

    def read_data(self, dynamixel_id, address, length):
        """Read bytes form the control table of the specified Dynamixel unit.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        :param int address: the starting address of the location where the data
            is to be read.
        :param int length: the length of the data to be read.
        """

        instruction = ip.READ_DATA
        params = (address, length)
        inst_packet = ip.InstructionPacket(dynamixel_id, instruction, params)

        status_packet = self.send(inst_packet)

        data_bytes = None
        if status_packet is not None:
            if status_packet.dynamixel_id == dynamixel_id:
                data_bytes = status_packet.parameters
            else:
                pass # TODO: exception ?
        # TODO: exception if dxl_id = 0xFE

        return data_bytes


    def write_data(self, dynamixel_id, address, data):
        """Write bytes to the control table of the specified Dynamixel unit.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFE).
        :param int address: the starting address of the location where the data
            is to be written.
        :param bytes data: the bytes of the data to be written (it can be an
            integer, a sequence of integer, a bytes or a bytearray).
        """

        bytes_address = bytes((address, ))

        if isinstance(data, int):
            bytes_to_write = bytes((data, ))
        else:
            bytes_to_write = bytes(data)

        instruction = ip.WRITE_DATA
        params = bytes_address + bytes_to_write
        inst_packet = ip.InstructionPacket(dynamixel_id, instruction, params)

        self.send(inst_packet)


    def ping(self, dynamixel_id):
        """Ping the specified Dynamixel unit.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        :return: ``True`` if the specified unit is available, ``False``
            otherwise.
        """

        instruction = ip.PING
        inst_packet = ip.InstructionPacket(dynamixel_id, instruction)

        status_packet = self.send(inst_packet)

        is_available = False
        if status_packet is not None:
            if status_packet.dynamixel_id == dynamixel_id:
                is_available = True
            else:
                pass # TODO: exception ?
        # TODO: exception if dxl_id = 0xFE

        return is_available


    #def reset(self, dynamixel_id):
    #
    #    status_packet = self.send(instruction_packet)
    #
    #    if status_packet is not None:
    #        pass
    #        # TODO warning


    ## HIGHEST LEVEL FUNCTIONS #################################################

    def dump_control_table(self, dynamixel_id):
        """Dump the *control table* of the specified Dynamixel unit.

        This function can be used to backup the current configuration of the
        given Dynamixel unit.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        :return: the sequence of all bytes in currently the *control table*.
        """

        byte_seq = self.read_data(dynamixel_id, 0, 50)
        return byte_seq




    def scan(self, dynamixel_id_bytes=None):
        """Return the ID sequence of available Dynamixel units.

        :param bytes dynamixel_id_bytes: a sequence of unique ID of the
            Dynamixel units to be pinged.
        """

        available_ids = bytearray()

        if dynamixel_id_bytes is None:
            dynamixel_id_bytes = bytes(range(0xfe)) # bytes in range (0, 0xfd)

        for dynamixel_id in dynamixel_id_bytes:
            if 0 <= dynamixel_id <= 0xfd:
                if self.ping(dynamixel_id):
                    available_ids.append(dynamixel_id)
            else:
                pass # TODO exception

        return available_ids




    # TODO: add the data value table (cf. p. 13)
    # baud_rate = 2000000 / (address_4 + 1)
    def get_baud_rate(self, dynamixel_id):
        """Return the communication speed (baud rate) of the specified
        Dynamixel unit.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.BAUD_RATE, 1)
        raw_value = byte_seq[0]

        baud_rate = 2000000 / (raw_value + 1)

        return round(baud_rate, 1)


    def get_return_delay_time(self, dynamixel_id):
        """Return the return delay time of the specified Dynamixel unit.

        The return delay time is the time it takes (in uSec) for the status
        packet to return after the instruction packet is sent.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.RETURN_DELAY_TIME, 1)
        raw_value = byte_seq[0]

        delay_time = 2 * raw_value

        return delay_time


    def get_cw_angle_limit(self, dynamixel_id):
        """Return the *clockwise angle limit* of the specified Dynamixel unit.

        The goal position should be higher or equal than this value, otherwise
        the *Angle Limit Error Bit* (the second error bit of Status Packets)
        will be set to ``1``.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.CW_ANGLE_LIMIT, 2)
        return utils.little_endian_bytes_to_int(byte_seq)


    def get_ccw_angle_limit(self, dynamixel_id):
        """Return the *counter clockwise angle limit* of the specified
        Dynamixel unit.

        The goal position should be lower or equal than this value, otherwise
        the *Angle Limit Error Bit* (the second error bit of Status Packets)
        will be set to ``1``.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.CCW_ANGLE_LIMIT, 2)
        return utils.little_endian_bytes_to_int(byte_seq)

 

    def get_status_return_level(self, dynamixel_id):
        """Say whether the specified Dynamixel unit is configured to return a
        *Status Packet* after receiving an *Instruction Packet*.

        +----------------+----------------------------------------+
        | Returned value | Meaning                                |
        +================+========================================+
        | 0              | Do not respond to any instructions     |
        +----------------+----------------------------------------+
        | 1              | Respond only to READ_DATA instructions |
        +----------------+----------------------------------------+
        | 2              | Respond to all instructions            |
        +----------------+----------------------------------------+

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.STATUS_RETURN_LEVEL, 1)
        return byte_seq[0]


  

    def has_checksum_alarm_led(self, dynamixel_id):
        """Return ``True`` if the LED of the specified Dynamixel unit is
        configured to blink when an *Checksum Error* occurs.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.ALARM_LED, 1)
        alarm_led_byte = byte_seq[0]

        return bool(alarm_led_byte & (1 << 4))





    def has_checksum_alarm_shutdown(self, dynamixel_id):
        """Return ``True`` if the specified Dynamixel unit is configured to
        turn off its torque when an *Checksum Error* occurs.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.ALARM_SHUTDOWN, 1)
        alarm_shutdown_byte = byte_seq[0]

        return bool(alarm_shutdown_byte & (1 << 4))


    def has_instruction_alarm_shutdown(self, dynamixel_id):
        """Return ``True`` if the specified Dynamixel unit is configured to
        turn off its torque when an *Instruction Error* occurs.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.ALARM_SHUTDOWN, 1)
        alarm_shutdown_byte = byte_seq[0]

        return bool(alarm_shutdown_byte & (1 << 6))






  

    def get_cw_compliance_margin(self, dynamixel_id):
        """Return the clockwise compliance margin of the specified Dynamixel
        unit.

        The compliance feature can be utilized for absorbing shocks at the
        output shaft.

        ::

                 CW
                  ▲
                  │━━━━━━━━━━     goal position
                  │         :╲          :
                  │         : ╲         :
                  │         :  ╲        :
                  │         :   ╲       :
                  │         :    ╲      :
                  │         :     ┃     :
                  │         :     ┃     ▽
            CCW ──┼───────────────┺━━━━━━━━━━━┱─────────────▶ CW
                  │         :     :     :     ┃
                  │         :     :     :     ┃
                  │         :     :     :     :╲
                  │         :     :     :     : ╲
                  │         :     :     :     :  ╲
                  │         :     :     :     :   ╲
                  │         :     :     :     :    ╲▂▂▂▂▂▂▂▂
                  │         :     :     :     :     :
                 CCW         ◀───▶ ◀───▶ ◀───▶ ◀───▶
                               A     B     C     D

            x axis: position error
            y axis: output torque

            A: CCW Compliance Slope
            B: CCW Compliance Margin
            C: CW Compliance Slope
            D: CW Compliance Margin

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.CW_COMPLIENCE_MARGIN, 1)
        return byte_seq[0]


    def get_ccw_compliance_margin(self, dynamixel_id):
        """Return the counter clockwise compliance margin of the specified
        Dynamixel unit.

        The compliance feature can be utilized for absorbing shocks at the
        output shaft.

        ::

                 CW
                  ▲
                  │━━━━━━━━━━     goal position
                  │         :╲          :
                  │         : ╲         :
                  │         :  ╲        :
                  │         :   ╲       :
                  │         :    ╲      :
                  │         :     ┃     :
                  │         :     ┃     ▽
            CCW ──┼───────────────┺━━━━━━━━━━━┱─────────────▶ CW
                  │         :     :     :     ┃
                  │         :     :     :     ┃
                  │         :     :     :     :╲
                  │         :     :     :     : ╲
                  │         :     :     :     :  ╲
                  │         :     :     :     :   ╲
                  │         :     :     :     :    ╲▂▂▂▂▂▂▂▂
                  │         :     :     :     :     :
                 CCW         ◀───▶ ◀───▶ ◀───▶ ◀───▶
                               A     B     C     D

            x axis: position error
            y axis: output torque

            A: CCW Compliance Slope
            B: CCW Compliance Margin
            C: CW Compliance Slope
            D: CW Compliance Margin

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.CCW_COMPLIENCE_MARGIN, 1)
        return byte_seq[0]


    def get_cw_compliance_slope(self, dynamixel_id):
        """Return the clockwise compliance scope of the specified Dynamixel
        unit.

        The compliance feature can be utilized for absorbing shocks at the
        output shaft.

        ::

                 CW
                  ▲
                  │━━━━━━━━━━     goal position
                  │         :╲          :
                  │         : ╲         :
                  │         :  ╲        :
                  │         :   ╲       :
                  │         :    ╲      :
                  │         :     ┃     :
                  │         :     ┃     ▽
            CCW ──┼───────────────┺━━━━━━━━━━━┱─────────────▶ CW
                  │         :     :     :     ┃
                  │         :     :     :     ┃
                  │         :     :     :     :╲
                  │         :     :     :     : ╲
                  │         :     :     :     :  ╲
                  │         :     :     :     :   ╲
                  │         :     :     :     :    ╲▂▂▂▂▂▂▂▂
                  │         :     :     :     :     :
                 CCW         ◀───▶ ◀───▶ ◀───▶ ◀───▶
                               A     B     C     D

            x axis: position error
            y axis: output torque

            A: CCW Compliance Slope
            B: CCW Compliance Margin
            C: CW Compliance Slope
            D: CW Compliance Margin

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.CW_COMPLIENCE_SLOPE, 1)
        return byte_seq[0]


    def get_ccw_compliance_slope(self, dynamixel_id):
        """Return the counter clockwise compliance scope of the specified
        Dynamixel unit.

        The compliance feature can be utilized for absorbing shocks at the
        output shaft.

        ::

                 CW
                  ▲
                  │━━━━━━━━━━     goal position
                  │         :╲          :
                  │         : ╲         :
                  │         :  ╲        :
                  │         :   ╲       :
                  │         :    ╲      :
                  │         :     ┃     :
                  │         :     ┃     ▽
            CCW ──┼───────────────┺━━━━━━━━━━━┱─────────────▶ CW
                  │         :     :     :     ┃
                  │         :     :     :     ┃
                  │         :     :     :     :╲
                  │         :     :     :     : ╲
                  │         :     :     :     :  ╲
                  │         :     :     :     :   ╲
                  │         :     :     :     :    ╲▂▂▂▂▂▂▂▂
                  │         :     :     :     :     :
                 CCW         ◀───▶ ◀───▶ ◀───▶ ◀───▶
                               A     B     C     D

            x axis: position error
            y axis: output torque

            A: CCW Compliance Slope
            B: CCW Compliance Margin
            C: CW Compliance Slope
            D: CW Compliance Margin

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.CCW_COMPLIENCE_SLOPE, 1)
        return byte_seq[0]


    def get_goal_position(self, dynamixel_id):
        """Return the requested goal angular position of the specified
        Dynamixel unit.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.GOAL_POSITION, 2)
        return utils.little_endian_bytes_to_int(byte_seq)


    def get_moving_speed(self, dynamixel_id):
        """Return the angular velocity of the specified Dynamixel unit.

        This angular velocity is defined in range (0, 1023) i.e. (0, 0x3FF) in
        hexadecimal notation. The maximum value (1023 or 0x3FF) corresponds to
        114 RPM (provided that there is enough power supplide).

        Zero is a special value meaning that the largest possible velocity is
        supplied for the configured voltage, e.g. no velocity control is
        applied.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.MOVING_SPEED, 2)
        return utils.little_endian_bytes_to_int(byte_seq)


    def get_torque_limit(self, dynamixel_id):
        """Return the maximum torque output of the specified Dynamixel unit.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.TORQUE_LIMIT, 2)
        return utils.little_endian_bytes_to_int(byte_seq)


    def get_present_position(self, dynamixel_id, degrees=False):
        """Return the current angular position of the specified Dynamixel unit.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        :param bool degrees: defines the returned `position` unit. If `degrees`
            is ``True``, `position` corresponds to the goal rotation angle *in
            degrees* with respect to the original position and is defined in
            range (-150, 150). Otherwise, `position` is a unit free angular
            position to the origin, defined in range (0, 1023) i.e. (0, 0x3FF)
            in hexadecimal notation.
        """
        byte_seq = self.read_data(dynamixel_id, pk.PRESENT_POSITION, 2)
        position = utils.little_endian_bytes_to_int(byte_seq)

        if degrees:
            position = utils.dxl_angle_to_degrees(position)

        return position


    def get_present_speed(self, dynamixel_id):
        """Return the current angular velocity of the specified Dynamixel unit.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.PRESENT_SPEED, 2)
        return utils.little_endian_bytes_to_int(byte_seq)


    # TODO: test this function
    def get_present_load(self, dynamixel_id):
        """Return the magnitude of the load applied to the specified Dynamixel
        unit.

        If the returned value is negative, the load is applied to the clockwise
        direction.

        If the returned value is positive, the load is applied to the counter
        clockwise direction.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = bytearray(self.read_data(dynamixel_id, pk.PRESENT_LOAD, 2))

        load_direction = -1 if (byte_seq[1] & (1 << 2)) == 0 else 1

        byte_seq[1] = 0b00000011 & byte_seq[1]

        abs_load = utils.little_endian_bytes_to_int(byte_seq)
        load = load_direction * abs_load

        return load


    def get_present_voltage(self, dynamixel_id):
        """Return the voltage currently applied to the specified Dynamixel
        unit (in Volts).

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.PRESENT_VOLTAGE, 1)
        raw_value = byte_seq[0]

        voltage = raw_value / 10.
        return voltage


    def get_present_temperature(self, dynamixel_id):
        """Return the internal temperature of the specified Dynamixel unit (in
        Degrees Celsius).

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.PRESENT_TEMPERATURE, 1)
        return byte_seq[0]


    # TODO: stupid ?
    def has_registred_instruction(self, dynamixel_id):
        """Return ``True`` if the specified Dynamixel unit is currently
        processing a REG_WRITE command; otherwise, return ``False``.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.REGISTRED_INSTRUCTION, 1)
        return byte_seq[0] == 1


    def is_moving(self, dynamixel_id):
        """Return ``True`` if the specified Dynamixel unit is moving by its own
        power; return ``False`` otherwise.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.MOVING, 1)
        return byte_seq[0] == 1


    def is_locked(self, dynamixel_id):
        """Return ``True`` if the specified Dynamixel unit is locked; return
        ``False`` otherwise.

        When a Dynamixel unit is locked, only addresses 0x18 to 0x23 can be
        written.
        Once locked, it can only be unlocked by turning the power off.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.LOCK, 1)
        return byte_seq[0] == 1


    def get_punch(self, dynamixel_id):
        """Return the minimum current supplied to the motor of the specified
        Dynamixel unit during operation.

        The initial value is set to 0x20 and its maximum value is 0x3FF.

        ::

                 CW
                  ▲
                  │━━━━━━━━━━     goal position
                  │         :╲          :
                  │         : ╲         :
                  │         :  ╲        :
                  │         :   ╲       :
                  │         :    ╲      :
                  │         :     ┃     :        ▲
                  │         :     ┃     :        │ E
                  │         :     ┃     ▽        ▼
            CCW ──┼───────────────┺━━━━━━━━━━━┱─────────────▶ CW
                  │         :     :     :     ┃  ▲
                  │         :     :     :     ┃  │ E
                  │         :     :     :     ┃  ▼
                  │         :     :     :     :╲
                  │         :     :     :     : ╲
                  │         :     :     :     :  ╲
                  │         :     :     :     :   ╲
                  │         :     :     :     :    ╲▂▂▂▂▂▂▂▂
                  │         :     :     :     :     :
                 CCW         ◀───▶ ◀───▶ ◀───▶ ◀───▶
                               A     B     C     D

            x axis: position error
            y axis: output torque

            A: CCW Compliance Slope
            B: CCW Compliance Margin
            C: CW Compliance Slope
            D: CW Compliance Margin
            E: Punch

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        """
        byte_seq = self.read_data(dynamixel_id, pk.PUNCH, 2)
        return utils.little_endian_bytes_to_int(byte_seq)


    # HIGH LEVEL MUTATORS #####################################################


    def set_id(self, dynamixel_id, new_id):
        """Set the *ID* for the specified Dynamixel unit
        i.e. the unique ID number assigned to actuators for identifying them.

        Different IDs are required for each Dynamixel actuators that are on the
        same network.

        :param int dynamixel_id: the current unique ID of the Dynamixel unit to
            update. It must be in range (0, 0xFE).
        :param int dynamixel_id: the new unique ID assigned to the selected
            Dynamixel unit. It must be in range (0, 0xFE).
        """
        # TODO: check ranges

        self.write_data(dynamixel_id, pk.ID, new_id)


    def set_baud_rate(self, dynamixel_id, baudrate, unit="kbps"):
        """Set the *baud rate* for the specified Dynamixel unit
        i.e. set the connection speed with the actuator.

        If `unit` is `"internal"` or `"raw"` then the actual speed `bps` (in
        bauds per second) is:

            bps = 2000000 / (`baudrate` + 1)

        If `unit` is `"bps"` then the actual speed `bps` (in bauds per second)
        is:

            bps = `baudrate`

        If `unit` is `"kbps"` then the actual speed `bps` (in bauds per second)
        is:

            bps = `baudrate` * 1000

        E.g. the following arguments will give the same result (200kbps)::

            set_baud_rate(1, baudrate=9, unit="internal")
            set_baud_rate(1, baudrate=200000, unit="bps")

        :param int dynamixel_id: the current unique ID of the Dynamixel unit to
            update. It must be in range (0, 0xFE).
        :param int dynamixel_id: the new baud rate assigned to the selected
            Dynamixel unit. It must be in range (1, 0xFF).
        :param bool  unit: define the units of the `baudrate`
            argument.
        """

        if unit == "bps":
            baudrate = int(round(2000000. / baudrate)) - 1
        elif unit == "kbps":
            baudrate = int(round(2000. / baudrate)) - 1

        # TODO: check ranges

        self.write_data(dynamixel_id, pk.BAUD_RATE, baudrate)


    def set_return_delay_time(self, dynamixel_id, return_delay_time, unit="us"):
        """Set the *return delay time* for the specified Dynamixel unit
        i.e. the time for the status packets to return after the instruction
        packet is sent.

        If `unit` is `"internal"` or `"raw"` then the actual return delay time
        `rdt` (in µs) will be::

            rdt = 2 * return_delay_time

        If `unit` is `"us"`, `"usec"` or `"microseconds"` then the actual
        return delay time `rdt` (in µs) will be::

            rdt = return_delay_time

        E.g. the following arguments will give the same result (return delay
        time = 500µs)::

            set_return_delay_time(1, return_delay_time=250, unit="internal")
            set_return_delay_time(1, return_delay_time=500, unit="us")

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFE).
        :param int  return_delay_time: the new return delay time. It must be in
            range (0, 255) i.e. (0, 0xFF) in hexadecimal notation.
        :param bool  unit: define the units of the `return_delay_time`
            argument.
        """

        if unit in ("us", "usec", "microseconds"):
            return_delay_time = int(round(return_delay_time / 2.))

        # TODO: check ranges

        self.write_data(dynamixel_id, pk.RETURN_DELAY_TIME, return_delay_time)


    def set_cw_angle_limit(self, dynamixel_id, angle_limit, degrees=False):
        """Set the *clockwise angle limit* of the specified Dynamixel unit to
        the specified `angle_limit`.

        The *goal position* should be higher or equal than this value, otherwise
        the *Angle Limit Error Bit* (the second error bit of Status Packets)
        will be set to ``1``.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        :param int angle_limit: the *clockwise angle limit* to be set for the
            specified Dynamixel unit. If `degrees` is ``True``, this value is
            defined in degrees and must be in range (-150, 150); otherwise, it
            is an unit free angle and must be in range (0, 1023) i.e.  (0,
            0x3FF) in hexadecimal notation.
        :param bool degrees: defines the `angle_limit` unit. If `degrees` is
            ``True``, `angle_limit` is defined *in degrees* and must be in
            range (-150, 150). Otherwise, `angle_limit` is a unit free angular
            limit, defined in range (0, 1023) i.e. (0, 0x3FF) in hexadecimal
            notation.
        """
        # TODO: check ranges

        if degrees:
            angle_limit = utils.degrees_to_dxl_angle(angle_limit)

        params = utils.int_to_little_endian_bytes(angle_limit)

        self.write_data(dynamixel_id, pk.CW_ANGLE_LIMIT, params)


    def set_ccw_angle_limit(self, dynamixel_id, angle_limit, degrees=False):
        """Set the *counter clockwise angle limit* of the specified
        Dynamixel unit to the specified `angle_limit`.

        The goal position should be lower or equal than this value, otherwise
        the *Angle Limit Error Bit* (the second error bit of Status Packets)
        will be set to ``1``.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFD).
        :param int angle_limit: the *counter clockwise angle limit* to be set
            for the specified Dynamixel unit. If `degrees` is ``True``, this
            value is defined in degrees and must be in range (-150, 150);
            otherwise, it is an unit free angle and must be in range (0, 1023)
            i.e.  (0, 0x3FF) in hexadecimal notation.
        :param bool degrees: defines the `angle_limit` unit. If `degrees` is
            ``True``, `angle_limit` is defined *in degrees* and must be in
            range (-150, 150). Otherwise, `angle_limit` is a unit free angular
            limit, defined in range (0, 1023) i.e. (0, 0x3FF) in hexadecimal
            notation.
        """
        # TODO: check ranges

        if degrees:
            angle_limit = utils.degrees_to_dxl_angle(angle_limit)

        params = utils.int_to_little_endian_bytes(angle_limit)

        self.write_data(dynamixel_id, pk.CCW_ANGLE_LIMIT, params)


    def set_speed(self, dynamixel_id, speed):
        """Set the *moving speed* for the specified Dynamixel unit.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFE).
        :param int speed: the new moving speed. It must be in range (0, 1023)
            i.e. (0, 0x3FF) in hexadecimal notation.
        """
        # TODO: check ranges

        params = utils.int_to_little_endian_bytes(speed)

        self.write_data(dynamixel_id, 0x20, params)


    def goto(self, dynamixel_id, position, speed=None, degrees=False):
        """Set the *goal position* and *moving speed* for the specified
        Dynamixel unit.

        :param int dynamixel_id: the unique ID of a Dynamixel unit. It must be
            in range (0, 0xFE).
        :param int position: the new goal position. If `degrees` is ``True``,
            `position` corresponds to the goal rotation angle *in degrees* with
            respect to the original position and must be in range (-150, 150).
            Otherwise, `position` is a unit free rotation angle to the origin,
            defined in range (0, 1023) i.e. (0, 0x3FF) in hexadecimal notation.
        :param int speed: the new moving speed. It must be in range (0, 1023)
            i.e. (0, 0x3FF) in hexadecimal notation. This parameter is
            optional; if `speed` is not specified, the *moving speed* present
            in the Dynamixel control table is kept and used to reach the goal
            position.
        :param bool degrees: defines the `position` unit. If `degrees` is
            ``True``, `position` corresponds to the goal rotation angle *in
            degrees* with respect to the original position and must be in range
            (-150, 150). Otherwise, `position` is a unit free angular position,
            defined in range (0, 1023) i.e. (0, 0x3FF) in hexadecimal notation.
        """
        # TODO: check ranges

        if degrees:
            position = utils.degrees_to_dxl_angle(position)

        params = utils.int_to_little_endian_bytes(position)

        if speed is not None:
            params += utils.int_to_little_endian_bytes(speed)

        self.write_data(dynamixel_id, pk.GOAL_POSITION, params)

