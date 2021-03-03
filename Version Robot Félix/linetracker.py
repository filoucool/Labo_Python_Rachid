#!/usr/bin/python
from pyax12.connection import Connection
from pyax12.argparse_default import common_argument_parser
import pyax12.packet as pk
import RPi.GPIO as gpio
from RPi import GPIO
from time import sleep

def main():
    # Parse options
    parser = common_argument_parser(desc=main.__doc__)
    args = parser.parse_args()
    # Connect to the serial port
    serial_connection = Connection(port='/dev/ttyS0',
                                   baudrate=1000000,
                                   timeout=0.01,
                                   rpi_gpio=18)

dynamixel_id1 = 0x12
dynamixel_id2 = 0x11

GPIO.setmode(GPIO.BCM)

Sensor_1 = 11
Sensor_2 = 11
Sensor_3 = 11
Sensor_4 = 11
Sensor_5 = 11

GPIO.setup(Sensor_1, GPIO.IN)
GPIO.setup(Sensor_2, GPIO.IN)
GPIO.setup(Sensor_3, GPIO.IN)
GPIO.setup(Sensor_4, GPIO.IN)
GPIO.setup(Sensor_5, GPIO.IN)

serial_connection.set_cw_angle_limit(dynamixel_id1, 0, degrees=False)
serial_connection.set_ccw_angle_limit(dynamixel_id1, 0, degrees=False)
serial_connection.set_cw_angle_limit(dynamixel_id2, 0, degrees=False)
serial_connection.set_ccw_angle_limit(dynamixel_id2, 0, degrees=False)

#sensor 2 is Left of line so should be 0
#sensor 4 is right of line so should be 0
#sensor 3 is on the line so should be 1

try:
	while True:
        s;
        if GPIO.input(Sensor_2) == 1:
            #go right a bit
            serial_connection.set_speed(dynamixel_id1, 1023)
            serial_connection.set_speed(dynamixel_id2, 2046)
		elif GPIO.input(Sensor_4) == 1:
            #go to left a bit
		elif GPIO.input(Sensor_3) == 1
            #continue going forward
        time.sleep(0.1)
except:
	GPIO.cleanup()
