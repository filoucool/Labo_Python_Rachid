
from pyax12.connection import Connection
from pyax12.argparse_default import common_argument_parser
import pyax12.packet as pk
import RPi.GPIO as gpio

import time

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
    #args.dynamixel_id = all motors
    # Set the "wheel mode"
    serial_connection.set_cw_angle_limit(dynamixel_id1, 0, degrees=False)
    serial_connection.set_ccw_angle_limit(dynamixel_id1, 0, degrees=False)
    serial_connection.set_cw_angle_limit(dynamixel_id2, 0, degrees=False)
    serial_connection.set_ccw_angle_limit(dynamixel_id2, 0, degrees=False)
    # Activate the actuator (speed=512)
    serial_connection.set_speed(dynamixel_id1, 1023)
    serial_connection.set_speed(dynamixel_id2, 2046)
    # Lets the actuator turn 5 seconds
    time.sleep(2)
    # Stop the actuator (speed=0)
    serial_connection.set_speed(dynamixel_id1, 0)
    serial_connection.set_speed(dynamixel_id2, 0)
    # Leave the "wheel mode"
    serial_connection.set_ccw_angle_limit(dynamixel_id1, 1023, degrees=False)
    serial_connection.set_ccw_angle_limit(dynamixel_id2, 2047, degrees=False)
    
    # Close the serial connection
    serial_connection.close()


if __name__ == '__main__':
    gpio.cleanup()
    gpio.setwarnings(False)
    main()
    gpio.cleanup()