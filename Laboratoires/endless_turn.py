from connection import Connection
from argparse_default import common_argument_parser
import packet as pk
import RPi.GPIO as gpio
import time

def main():
    parser = common_argument_parser(desc=main.__doc__)
    args = parser.parse_args()
    serial_connection = Connection(port='/dev/ttyS0',
                                   baudrate=1000000,
                                   timeout=0.01,
                                   rpi_gpio=18)

    dynamixel_id1 = 0x11
    dynamixel_id2 = 0x12
    
    while 1:
        serial_connection.set_speed(dynamixel_id1, 0)
        time.sleep(1)


#     serial_connection.set_speed(dynamixel_id1, 0)
    serial_connection.set_ccw_angle_limit(dynamixel_id2, 1023, degrees=False)
    serial_connection.set_ccw_angle_limit(dynamixel_id2, 1023, degrees=False)
    serial_connection.set_speed(dynamixel_id2, 200)

    gpio.cleanup()
    serial_connection.close()

if __name__ == '__main__':
    gpio.cleanup()
    gpio.setwarnings(False)
    main()
    gpio.cleanup()