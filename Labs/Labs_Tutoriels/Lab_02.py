from pyax12.connection import Connection
from pyax12.argparse_default import common_argument_parser

import time

"""
Dans ce lab, vous allez comprendre la base du fonctionnement des moteur AX-12.
Ce code initialise les moteurs au premier step (0/300) et va ensuite tourner pendant 5 secondes.

"""

def main():
   

    # Parsing
    parser = common_argument_parser(desc=main.__doc__)
    args = parser.parse_args()

    # Serial Connection
    serial_connection = Connection(port=args.port,
                                   baudrate=args.baudrate,
                                   timeout=args.timeout,
                                   rpi_gpio=args.rpi)

dynamixel_id_mt1 = 18
dynamixel_id_mt2 = 17

serial_connection.set_cw_angle_limit(dynamixel_id_mt1, 0, degrees=False)
serial_connection.set_ccw_angle_limit(dynamixel_id_mt2, 0, degrees=False)

serial_connection.set_speed(dynamixel_id_mt1, 512)
serial_connection.set_speed(dynamixel_id_mt2, 512)

time.sleep(5)

serial_connection.set_speed(dynamixel_id_mt1, 0)
serial_connection.set_speed(dynamixel_id_mt2, 0)

serial_connection.set_ccw_angle_limit(dynamixel_id_mt1, 1023, degrees=False)
serial_connection.set_ccw_angle_limit(dynamixel_id_mt2, 1023, degrees=False)


serial_connection.close()

if __name__ == '__main__':
    main()