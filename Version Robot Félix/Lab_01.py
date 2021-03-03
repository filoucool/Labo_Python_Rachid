from pyax12.connection import Connection
from pyax12.argparse_default import common_argument_parser

import time

"""
Dans ce lab, vous allez comprendre la base du fonctionnement des moteur AX-12.
Ce code initialise les moteurs au premier step (0/300) et ensuite fait avancer et reculer le robot.

Il faut tenir compte qu'un des moteurs est à l'envers donc il tournera CCW pendant que l'autre tourne CW. On peut 
contrer cet effet une mettant un - devant

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


serial_connection.goto(dynamixel_id_mt1, 0, speed=512, degrees=True)
serial_connection.goto(dynamixel_id_mt2, 0, speed=512, degrees=True)
time.sleep(1)

serial_connection.goto(dynamixel_id_mt2, -300, speed=512, degrees=True)
serial_connection.goto(dynamixel_id_mt2, 300, speed=512, degrees=True)
time.sleep(1)


serial_connection.close()

if __name__ == '__main__':
    main()