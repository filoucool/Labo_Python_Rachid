

from pyax12.connection import Connection
from pyax12.argparse_default import common_argument_parser
import pyax12.packet as pk

DEFAULT_VALUE = 1000

def main():
    """Set the *baud rate* for the specified Dynamixel unit
    i.e. set the connection speed with the actuator in kbps
    (kilo bauds per second).
    """

    # Parse options
    parser = common_argument_parser(desc=main.__doc__)

    parser.add_argument("--new-baudrate",
                        "-n",
                        help="the new baud rate assigned to the selected "
                             "Dynamixel unit (in kbps)."
                             "The default value is {default}kbps.".format(default=DEFAULT_VALUE),
                        type=float,
                        metavar="FLOAT",
                        default=1000)

    args = parser.parse_args()

    # Connect to the serial port
    serial_connection = Connection(port='/dev/ttyS0',
                                   baudrate=1000000,
                                   timeout=0.01,
                                   rpi_gpio=18)

    serial_connection.set_baud_rate(args.dynamixel_id, args.new_baudrate)

    # Close the serial connection
    serial_connection.close()

if __name__ == '__main__':
    main()
