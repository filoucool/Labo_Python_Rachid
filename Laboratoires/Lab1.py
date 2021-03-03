import Ax12
from serial import Serial
from serial import SerialException
import sys
import time
import math
import argparse

def main():

    # Parse options
    

    # Connect to the serial port
    serial_connection =(port="/dev/ttyS0",baudrate=1000000,timeout=0.1,
             waiting_time=0.02,rpi_gpio=True)

    dynamixel_id = 18

    # Set the "wheel mode"
    serial_connection.set_cw_angle_limit(dynamixel_id, 0, degrees=False)
    serial_connection.set_ccw_angle_limit(dynamixel_id, 0, degrees=False)

    # Activate the actuator (speed=512)
    serial_connection.set_speed(dynamixel_id, 512)

    # Lets the actuator turn 5 seconds
    time.sleep(5)

    # Stop the actuator (speed=0)
    serial_connection.set_speed(dynamixel_id, 0)

    # Leave the "wheel mode"
    serial_connection.set_ccw_angle_limit(dynamixel_id, 1023, degrees=False)

    # Go to the initial position (0 degree)
    serial_connection.goto(dynamixel_id, 0, speed=512, degrees=True)

    # Close the serial connection
    serial_connection.close()

if __name__ == '__main__':
    main()
