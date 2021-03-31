from time import sleep
from serial import Serial
import RPi.GPIO as GPIO
import AX12LIB


def main():
    TEST(0x12, 10, 500)
    moveSpeed(0x11, 90, 500)
    moveSpeed(0x12, 120, 500)  
    
if __name__ == '__main__':
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.OUT)
    gpioSet = True
    main()