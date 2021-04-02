from time import sleep
from serial import Serial
import RPi.GPIO as GPIO
from AX12LIB import Ax12
ax12_o = Ax12()

def main():
    ax12_o.moveSpeed(0x11, 90, 500)
    ax12_o.moveSpeed(0x12, 120, 500)  
    
if __name__ == '__main__':

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.OUT)
    gpioSet = True
    main()