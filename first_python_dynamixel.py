
import serial
import time
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
print('test')

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

#port = serial.Serial("/dev/ttyAMA0", baudrate=1000000, timeout=3.0)
ser = serial.Serial(                                         
    port='/dev/ttyS0',
    #port='/dev/ttyAMA0',
    baudrate = 1000000,
    #baudrate = 57600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.1
)

ID = 0x03
BAUD_RATE = 0x04
RETURN_DELAY_TIME = 0x05
CW_ANGLE_LIMIT = 0x06
CCW_ANGLE_LIMIT = 0x08

def avancer(vitesse):
    GPIO.output(18, GPIO.HIGH)
    
    packetData = chr(255)
    packetData += chr(255)
    packetData += chr(254)
    packetData += chr(7)
    packetData += chr()
    packetData += chr()
    packetData += chr(checksum)
    GPIO.output(18, GPIO.LOW)
    



while True:
        GPIO.output(18, GPIO.HIGH)
        #ser.write(bytearray.fromhex("FF FF 01 05 03 1E 32 03 A3"))
        #ser.write(bytearray.fromhex("FF FF 14 05 03 1E 32 03 90"))#c5
        ser.write(bytearray.fromhex("FF FF FE 05 03 1E 32 03 A6"))
        time.sleep(0.1)
        #time.sleep(0.1)
        GPIO.output(18, GPIO.LOW)
        time.sleep(3)

        GPIO.output(18,GPIO.HIGH)
        #ser.write(bytearray.fromhex("FF FF 01 05 03 1E CD 00 0b"))
        #ser.write(bytearray.fromhex("FF FF 14 05 03 1E CD 00 F8"))
        ser.write(bytearray.fromhex("FF FF FE 05 03 1E CD 00 0E"))
        time.sleep(0.1)
        
        #time.sleep(0.1)
        GPIO.output(18,GPIO.LOW)
        time.sleep(3)
        
GPIO.cleanup()        