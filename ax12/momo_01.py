import serial
import time
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)




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

AX_GOAL_SP_LENGTH  = 0x05
AX_WRITE_DATA      = 0x03
AX_GOAL_POSITION_L = 0x1E#30
AX_START           = 0xFF

def moveSpeed(id, position, speed):    
        p = [position&0xff, position>>8]
        s = [speed&0xff, speed>>8]
        checksum = (~(id + AX_GOAL_SP_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + p[0] + p[1] + s[0] + s[1]))&0xff
        outData = chr(AX_START)
        outData += chr(AX_START)
        outData += chr(id)
        outData += chr(AX_GOAL_SP_LENGTH)
        outData += chr(AX_WRITE_DATA)
        outData += chr(AX_GOAL_POSITION_L)
        outData += chr(p[0])
        outData += chr(p[1])
        outData += chr(s[0])
        outData += chr(s[1])
        outData += chr(checksum)
        ser.write(outData)
        
        #ser.write(bytearray.str(outData))
        #byte_array = bytearray.fromhex((outData))
        #print(byte_array)
        #bytes.fromhex(hex_string)
        #ser.write("lola".encode())
        #ser.write(AX_START.encode())
        
        #outData = AX_START.encode()
        #ser.write(outData)
        #ser.write(byte_array)  
#ser.write(bytearray.fromhex("FF FF 01 05 03 1E 32 03 A3"))
#ser.write(bytearray.fromhex("FF FF 14 05 03 1E 32 03 90"))#c5
ser.flushInput()
var=1
while (var==1):
    global gpioSet
    GPIO.output(18, GPIO.HIGH)
    #ser.write(bytearray.fromhex("FF FF FE 05 03 1E 32 03 A6"))
    moveSpeed(8, 0x10, 0xf0)  
    time.sleep(0.0001)
    GPIO.output(18, GPIO.LOW)
    time.sleep(0.1)
    lecture = ser.read()
    print(lecture)

    GPIO.output(18, GPIO.HIGH)
    #ser.write(bytearray.fromhex("FF FF FE 05 03 1E 32 03 A6"))
    moveSpeed(8, 0xfa, 0xf0)  
    time.sleep(0.0001)
    GPIO.output(18, GPIO.LOW)
    time.sleep(0.1)
    lecture = ser.read()
    print(lecture)
    var=0

  

#GPIO.output(18,GPIO.HIGH)
#ser.write(bytearray.fromhex("FF FF 01 05 03 1E CD 00 0b"))
#ser.write(bytearray.fromhex("FF FF 14 05 03 1E CD 00 F8"))
#ser.write(bytearray.fromhex("FF FF FE 05 03 1E CD 00 0E"))
#time.sleep(0.1)

#time.sleep(0.1)
#GPIO.output(18,GPIO.LOW)
#time.sleep(3)
