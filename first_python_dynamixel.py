import serial
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
MDroite = 17
MGauche = 18
AX_PING = 1
AX_READ_DATA = 2
AX_WRITE_DATA = 3
AX_REG_WRITE = 4
AX_ACTION = 5
AX_RESET = 6
AX_SPEED_LENGTH = 5
AX_SYNC_WRITE = 131
AX_BROADCAST_ID = 254

def Connection():
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

def avancer(vitesse):
    GPIO.output(18, GPIO.HIGH)
    s = [vitesse&0xff, vitesse>>8]
    checksum = (~(id + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_PING + s[0] + s[1]))&0xff
    packetData = chr(255) #FF
    packetData += chr(255) #FF
    packetData += chr(AX_BROADCAST_ID) #ID, broadcasting ID
    packetData += chr(AX_SPEED_LENGTH) #Set speed length
    packetData += chr(AX_WRITE_DATA) #Instruction set, write data
    packetData += chr(s[0])
    packetData += chr(s[1])
    packetData += chr(checksum)
    ser.write(packetData)
    GPIO.output(18, GPIO.LOW)
    
def reculler(vitesse):
    GPIO.output(18, GPIO.HIGH)
    s = [vitesse&0xff, vitesse>>8]
    checksum = (~(id + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_PING + s[0] + s[1]))&0xff
    packetData = chr(255) #FF
    packetData += chr(255) #FF
    packetData += chr(AX_BROADCAST_ID) #ID, broadcasting ID
    packetData += chr(AX_SPEED_LENGTH) #Set speed length
    packetData += chr(AX_WRITE_DATA) #Instruction set, write data
    packetData += chr(s[0])
    packetData += chr(s[1])
    packetData += chr(checksum)
    ser.write(packetData)
    GPIO.output(18, GPIO.LOW)
    
def Droite(vitesse):
    GPIO.output(18, GPIO.HIGH)
    s = [vitesse&0xff, vitesse>>8]
    checksum = (~(id + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_PING + s[0] + s[1]))&0xff
    packetData = chr(255) #FF
    packetData += chr(255) #FF
    packetData += chr(MGauche) #ID moteur gauche
    packetData += chr(AX_SPEED_LENGTH) #Set speed length
    packetData += chr(AX_WRITE_DATA) #Instruction set, write data
    packetData += chr(s[0])
    packetData += chr(s[1])
    packetData += chr(checksum)
    ser.write(packetData)
    GPIO.output(18, GPIO.LOW)
    
def Gauche(vitesse):
    GPIO.output(18, GPIO.HIGH)
    s = [vitesse&0xff, vitesse>>8]
    checksum = (~(id + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_PING + s[0] + s[1]))&0xff
    packetData = chr(255) #FF
    packetData += chr(255) #FF
    packetData += chr(MDroite) #ID mopteur droite
    packetData += chr(AX_SPEED_LENGTH) #Set speed length
    packetData += chr(AX_WRITE_DATA) #Instruction set, write data
    packetData += chr(s[0])
    packetData += chr(s[1])
    packetData += chr(checksum)
    ser.write(packetData)
    GPIO.output(18, GPIO.LOW)
 
def main():
    while True:
            # GPIO.output(18, GPIO.HIGH)
            # #ser.write(bytearray.fromhex("FF FF 01 05 03 1E 32 03 A3"))
            # #ser.write(bytearray.fromhex("FF FF 14 05 03 1E 32 03 90"))#c5
            # ser.write(bytearray.fromhex("FF FF FE 05 03 1E 32 03 A6"))
            # time.sleep(0.1)
            # #time.sleep(0.1)
            # GPIO.output(18, GPIO.LOW)
            # time.sleep(3)

            # GPIO.output(18,GPIO.HIGH)
            # #ser.write(bytearray.fromhex("FF FF 01 05 03 1E CD 00 0b"))
            # #ser.write(bytearray.fromhex("FF FF 14 05 03 1E CD 00 F8"))
            # ser.write(bytearray.fromhex("FF FF FE 05 03 1E CD 00 0E"))
            # time.sleep(0.1)
            
            # #time.sleep(0.1)
            # GPIO.output(18,GPIO.LOW)
            # time.sleep(3)
            
            avancer(200)
        
if __name__ == '__main__':
        Connection()
        Main()