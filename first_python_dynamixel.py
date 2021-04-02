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
AX_GOAL_SP_LENGTH = 5
AX_SYNC_WRITE = 131
AX_BROADCAST_ID = 254
AX_GOAL_POSITION_L = 30
RPI_DIRECTION_TX = GPIO.HIGH
RPI_DIRECTION_RX = GPIO.LOW
RPI_DIRECTION_PIN = 18
RPI_DIRECTION_SWITCH_DELAY = 0.0001
TX_DELAY_TIME = 0.00002
gpioSet = False
AX_START = 255

 
def Main():
    global gpioSet
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

    def direction(d):
        GPIO.output(RPI_DIRECTION_PIN, d)
        time.sleep(RPI_DIRECTION_SWITCH_DELAY)
        
    def moveSpeed(id, position, speed):
        direction(RPI_DIRECTION_TX)
        ser.flushInput()
        p = [position&0xff, position>>8]
        s = [speed&0xff, speed>>8]
        checksum = (~(id + AX_GOAL_SP_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + p[0] + p[1] + s[0] + s[1]))&0xff
        outData = bytes(AX_START)
        outData += bytes(AX_START)
        outData += bytes(id)
        outData += bytes(AX_GOAL_SP_LENGTH)
        outData += bytes(AX_WRITE_DATA)
        outData += bytes(AX_GOAL_POSITION_L)
        outData += bytes(p[0])
        outData += bytes(p[1])
        outData += bytes(s[0])
        outData += bytes(s[1])
        outData += bytes(checksum)
        ser.write(outData)
        time.sleep(TX_DELAY_TIME)
        
        ser.write(bytearray.fromhex("FF FF FE 05 03 1E 32 03 A6"))

    
    def Reculler(vitesse):
        GPIO.output(18, GPIO.HIGH)
        s = [vitesse&0xff, vitesse>>8]
        checksum = (~(AX_BROADCAST_ID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_PING + s[0] + s[1]))&0xff
        packetData = bytes(255) #FF
        packetData += bytes(255) #FF
        packetData += bytes(AX_BROADCAST_ID) #ID, broadcasting ID
        packetData += bytes(AX_SPEED_LENGTH) #Set speed length
        packetData += bytes(AX_WRITE_DATA) #Instruction set, write data
        packetData += bytes(s[0])
        packetData += bytes(s[1])
        packetData += bytes(checksum)
        ser.write(packetData)
        GPIO.output(18, GPIO.LOW)
        print("Reculler")
        
    def Droite(vitesse):
        GPIO.output(18, GPIO.HIGH)
        s = [vitesse&0xff, vitesse>>8]
        checksum = (~(MGauche + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_PING + s[0] + s[1]))&0xff
        packetData = bytes(255) #FF
        packetData += bytes(255) #FF
        packetData += bytes(MGauche) #ID moteur gauche
        packetData += bytes(AX_SPEED_LENGTH) #Set speed length
        packetData += bytes(AX_WRITE_DATA) #Instruction set, write data
        packetData += bytes(s[0])
        packetData += bytes(s[1])
        packetData += bytes(checksum)
        ser.write(packetData)
        GPIO.output(18, GPIO.LOW)
        print("Droite")
        
    def Gauche(vitesse):
        GPIO.output(18, GPIO.HIGH)
        s = [vitesse&0xff, vitesse>>8]
        checksum = (~(MDroite + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_PING + s[0] + s[1]))&0xff
        packetData = bytes(255) #FF
        packetData += bytes(255) #FF
        packetData += bytes(MDroite) #ID mopteur droite
        packetData += bytes(AX_SPEED_LENGTH) #Set speed length
        packetData += bytes(AX_WRITE_DATA) #Instruction set, write data
        packetData += bytes(s[0])
        packetData += bytes(s[1])
        packetData += bytes(checksum)
        ser.write(packetData)
        GPIO.output(18, GPIO.LOW)
        print("Gauche")
        
    if(gpioSet ==  False):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RPI_DIRECTION_PIN, GPIO.OUT)
        gpioSet = True
    direction(RPI_DIRECTION_RX)  
        
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
            
#             Avancer(200)
            moveSpeed(254, 30, 100)
            time.sleep(1)
            
        
if __name__ == '__main__':
        Main()