from time import sleep
from serial import Serial
import RPi.GPIO as GPIO

if 1 == 1:
    AX_MODEL_NUMBER_L = 0
    AX_MODEL_NUMBER_H = 1
    AX_VERSION = 2
    AX_ID = 3
    AX_BAUD_RATE = 4
    AX_RETURN_DELAY_TIME = 5
    AX_CW_ANGLE_LIMIT_L = 6
    AX_CW_ANGLE_LIMIT_H = 7
    AX_CCW_ANGLE_LIMIT_L = 8
    AX_CCW_ANGLE_LIMIT_H = 9
    AX_SYSTEM_DATA2 = 10
    AX_LIMIT_TEMPERATURE = 11
    AX_DOWN_LIMIT_VOLTAGE = 12
    AX_UP_LIMIT_VOLTAGE = 13
    AX_MAX_TORQUE_L = 14
    AX_MAX_TORQUE_H = 15
    AX_RETURN_LEVEL = 16
    AX_ALARM_LED = 17
    AX_ALARM_SHUTDOWN = 18
    AX_OPERATING_MODE = 19
    AX_DOWN_CALIBRATION_L = 20
    AX_DOWN_CALIBRATION_H = 21
    AX_UP_CALIBRATION_L = 22
    AX_UP_CALIBRATION_H = 23

    # ////////////////////////////////////////////////////////////// RAM AREA
    AX_TORQUE_STATUS = 24
    AX_LED_STATUS = 25
    AX_CW_COMPLIANCE_MARGIN = 26
    AX_CCW_COMPLIANCE_MARGIN = 27
    AX_CW_COMPLIANCE_SLOPE = 28
    AX_CCW_COMPLIANCE_SLOPE = 29
    AX_GOAL_POSITION_L = 30
    AX_GOAL_POSITION_H = 31
    AX_GOAL_SPEED_L = 32
    AX_GOAL_SPEED_H = 33
    AX_TORQUE_LIMIT_L = 34
    AX_TORQUE_LIMIT_H = 35
    AX_PRESENT_POSITION_L = 36
    AX_PRESENT_POSITION_H = 37
    AX_PRESENT_SPEED_L = 38
    AX_PRESENT_SPEED_H = 39
    AX_PRESENT_LOAD_L = 40
    AX_PRESENT_LOAD_H = 41
    AX_PRESENT_VOLTAGE = 42
    AX_PRESENT_TEMPERATURE = 43
    AX_REGISTERED_INSTRUCTION = 44
    AX_PAUSE_TIME = 45
    AX_MOVING = 46
    AX_LOCK = 47
    AX_PUNCH_L = 48
    AX_PUNCH_H = 49
    AX_RETURN_NONE = 0
    AX_RETURN_READ = 1
    AX_RETURN_ALL = 2
    AX_PING = 1
    AX_READ_DATA = 2
    AX_WRITE_DATA = 3
    AX_REG_WRITE = 4
    AX_ACTION = 5
    AX_RESET = 6
    AX_SYNC_WRITE = 131
    AX_RESET_LENGTH = 2
    AX_ACTION_LENGTH = 2
    AX_ID_LENGTH = 4
    AX_LR_LENGTH = 4
    AX_SRL_LENGTH = 4
    AX_RDT_LENGTH = 4
    AX_LEDALARM_LENGTH = 4
    AX_SHUTDOWNALARM_LENGTH = 4
    AX_TL_LENGTH = 4
    AX_VL_LENGTH = 6
    AX_AL_LENGTH = 7
    AX_CM_LENGTH = 6
    AX_CS_LENGTH = 5
    AX_COMPLIANCE_LENGTH = 7
    AX_CCW_CW_LENGTH = 8
    AX_BD_LENGTH = 4
    AX_TEM_LENGTH = 4
    AX_MOVING_LENGTH = 4
    AX_RWS_LENGTH = 4
    AX_VOLT_LENGTH = 4
    AX_LOAD_LENGTH = 4
    AX_LED_LENGTH = 4
    AX_TORQUE_LENGTH = 4
    AX_POS_LENGTH = 4
    AX_GOAL_LENGTH = 5
    AX_MT_LENGTH = 5
    AX_PUNCH_LENGTH = 5
    AX_SPEED_LENGTH = 5
    AX_GOAL_SP_LENGTH = 7
    AX_BYTE_READ = 1
    AX_INT_READ = 2
    AX_ACTION_CHECKSUM = 250
    AX_BROADCAST_ID = 254
    AX_START = 255
    AX_CCW_AL_L = 255
    AX_CCW_AL_H = 3
    AX_LOCK_VALUE = 1
    LEFT = 0
    RIGTH = 1
    RX_TIME_OUT = 10
    TX_DELAY_TIME = 0.00002
    RPI_DIRECTION_PIN = 18
    RPI_DIRECTION_TX = GPIO.HIGH
    RPI_DIRECTION_RX = GPIO.LOW
    RPI_DIRECTION_SWITCH_DELAY = 0.0001

    

        
    connectedServos = []

    # Error lookup dictionary for bit masking
    dictErrors = {  1 : "Input Voltage",
            2 : "Angle Limit",
            4 : "Overheating",
            8 : "Range",
            16 : "Checksum",
            32 : "Overload",
            64 : "Instruction"
            }

    # Custom error class to report AX servo errors

    # Servo timeout

    def direction(d):
        GPIO.output(RPI_DIRECTION_PIN, d)
        sleep(RPI_DIRECTION_SWITCH_DELAY)

    def readData(id):
        direction(RPI_DIRECTION_RX)
        reply = port.read(5) # [0xff, 0xff, origin, length, error]
        try:
            assert ord(reply[0]) == 0xFF
        except:
            e = "Timeout on servo " + str(id)
            raise timeoutError(e)

        try :
            length = ord(reply[3]) - 2
            error = ord(reply[4])

            if(error != 0):
                print ("Error from servo: " + dictErrors[error] + ' (code  ' + hex(error) + ')')
                return -error
            # just reading error bit
            elif(length == 0):
                return error
            else:
                if(length > 1):
                    reply = port.read(2)
                    returnValue = (ord(reply[1])<<8) + (ord(reply[0])<<0)
                else:
                    reply = port.read(1)
                    returnValue = ord(reply[0])
                return returnValue
        except detail:
            raise axError(detail)

    def ping(id):
        direction(RPI_DIRECTION_TX)
        port.flushInput()
        checksum = (~(id + AX_READ_DATA + AX_PING))&0xff
        outData = chr(AX_START)
        outData += chr(AX_START)
        outData += chr(id)
        outData += chr(AX_READ_DATA)
        outData += chr(AX_PING)
        outData += chr(checksum)
        port.write(outData)
        sleep(TX_DELAY_TIME)
        return readData(id)

    def factoryReset(id, confirm = False):
        if(confirm):
            direction(RPI_DIRECTION_TX)
            port.flushInput()
            checksum = (~(id + AX_RESET_LENGTH + AX_RESET))&0xff
            outData = chr(AX_START)
            outData += chr(AX_START)
            outData += chr(id)
            outData += chr(AX_RESET_LENGTH)
            outData += chr(AX_RESET)
            outData += chr(checksum)
            port.write(outData)
            sleep(TX_DELAY_TIME)
            return readData(id)
        else:
            print ("nothing done, please send confirm = True as this fuction reset to the factory default value, i.e reset the motor ID")
            return

    def move(id, position):
        direction(RPI_DIRECTION_TX)
        port.flushInput()
        p = [position&0xff, position>>8]
        checksum = (~(id + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + p[0] + p[1]))&0xff
        outData = chr(AX_START)
        outData += chr(AX_START)
        outData += chr(id)
        outData += chr(AX_GOAL_LENGTH)
        outData += chr(AX_WRITE_DATA)
        outData += chr(AX_GOAL_POSITION_L)
        outData += chr(p[0])
        outData += chr(p[1])
        outData += chr(checksum)
        port.write(outData)
        sleep(TX_DELAY_TIME)
        return readData(id)

    def moveSpeed(id, position, speed):
        direction(RPI_DIRECTION_TX)
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
        port.write(outData)
        sleep(TX_DELAY_TIME)
        return readData(id)

    def moveRW(id, position):
        direction(RPI_DIRECTION_TX)
        port.flushInput()
        p = [position&0xff, position>>8]
        checksum = (~(id + AX_GOAL_LENGTH + AX_REG_WRITE + AX_GOAL_POSITION_L + p[0] + p[1]))&0xff
        outData = chr(AX_START)
        outData += chr(AX_START)
        outData += chr(id)
        outData += chr(AX_GOAL_LENGTH)
        outData += chr(AX_REG_WRITE)
        outData += chr(AX_GOAL_POSITION_L)
        outData += chr(p[0])
        outData += chr(p[1])
        outData += chr(checksum)
        port.write(outData)
        sleep(TX_DELAY_TIME)
        return readData(id)

    def moveSpeedRW(id, position, speed):
        direction(RPI_DIRECTION_TX)
        port.flushInput()
        p = [position&0xff, position>>8]
        s = [speed&0xff, speed>>8]
        checksum = (~(id + AX_GOAL_SP_LENGTH + AX_REG_WRITE + AX_GOAL_POSITION_L + p[0] + p[1] + s[0] + s[1]))&0xff
        outData = chr(AX_START)
        outData += chr(AX_START)
        outData += chr(id)
        outData += chr(AX_GOAL_SP_LENGTH)
        outData += chr(AX_REG_WRITE)
        outData += chr(AX_GOAL_POSITION_L)
        outData += chr(p[0])
        outData += chr(p[1])
        outData += chr(s[0])
        outData += chr(s[1])
        outData += chr(checksum)
        port.write(outData)
        sleep(TX_DELAY_TIME)
        return readData(id)

    def action():
        direction(RPI_DIRECTION_TX)
        port.flushInput()
        outData = chr(AX_START)
        outData += chr(AX_START)
        outData += chr(AX_BROADCAST_ID)
        outData += chr(AX_ACTION_LENGTH)
        outData += chr(AX_ACTION)
        outData += chr(AX_ACTION_CHECKSUM)
        port.write(outData)
        #sleep(TX_DELAY_TIME)

    def setAngleLimit(id, cwLimit, ccwLimit):
        direction(RPI_DIRECTION_TX)
        port.flushInput()
        cw = [cwLimit&0xff, cwLimit>>8]
        ccw = [ccwLimit&0xff, ccwLimit>>8]
        checksum = (~(id + AX_AL_LENGTH + AX_WRITE_DATA + AX_CW_ANGLE_LIMIT_L + cw[0] + cw[1] + ccw[0] + ccw[1]))&0xff
        outData = chr(AX_START)
        outData += chr(AX_START)
        outData += chr(id)
        outData += chr(AX_AL_LENGTH)
        outData += chr(AX_WRITE_DATA)
        outData += chr(AX_CW_ANGLE_LIMIT_L)
        outData += chr(cw[0])
        outData += chr(cw[1])
        outData += chr(ccw[0])
        outData += chr(ccw[1])
        outData += chr(checksum)
        port.write(outData)
        sleep(TX_DELAY_TIME)
        return readData(id)

    def setPunchLimit(id, punch):
        direction(RPI_DIRECTION_TX)
        port.flushInput()
        p = [punch&0xff, punch>>8]
        checksum = (~(id + AX_PUNCH_LENGTH + AX_WRITE_DATA + AX_PUNCH_L + p[0] + p[1]))&0xff
        outData = chr(AX_START)
        outData += chr(AX_START)
        outData += chr(id)
        outData += chr(AX_PUNCH_LENGTH)
        outData += chr(AX_WRITE_DATA)
        outData += chr(AX_PUNCH_L)
        outData += chr(p[0])
        outData += chr(p[1])
        outData += chr(checksum)
        port.write(outData)
        sleep(TX_DELAY_TIME)
        return readData(id)

    def setCompliance(id, cwMargin, ccwMargin, cwSlope, ccwSlope):
        direction(RPI_DIRECTION_TX)
        port.flushInput()
        checksum = (~(id + AX_COMPLIANCE_LENGTH + AX_WRITE_DATA + AX_CW_COMPLIANCE_MARGIN + cwMargin + ccwMargin + cwSlope + ccwSlope))&0xff
        outData = chr(AX_START)
        outData += chr(AX_START)
        outData += chr(id)
        outData += chr(AX_COMPLIANCE_LENGTH)
        outData += chr(AX_WRITE_DATA)
        outData += chr(AX_CW_COMPLIANCE_MARGIN)
        outData += chr(cwMargin)
        outData += chr(ccwMArgin)
        outData += chr(cwSlope)
        outData += chr(ccwSlope)
        outData += chr(checksum)
        port.write(outData)
        sleep(TX_DELAY_TIME)
        return readData(id)

    def readPosition(id):
        direction(RPI_DIRECTION_TX)
        port.flushInput()
        checksum = (~(id + AX_POS_LENGTH + AX_READ_DATA + AX_PRESENT_POSITION_L + AX_INT_READ))&0xff
        outData = chr(AX_START)
        outData += chr(AX_START)
        outData += chr(id)
        outData += chr(AX_POS_LENGTH)
        outData += chr(AX_READ_DATA)
        outData += chr(AX_PRESENT_POSITION_L)
        outData += chr(AX_INT_READ)
        outData += chr(checksum)
        port.write(outData)
        sleep(TX_DELAY_TIME)
        return readData(id)


    def learnServos(minValue=1, maxValue=6, verbose=False) :
        servoList = []
        for i in range(minValue, maxValue + 1):
            try :
                temp = ping(i)
                servoList.append(i)
                if verbose: print ("Found servo #" + str(i))
                time.sleep(0.1)

            except detail:
                if verbose : print ( "Error pinging servo #" + str(i) + ': ' + str(detail))
                pass
        return servoList

def main():
    port = None
    gpioSet = False
    if(port == None):
        port = Serial("/dev/ttyS0", baudrate=1000000, timeout=0.01)
    if(not gpioSet):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RPI_DIRECTION_PIN, GPIO.OUT)
        gpioSet = True
    direction(RPI_DIRECTION_RX)

    moveSpeed(0x12, 10, 500)
    moveSpeed(0x11, 90, 500)
    moveSpeed(0x12, 120, 500)  
    
if __name__ == '__main__':
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RPI_DIRECTION_PIN, GPIO.OUT)
    main()