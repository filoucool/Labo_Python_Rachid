from time import sleep
from serial import Serial
import RPi.GPIO as GPIO
import time


delay_0=0.001
ax12_Lib = Ax12()
ax12_Lib.__init__()
time.sleep(1)


dynamixel_id1 = 17
dynamixel_id2 = 18
ax12_Lib.ping(dynamixel_id1)
time.sleep(delay_0)
ax12_Lib.ping(dynamixel_id2)
time.sleep(delay_0)


ax12_Lib.setAngleLimit(dynamixel_id1, 0, 0)
time.sleep(delay_0)
ax12_Lib.setAngleLimit(dynamixel_id2, 0, 0)
time.sleep(1)

time.sleep(1)
ax12_Lib.Speed(dynamixel_id1, 1023+250)
time.sleep(5.7)
ax12_Lib.Speed(dynamixel_id1, 0)
time.sleep(delay_0)
ax12_Lib.Speed(dynamixel_id2, 0)
time.sleep(delay_0)

ax12_Lib.direction(Ax12.RPI_DIRECTION_TX)



