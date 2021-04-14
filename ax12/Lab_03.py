import mysql.connector
from time import sleep
from serial import Serial
import RPi.GPIO as GPIO
import time
from Ax12 import Ax12

db = mysql.connector.connect(user='sql5405530', password='BVnFU6eGiS', host='sql5.freemysqlhosting.net', database='sql5405530')
cur = db.cursor()

cur.execute("SELECT * FROM TEST WHERE ID = 3")

for results in cur.fetchall():
    ID = results[0]
    SPEED = results[1]
    AX1 = results[2]
    AX2 = results[3]
    PAUSE = results[4]


delay_0=0.001
ax12_Lib = Ax12()
ax12_Lib.__init__()
time.sleep(1)


ax12_Lib.ping(AX1)
time.sleep(delay_0)
ax12_Lib.ping(AX2)
time.sleep(delay_0)


ax12_Lib.setAngleLimit(AX1, 0, 0)
time.sleep(delay_0)
ax12_Lib.setAngleLimit(AX2, 0, 0)
time.sleep(1)

ax12_Lib.Speed(AX1, SPEED)
time.sleep(delay_0)
ax12_Lib.Speed(AX2, SPEED+1024)
time.sleep(0.1)
    
time.sleep(PAUSE)
ax12_Lib.Speed(AX1, 0)
time.sleep(delay_0)
ax12_Lib.Speed(AX2, 0)
time.sleep(delay_0)

ax12_Lib.direction(Ax12.RPI_DIRECTION_TX)