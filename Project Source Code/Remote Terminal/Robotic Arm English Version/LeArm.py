#!/usr/bin/python3
# encoding: utf-8
import time
import sqlite3 as sql    
import pigpio
from LeServo import PWM_Servo   
import os

Servos = ()     
runningAction = False   
pi = None
stopRunning = False     


def setServo(servoId, pos, time):     
    global runningAction
    
    if servoId < 1 or servoId > 6:
        return
    if pos > 2500:
        pos = 2500
    elif pos < 500:
        pos = 500
    else:
        pass    
    if time > 30000:
        time = 30000
    elif time < 20:
        time = 20
    else:
        pass    
    if runningAction is False:  
        Servos[servoId - 1].setPosition(pos, time)

# Current angle
def setServo_CMP(servoId, pos, time):
    # print(servoId, pos, time)
    if servoId < 1 or servoId > 6:
        return
    # print(Servos[servoId-1].getPosition())
    setServo(servoId, Servos[servoId - 1].getPosition() + pos, time);


def setDeviation(servoId, d):
    global runningAction
    if servoId < 1 or servoId > 6:
        return
    if d < -300 or d > 300:
        return
    if runningAction is False:
        Servos[servoId -1].setDeviation(d) 


def stopActionGroup():  
    global stopRunning
    stopRunning = True


def runActionGroup(actNum, times):
    global runningAction
    global stopRunning
    actNum = "./ActionGroups/" + actNum + ".d6a"
    # print(actNum)
    if os.path.exists(actNum) is True: 
        ag = sql.connect(actNum)   
        cu = ag.cursor()    
        cu.execute("select * from ActionGroup")    
        if runningAction is False:  
            runningAction = True
            while True:
                if stopRunning is True:     
                    stopRunning = False
                    runningAction = False
                    cu.close()  
                    ag.close()  
                    break
                act = cu.fetchone() 
                if act is not None:
                    # print(act)
                    for i in range(0, 6, 1):
                        Servos[i].setPosition(act[2+i], act[1])
                    time.sleep(float(act[1]*1.2)/1000.0)    # Running time
                else:
                    runningAction = False
                    cu.close()
                    ag.close()
                    break
    else:
        runningAction = False
        print("Cannot find the file.")


def initLeArm(d):
    global Servos
    global pi
    pi = pigpio.pi()   
    # Initialization
    servo1 = PWM_Servo(pi, 12,  deviation=d[0], control_speed = True) 
    servo2 = PWM_Servo(pi, 16, deviation=d[1], control_speed = True)
    servo3 = PWM_Servo(pi, 20, deviation=d[2], control_speed = True)
    servo4 = PWM_Servo(pi, 21, deviation=d[3], control_speed = True)
    servo5 = PWM_Servo(pi, 19, deviation=d[4], control_speed = True)
    servo6 = PWM_Servo(pi, 13, deviation=d[5], control_speed = True)
    Servos = (servo1, servo2, servo3, servo4, servo5, servo6)
    
    
    #for i in range(0, 6, 1):
    #    Servos[i].setPosition(1500, 1000)  
    Servos[0].setPosition(1500, 1000)
    Servos[1].setPosition(1500, 1000)
    Servos[2].setPosition(1500, 1000)
    Servos[3].setPosition(1500, 1000)
    Servos[4].setPosition(2400, 1000)
    Servos[5].setPosition(1500, 1000)





def stopLeArm():
    print("The Robotic Arm stops.")
    pi.stop()   

if __name__ == "__main__":
    initLeArm([0,0,0,0,0,0])
    setServo(1, 1000, 1000)
    time.sleep(1.1)
