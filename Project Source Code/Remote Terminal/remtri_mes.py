import socket
import time
import math
import spidev # To communicate with SPI devices
import numpy as np
import threading
import signal
import LeArm
import kinematics as kin
import RPi.GPIO as GPIO

# Public IP
# host = '34.87.223.236'
# port = 39125

# Local IP
host = '192.168.0.173'
port = 9125

def setupSocket():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    print("Connection established.")
    return s

# Send feedback ionformation and receive motion information.
def sendReceive(s, finger_data):
    time.sleep(float(slip))
    s.sendall(finger_data.encode('utf-8'))
    motor_data = s.recv(1024)
    time.sleep(float(slip))
    motor_data = motor_data.decode('utf-8')
    return motor_data

# Read MCP3008 data
def analogInput(channel):
    spi.max_speed_hz = 1350000
    adc = spi.xfer2([1,(8+channel)<<4,0])
    data = ((adc[1]&3) << 8) + adc[2]
    return data

# Switch on the Extra delay mode.
while True:
    sli = input("Extra Delay Mode: ")
    if sli == 'no':
        slip = 0
        break
    elif sli == 'yes':
        slip = 0.055
        break
    
# Initialize finger data buffer.
i = 0
current_0 = [0]*3
current_1 = [0]*3

# Initialize movement time.
movetime = 200

# Initialize opening gesture.
pwm_1 = 1500
pwm_2 = 1500
pwm_3 = 1500
pwm_4 = 1500
pwm_5 = 2400
pwm_6 = 1500

# Start SPI connection
spi = spidev.SpiDev() # Created an object
spi.open(0,0)

# Establish network connection.
s = setupSocket()

# Enter the password.
while True:
    pword = input("Enter the word: ")
    s.sendall(pword.encode('utf-8'))
    aword = s.recv(1024)
    aword = aword.decode('utf-8')
    if aword == 'Login Successful':
        time.sleep(2)
        print(aword)
        time.sleep(3)
        break
    else:
        time.sleep(1)
        print(aword)
        time.sleep(1)
        continue

while True:
    # Read pressure sensor data.
    if analogInput(1) <= 120:
        current_0[i] = 0
    else:
        current_0[i] = analogInput(1) # Reading from CH1 THUMB
    if analogInput(0) <= 120:
        current_1[i] = 0
    else:
        current_1[i] = analogInput(0) # Reading from CH0 INDEX FINGER
    i = (i + 1) % 3
    finger_0 = int(400*math.log(current_0[i]*0.7+current_0[(i-1)%3]*0.2\+current_0[(i-2)%3]*0.1+1))
    finger_1 = int(400*math.log(current_1[i]*0.7+current_1[(i-1)%3]*0.2\+current_1[(i-2)%3]*0.1+1))
    # Convert the data.
    finger_data = str(finger_0) + ' ' + str(finger_1)
    # Send feedback ionformation and receive motion information.
    motor_data = sendReceive(s, finger_data)
    try:
        motor_data = motor_data.split(' ')
        new_pwm_1 = int(motor_data[0])
        new_pwm_2 = int(motor_data[1])
        new_pwm_3 = int(motor_data[2])
        new_pwm_4 = int(motor_data[3])
        new_pwm_5 = int(motor_data[4])
        new_pwm_6 = int(motor_data[5])
        # Data lock.
        if new_pwm_1 >= 500 and new_pwm_1 <= 2500:
            pwm_1 = new_pwm_1
        if new_pwm_2 >= 1000 and new_pwm_2 <= 2000:
            pwm_2 = new_pwm_2
        if new_pwm_3 >= 500 and new_pwm_3 <= 2500:
            pwm_3 = new_pwm_3
        # Limit the MIDDLE ARM.
        if new_pwm_4 >= 1500 and new_pwm_4 <= 2500:
            pwm_4 = new_pwm_4
        # Limit ELBOW SWING.
        if new_pwm_5 >= 1500 and new_pwm_5 <= 2400:
            pwm_5 = new_pwm_5
        if new_pwm_6 >= 1000 and new_pwm_6 <= 2000:
            pwm_6 = new_pwm_6
    except:
        pass
    # Execute the motion.
    LeArm.setServo(1, pwm_1, movetime) # CLAMP
    LeArm.setServo(2, pwm_2, movetime) # WRIST ROTATE
    LeArm.setServo(3, pwm_3, movetime) # WRIST SWING
    LeArm.setServo(4, pwm_4, movetime) # MIDDLE ARM
    LeArm.setServo(5, pwm_5, movetime) # ELBOW SWING
    LeArm.setServo(6, pwm_6, movetime) # ELBOW ROTATE
    time.sleep(movetime/1000 + 0.001)
    
