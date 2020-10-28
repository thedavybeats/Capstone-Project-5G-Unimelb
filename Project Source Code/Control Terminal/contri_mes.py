import socket
import time
import gpiozero
import sys
sys.path.append("")
import math
import time, datetime
from registers import *
from mpu_9250 import MPU9250
import spidev # To communicate with SPI devices

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

def sendReceive(s, motor_data):
    sendtime_t = time.time()
    time.sleep(float(slip))
    s.sendall(motor_data.encode('utf-8'))
    finger_data = s.recv(1024)
    time.sleep(float(slip))
    recvtime_t = time.time()
    print('Execution Cycle Delay: ', recvtime_t-sendtime_t)
    finger_data = finger_data.decode('utf-8')
    return finger_data

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

# Initialize motor I/O.
Backward_1 = gpiozero.OutputDevice(24) # On/Off output
Forward_1 = gpiozero.OutputDevice(23) #On/Off output
SpeedPWM_1 = gpiozero.PWMOutputDevice(18) # set up PWM pin
Backward_0 = gpiozero.OutputDevice(12) # On/Off output
Forward_0 = gpiozero.OutputDevice(13) #On/Off output
SpeedPWM_0 = gpiozero.PWMOutputDevice(16) # set up PWM pin

# Create SPI connection.
spi = spidev.SpiDev()
spi.open(0,0)

# Create MPU9250 connection.
mpu_0 = MPU9250(
    address_ak=AK8963_ADDRESS, 
    address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address WRIST
    address_mpu_slave=None, 
    bus=1, 
    gfs=GFS_1000, 
    afs=AFS_8G, 
    mfs=AK8963_BIT_16, 
    mode=AK8963_MODE_C100HZ,
    tau=0,
    dtTimer = time.time(),
    gyroYaw = 0)

mpu_1 = MPU9250(
    address_ak=AK8963_ADDRESS, 
    address_mpu_master=MPU9050_ADDRESS_69, # In 0x69 Address ELBOW 
    address_mpu_slave=None, 
    bus=1, 
    gfs=GFS_1000, 
    afs=AFS_8G, 
    mfs=AK8963_BIT_16, 
    mode=AK8963_MODE_C100HZ,
    tau=0,
    dtTimer=time.time(),
    gyroYaw=0)

# MPU9250 calibrition.
mpu_0.calibrate() # Calibrate sensors
mpu_1.calibrate() # Calibrate sensors

mpu_0.configure() # The calibration function resets the sensors, so you need to reconfigure them
mpu_1.configure() # The calibration function resets the sensors, so you need to reconfigure them

# Initialize theta_6.
theta_6 = 0

# Initialize feedback parameter.
finger_0 = 0
finger_1 = 0

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

# Initialize old time.
old_time = time.time()

while True:
    # Read flex sensor data.
    current = [260]*50
    for i in range(50):
        current[i] = analogInput(0)
    current.remove(max(current))
    current.remove(min(current))
    current.remove(max(current))
    current.remove(min(current))
    current.remove(max(current))
    current.remove(min(current))
    current.remove(max(current))
    current.remove(min(current))
    clamp_data = int((sum(current)-max(current)-min(current))/40)
    # Compute clamp data.
    if clamp_data > 280:
        pwm_1 = 533
    else:
        pwm_1 = int(0.1947*pow(clamp_data,2)-109.6*clamp_data+15960)
    # Read ELBOW MPU9250 No.1 data.
    ax_1,ay_1,az_1 = mpu_1.readAccelerometerMaster()
    gx_1,gy_1,gz_1 = mpu_1.readGyroscopeMaster()
    dt = time.time() - old_time
    old_time = time.time()
    # Compute ELBOW pitch data.
    if az_1 < 0:
        pass
    else:
        theta_5 = math.degrees\(math.atan2(ay_1, math.sqrt(ax_1*ax_1+az_1*az_1)))
    pwm_5 = int(-1000.0 * theta_5 / 90.0 +2500.0)
    # Read WRIST MPU9250 No.0 data.
    ax_0,ay_0,az_0 = mpu_0.readAccelerometerMaster()
    # Compute WRIST roll data.
    theta_2 = math.degrees(math.atan2(ax_0,az_0))
    pwm_2 = int(-2000.0 * theta_2 / 180.0 + 1500.0)
    # Compute WRIST pitch data.
    if az_0 < 0:
        pass
    else:
        theta_3 = math.degrees\(math.atan2\(ay_0, math.sqrt(ax_0*ax_0+az_0*az_0))) - theta_5
    pwm_3 = int(2000.0 * theta_3 / 180.0 + 1500.0)
    # Compute MIDDLE ARM data.
    if theta_3 >= -90 and theta_3 <= 0:
        alpha = 0.7*math.sin(math.radians(-theta_3))
        if alpha <= 1 and alpha >= 0:
            theta_4 = math.degrees(math.asin(alpha))
            pwm_4 = int(1000.0 * theta_4 / 90.0 + 1500.0)
            theta_3 = theta_3 - theta_4
            pwm_3 = int(2000.0 * theta_3 / 180.0 + 1500.0)
        else:
            pwm_4 = 0
    else:
        pwm_4 = 0
    # Compute Elbow yaw data.
    if abs(gz_1) <= 5:
        gz_1 = 0
    theta_6_temp = theta_6 + gz_1 * dt
    if theta_6_temp >= 45:
        theta_6 = 45
    elif theta_6_temp <= -45:
        theta_6 = -45
    else:
        theta_6 = theta_6_temp
    pwm_6 = int(1000.0*theta_6 / 90.0 + 1500.0) 
    # Convert the motion information.
    motor_data = str(pwm_1) + ' ' + str(pwm_2) + ' '\
               + str(pwm_3) + ' ' + str(pwm_4) + ' '\
               + str(pwm_5) + ' ' + str(pwm_6)
    # Send the motion information and receive the feedback.
    finger_data = sendReceive(s, motor_data)
    try:
        finger_data = finger_data.split(' ')
        new_finger_0 = int(finger_data[0])
        new_finger_1 = int(finger_data[1])
        if new_finger_0 >= 0 and new_finger_0 <= 6931:
            finger_0 = new_finger_0
        if new_finger_1 >= 0 and new_finger_1 <= 6931:
            finger_1 = new_finger_1
    except:
        pass
    # Set motor direction.
    Backward_0.on() # Sets Backward Direction pin on
    Forward_0.off() # Sets Backward Direction pin on
    Backward_1.on() # Sets Backward Direction pin on
    Forward_1.off() # Sets Backward Direction pin on
    # Output motor strength.
    SpeedPWM_0.value = finger_0/6931 # Sets the duty cycle of the PWM between 0-1
    SpeedPWM_1.value = finger_1/6931 # Sets the duty cycle of the PWM between 0-1

