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
from __init__ import ICM20948
from threading import Thread
import ahrs
from ahrs.common.orientation import q2euler
import numpy as np
from math import sin, cos, tan, pi
# Local IP
host = '192.168.0.142'
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
#     print('Execution Cycle Delay: ', recvtime_t-sendtime_t)
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
        slip = 0.15
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
    dtTimer=time.time(),
    gyroYaw=0)


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

#initilize the EKF for mpu9250
orientation = ahrs.filters.EKF()
Q = np.tile([1., 0., 0., 0.], (1, 1)) # Allocate for quaternions
Q = Q[0]

# Set the offset of magnetometer, the value is calibrated before the experiment
b =  [[-43.3843825 ],[-41.1611215 ],[ 12.13742856]]
A_1 =  [[8.39968685e-01, 8.87931233e-03, 1.07547548e-02],[8.87931233e-03, 8.58841081e-01, 4.77558199e-04],[1.07547548e-02, 4.77558199e-04, 8.33393298e-01]]
start = time.time()
# MPU9250 calibrition.
print("Calibrating...")
mpu_0.calibrate() # Calibrate sensors
mpu_1.calibrate() # Calibrate sensors

mpu_0.configure() # The calibration function resets the sensors, so you need to reconfigure them
mpu_1.configure() # The calibration function resets the sensors, so you need to reconfigure them

#Initialize the time interval and estimate angle
dt = 0.0

phi_hat = 0.0
theta_hat = 0.0

# Initialise matrices and variables
C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])
P = np.eye(4)
Q = np.eye(4)
R = np.eye(2)

state_estimate = np.array([[0], [0], [0], [0]])

#Initialize the estimate angle
phi_hat_0 = 0.0
theta_hat_0 = 0.0

# Initialise matrices and variables
C_0 = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])
P_0 = np.eye(4)
Q_0 = np.eye(4)
R_0 = np.eye(2)

state_estimate_0 = np.array([[0], [0], [0], [0]])

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
    # Process data from MPU9250
    ss = np.array(mpu_1.readMagnetometerMaster()).reshape(3, 1)
    ss = np.dot(A_1, ss - b)
    x,y,z = ss[0,0], ss[1,0], ss[2,0]
    ax, ay, az = mpu_1.readAccelerometerMaster()
    gx, gy, gz = mpu.readGyroscopeMaster()
    dt = time.time()-start
    start = time.time()

    ax = ax *9.8
    ay = ay*9.8
    az = az*9.8
    if abs(gx) < 5:
        gx = 0
    if abs(gy) < 5:
        gy = 0
    if abs(gz) < 6:
        gz = 0
    gx = gx*pi/180
    gy = gy*pi/180
    gz = gz*pi/180

    #Update the state for EKF using quaternion
    acce = np.array([ax,ay,az])
    gyo = np.array([gx,gy,gz])
    mag = np.array([x,y,z])
    orientation.Dt = dt
    Q = orientation.update(gyo, acce,mag,Q)
    pitch,roll,yaw = q2euler(Q)

    # Get accelerometer measurements and remove offsets
    [phi_acc, theta_acc] = imu.get_acc_angles()

    # Gey gyro measurements and calculate Euler angle derivatives

    phi_dot_0 = gx + sin(phi_hat) * tan(theta_hat) * gy + cos(phi_hat) * tan(theta_hat) * gz
    theta_dot = cos(phi_hat) * gy - sin(phi_hat) * gz

    # Kalman filter
    A = np.array([[1, -dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, -dt], [0, 0, 0, 1]])
    B = np.array([[dt, 0], [0, 0], [0, dt], [0, 0]])

    gyro_input = np.array([[phi_dot_0], [theta_dot]])
    state_estimate = A.dot(state_estimate) + B.dot(gyro_input)
    P = A.dot(P.dot(np.transpose(A))) + Q

    measurement = np.array([[phi_acc], [theta_acc]])
    y_tilde = measurement - C.dot(state_estimate)
    S = R + C.dot(P.dot(np.transpose(C)))
    K = P.dot(np.transpose(C).dot(np.linalg.inv(S)))
    state_estimate = state_estimate + K.dot(y_tilde)
    P = (np.eye(4) - K.dot(C)).dot(P)

    phi_hat = state_estimate[0]
    theta_hat = state_estimate[2]

    # Compute the pitch PWM
    if az < 0:
        pass
    else:
        theta_5 = math.degrees(theta_hat * 180.0 / pi)
    pwm_5 = int(-1000.0 * theta_5 / 90.0 +2500.0)
    # Read WRIST MPU9250 No.0 data.

    [phi_acc_0, theta_acc_0] = imu.get_acc_angles()
    ax_0,ay_0,az_0 = mpu_0.readAccelerometerMaster()
    gx_0,gy_0,gz_0 = mpu_0.readGyroscopeMaster()
    phi_dot_0 = gx_0 + sin(phi_hat_0) * tan(theta_hat_0) * gy_0 + cos(phi_hat_0) * tan(theta_hat_0) * gz_0
    theta_dot_0 = cos(phi_hat_0) * gy_0 - sin(phi_hat_0) * gz_0

    # Kalman filter
    A_0 = np.array([[1, -dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, -dt], [0, 0, 0, 1]])
    B_0 = np.array([[dt, 0], [0, 0], [0, dt], [0, 0]])

    gyro_input_0 = np.array([[phi_dot_0], [theta_dot_0]])
    state_estimate_0 = A_0.dot(state_estimate_0) + B_0.dot(gyro_input_0)
    P_0 = A_0.dot(P_0.dot(np.transpose(A_0))) + Q_0

    measurement_0 = np.array([[phi_acc_0], [theta_acc_0]])
    y_tilde_0 = measurement_0 - C_0.dot(state_estimate_0)
    S_0 = R + C_0.dot(P_0.dot(np.transpose(C_0)))
    K_0 = P_0.dot(np.transpose(C_0).dot(np.linalg.inv(S_0)))
    state_estimate_0 = state_estimate_0 + K_0.dot(y_tilde_0)
    P_0 = (np.eye(4) - K_0.dot(C_0)).dot(P_0)

    phi_hat_0 = state_estimate_0[0]
    theta_hat_0 = state_estimate_0[2]


    # Compute WRIST roll data.
    theta_2 = math.degrees(phi_hat_0)
    pwm_2 = int(-2000.0 * theta_2 / 180.0 + 1500.0)
    # Compute WRIST pitch data.
    if az_0 < 0:
        pass
    else:
        theta_3 = math.degrees(theta_hat_0) - theta_5
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
    theta_6 = -math.degrees(yaw.item())
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
