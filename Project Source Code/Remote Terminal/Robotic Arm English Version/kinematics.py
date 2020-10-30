#!/usr/bin/env python3
# coding:utf8
#
from math import *
import LeArm
import time
import matplotlib.pyplot as plt
import LeConf

d = []
for i in range(0, len(LeConf.Deviation), 1):
    if LeConf.Deviation[i] > 1600 or LeConf.Deviation [i]< 1400:
        print("Out of range (1200-1800)")
    else:
        d.append(LeConf.Deviation[i] - 1500)

LeArm.initLeArm(tuple(d))
targetX = 0
targetY = 0
targetZ = 4000
lastX = 0
lastY = 0
lastZ = 4000

l0 = 960    # Distance from the base to motor 2: 9.6cm
l1 = 1050   # Distance from motor 2 to motor 3: 10.5cm
l2 = 880    # Distance from motor 3 to motor 4: 8.8cm
l3 = 1650   # Distance from motor 4 to the clamp: 16.5cm


alpha = 0   
# theta3 = 0  
# theta4 = 0  
# theta5 = 0  
# theta6 = 0  


def kinematic_analysis(x, y, z, Alpha):
    global alpha
    global l0, l1, l2, l3
    if x == 0:
        theta6 = 90.0
    elif x < 0:
        theta6 = atan(y / x)
        theta6 = 180 + (theta6 * 180.0/pi)
    else:
        theta6 = atan(y / x)
        theta6 = theta6 * 180.0 / pi
    y = sqrt(x*x + y*y)     
    y = y-l3 * cos(Alpha*pi/180.0)    # ytotal - y3 = y2 + y1
    z = z-l0-l3*sin(Alpha*pi/180.0)   # z1 + z2
    if z < -l0:
        return False
    if sqrt(y*y + z*z) > (l1 + l2):
        return False
    aaa = -(y*y+z*z-l1*l1-l2*l2)/(2*l1*l2)
    if aaa > 1 or aaa < -1:
        return False
    theta4 = acos(aaa)  
    theta4 = 180.0 - theta4 * 180.0 / pi    
    if theta4 > 135.0 or theta4 < -135.0:
        return False
    alpha = acos(y / sqrt(y * y + z * z))
    bbb = (y*y+z*z+l1*l1-l2*l2)/(2*l1*sqrt(y*y+z*z))
    if bbb > 1 or bbb < -1:
        return False
    if z < 0:
        zf_flag = -1
    else:
        zf_flag = 1
    theta5 = alpha * zf_flag + acos(bbb)
    theta5 = theta5 * 180.0 / pi
    if theta5 > 180.0 or theta5 < 0:
        return False

    theta3 = Alpha - theta5 + theta4
    if theta3 > 90.0 or theta3 < -90.0:
        return False
    return theta3, theta4, theta5, theta6     


def averagenum(num):
    nsum = 0
    for i in range(len(num)):
        nsum += num[i]
    return nsum / len(num)


def ki_move(x, y, z, movetime):
    alpha_list = []
    for alp in range(-25, -65, -1):
        if kinematic_analysis(x, y, z, alp):
            alpha_list.append(alp)
    if len(alpha_list) > 0:
        if y > 2150:
            best_alpha = max(alpha_list)
        else:
            best_alpha = min(alpha_list)
        theta3, theta4, theta5, theta6 = kinematic_analysis(x, y, z, best_alpha)
        pwm_6 = int(2000.0 * theta6 / 180.0 + 500.0)
        pwm_5 = int(2000.0 * (90.0 - theta5) / 180.0 + 1500.0)
        pwm_4 = int(2000.0 * (135.0 - theta4) / 270.0 + 500.0)
        pwm_3 = int(2000.0 * theta3 / 180.0 + 1500.0)
        LeArm.setServo(3, pwm_3, movetime)
        LeArm.setServo(4, pwm_4, movetime)
        LeArm.setServo(5, pwm_5, movetime)
        LeArm.setServo(6, pwm_6, movetime)
        time.sleep(movetime / 1000.0)
        return True
    else:
        return False


def plt_image(x_list, y_list):
    #plt.plot(x, y, label = 'target')
    plt.title('Scatter Plot')
    plt.xlabel('x-value')
    plt.ylabel('y-label')
    # plt.scatter(x, y, s, c, marker)
    # x: x-axis
    # y：y-axis
    # s：default-rcParams['lines.markersize'] ** 2
    # c: colour
    # marker: default-'o'
    plt.legend()

    plt.scatter(x_list, y_list, s=10, c="#ff1212", marker='o')

if __name__ == '__main__':
     try:
         for y in range(1250, 3250, 50):
             for x in range(1500, -1500, -50):
                 ok = kinematic_analysis(x, y, 200, 0)
                 if ok:
                     print(x, y)
                     plt_image(x, y)
                 else:
                     for a in range(0, -60, -1):
                         ok = kinematic_analysis(x, y, 200, a)
                         if ok:
                             print('a', (x, y))
                             plt_image(x, y)
                             #print('a', a)
                             break
                 # time.sleep(0.1)
         plt.show()
     except Exception as e:
         print(e)
