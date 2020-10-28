import socket
import time
import cv2
import numpy as np

# Public IP
# host = '34.87.223.236'
# port = 39725

# Local IP
host = '192.168.0.173'
port = 9725

def setupSocket():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    print("Connection established.")
    return s

# Switch on the Extra Delay Mode.
while True:
    sli = input("Extra Delay Mode: ")
    if sli == 'no':
        slip = 0
        break
    elif sli == 'yes':
        slip = 0.01
        break

# Switch on the Extra Packet Mode.
while True:
    hd_input = input("Extra Packet Mode: ")
    if hd_input == 'yes':
        hd_on = True
        break
    elif hd_input == 'no':
        hd_on = False
        break
    
# Set the compression rate.
while True:
    qual = int(input("Video Quality: "))
    if qual >= 1 and qual <= 100:
        break

redun = bytes(512)
s = setupSocket()
cap = cv2.VideoCapture("http://127.0.0.1:8080/?action=stream?dummy=param.mjpg")

# Enter the password.
while True:
    pword = input("Enter the word: ")
    s.sendall(pword.encode('utf-8'))
    aword = s.recv(1024)
    aword = aword.decode('utf-8')
    if aword == 'Login Successful':
        time.sleep(2)
        print(aword)
        break
    else:
        time.sleep(1)
        print(aword)
        time.sleep(1)
        continue
time.sleep(5)

# MJPEG Video Streaming.
while True:
    # Read the frame.
    _, frame = cap.read()
    # JPEG encoding.
    _, img_encode = cv2.imencode('.jpg', frame,\
                     [int(cv2.IMWRITE_JPEG_QUALITY),qual])
    # Convert array to string.
    img_array = np.array(img_encode)
    img_data = img_array.tostring()
    # Send the redundant data.
    if hd_on:
        for _ in range(8):
            conn.sendall(redun)
            _ = conn.recv(4)
    # Send the image data.
    s.sendall(bytes(str(len(img_data)).ljust(16),\
                     encoding = "utf8"))
    time.sleep(float(slip))
    s.sendall(img_data)

