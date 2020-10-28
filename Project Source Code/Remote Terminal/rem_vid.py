import socket
import time
import cv2
import numpy as np

host = ''
port = 9725

def setupServer():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("Socket created.")
    try:
        s.bind((host, port))
    except socket.error as msg:
        print(msg)
    print("Socket bind complete.")
    return s

def setupConnection():
    s.listen(1) # Allows one connection at a time.
    conn, address = s.accept()
    print("Connected to: " + address[0] + ":" + str(address[1]))
    return conn

# Switch on the Extra Packet Mode.
while True:
    hd_input = input("Extra Pakcet Mode: ")
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
s = setupServer()
cap = cv2.VideoCapture("http://127.0.0.1:8080/?action=stream?dummy=param.mjpg")

conn = setupConnection()
# Check the password.
while True:
    pword = conn.recv(1024)
    pword = pword.decode('utf-8')
    if pword == '5g':
        aword = 'Login Successful'
        time.sleep(2)
        conn.sendall(aword.encode('utf-8'))
        break
    else:
        aword = 'Login Failed'
        conn.sendall(aword.encode('utf-8'))
        continue
time.sleep(5)

# MJPEG video streaming.
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
    conn.sendall(bytes(str(len(img_data)).ljust(16),\
                     encoding = "utf8"))
    conn.sendall(img_data)

