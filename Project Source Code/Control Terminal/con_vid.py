import socket
import time
import cv2
import numpy as np

host = '192.168.0.142'
port = 9725

def setupSocket():  
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    print("Connection established.")
    return s

def recv_all(s, count):
    buf = bytes()
    while count:
        newbuf = s.recv(count)
        if not newbuf:
            return 'rewait'
        buf += newbuf
        count -= len(newbuf)
    # Check if the header is corrupted.
    if len(newbuf) == 16:
        try:
            _ = int(newbuf)
        except:
            return 'rewait'
    return buf

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
        packet_on = True
        redun = bytes(0)
        redun_size = 4096
        break
    elif hd_input == 'no':
        hd_on = False
        packet_on = False
        break

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
        break
    else:
        time.sleep(1)
        print(aword)
        time.sleep(1)
        continue
    
# # Set the full screen window.
# cv2.namedWindow("Video Streaming", cv2.WINDOW_NORMAL)
# cv2.setWindowProperty("Video Streaming",\
#                        cv2.WND_PROP_FULLSCREEN,\
#                        cv2.WINDOW_FULLSCREEN)

while True:
    # Receive the redundant data.
    if hd_on and packet_on:
        while True:
            redun_rec = s.recv(1024)
            redun = redun + redun_rec
            s.sendall(bytes(1))
            if len(redun) == redun_size:
                redun = bytes(0)
                packet_on = False
                break
    data_len = recv_all(s, 16)
    # MJPEG video streaming.
    if data_len == 'rewait':
        continue
    if len(data_len) == 16:
        time.sleep(float(slip))
        img_data = recv_all(s, int(data_len))
        packet_on = True
        # Check if the JPEG data is corrupted.
        if img_data[0] != 255 or img_data[1] != 216\
            or img_data[-2] != 255 or img_data[-1] != 217:
            continue
        # Convert string back to array.
        img_array = np.frombuffer(img_data, dtype='uint8')
        # JPEG decoding.
        frame = cv2.imdecode(img_array, 1)
        # Specify the size.
        frame = cv2.resize(frame, (800, 480))
        # Play the video.
        cv2.imshow("Video Streaming", frame)
        cv2.waitKey(1)

