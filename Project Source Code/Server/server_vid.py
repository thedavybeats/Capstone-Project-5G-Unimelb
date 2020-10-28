import socket
import time

host = ''
port = 9725 # Local
# port = 39725 # Public

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
    s.listen(2) # Allows two connection at a time.
    conn, address = s.accept()
    print("Connected to: " + address[0] + ":" + str(address[1]))
    return conn

s = setupServer()

conn1 = setupConnection()
conn2 = setupConnection()
# Check the password from eva_01.
while True:
    pword = conn1.recv(1024)
    pword = pword.decode('utf-8')
    if pword == '5g':
        break
    else:
        aword = 'Login Failed'
        conn1.sendall(aword.encode('utf-8'))
        continue
# Check the password from eva_02.
while True:
    pword = conn2.recv(1024)
    pword = pword.decode('utf-8')
    if pword == '5g':
        break
    else:
        aword = 'Login Failed'
        conn2.sendall(aword.encode('utf-8'))
        continue
aword = 'Login Successful'
conn1.sendall(aword.encode('utf-8'))
conn2.sendall(aword.encode('utf-8'))

# Send data and receive message.
while True:
    # Receive the image fragments from eva_01.
    image_frag = conn1.recv(65536)
    # Send the image data to eva_02.
    conn2.sendall(image_frag)

