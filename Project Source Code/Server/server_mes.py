import socket
import time

host = ''
port = 9125 # Local
# port = 39125 # Public

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
    # Receive the message from eva_01.
    message1 = conn1.recv(1024)
    # Receive the message from eva_02.
    message2 = conn2.recv(1024)
    # Send the message1 to eva_02.
    conn2.sendall(message1)
    # Send the message2 to eva_01.
    conn1.sendall(message2)

