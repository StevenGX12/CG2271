import socket

TCP_IP = '192.168.1.1'
TCP_PORT = 2001
BUFFER_SIZE = 1024


# msg = b'\xff\x00\xfe\x00\x00\x00\xff'
msg = b'\xfe\xfa\x7e\x03\x03\x\xff'


# msg = b'\xff\xfe\x7f\x00\x00\x00\xff'
# \xFF
# \x00 - base indicator 
# \xleft motor speed
# \xright motor speed
# \xFF 


try:
    s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP,TCP_PORT))
    print("connected")

    s.send(msg)
    # data = s.recv(BUFFER_SIZE)
    
    s.close()
    print("message sent")
except:
    print("error")