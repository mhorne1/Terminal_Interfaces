#!/usr/bin/env python
# coding: utf-8

# In[1]:


import socket
import struct
import threading
import time
import queue

import messagetools as mt

HOST = socket.gethostname()
PORT = 5678
HEADER_SIZE = 10
HEADER_FORMAT = "!IHI"
CONN_COUNT_MAX = 1 # Temporary
SEND_COUNT_MAX = 5 # Temporary

msgqueue = queue.Queue()
event = threading.Event()

#msg = "Server Message: 0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!\"#$%&\'()*+,-./:;<=>?@[\\]^_`{|}~ \t\n\r\x0b\x0c"
msg = "Server Message: 0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"

messagelength = len(msg)
messagetype = 1
messagenumber = 1


def msgtype1(message):
    '''
    Encodes type 1 messages
    Parameters
    ----------
    message : Text string
    Returns
    -------
    Byte encoded text string
    '''
    return message.encode("utf-8")


def pyserverthread(xhost, xport, xheaderformat, xmsgtype, xmsgnumber, xmyqueue, orig_msg):
    '''
    Creates socket, waits for connection by client, and sends messages to clients
    Parameters
    ----------
    xhost : IP address
    xport : Port number
    xheaderformat : String describing header struct, e.g. "!2I2F"
    xmsgtype : Number indicating type of message
    xmsgnumber : Initial message number
    xmyqueue : Queue that can contain new messages to be sent
    orig_msg : Initial text string to be sent
    Returns
    -------
    None
    '''
    print("Running pyserver thread...")
    pyserver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    pyserver.bind((xhost, xport))
    pyserver.listen(5)
    
    xmsg = orig_msg
    
    msg_delay = 3 # Number of seconds between sending each message
    
    conn_count = 0
    send_count = 0
    
    while conn_count < CONN_COUNT_MAX:
        conn_count += 1
        print("Listening for client...")
        clientsocket, clientaddress = pyserver.accept()
        with clientsocket:
            print(f"Connection from {clientaddress}")
            try:
                while send_count < SEND_COUNT_MAX:
                    fullmsg = ''
                    if xmyqueue.empty() == False:
                        print("Pyserver thread getting message from queue")
                        xmsg = xmyqueue.get()
                    xmsglen = len(xmsg)
                    headerpacked = struct.pack(xheaderformat, xmsglen, xmsgtype, xmsgnumber)
                    xmsgnumber += 1
                    #packedmsg = xmsg.encode("utf-8")
                    if xmsgtype == 1:
                        packedmsg = msgtype1(xmsg)
                    fullmsg = headerpacked + packedmsg
                    try:
                        clientsocket.send(fullmsg)
                    except socket.error as emsg:
                        print(f"socket.error exception: {emsg}")
                    send_count += 1
                    time.sleep(msg_delay)
            except socket.error as emsg:
                print(f"socket.error exception: {emsg}")
                time.sleep(msg_delay)
            except KeyboardInterrupt:
                print("Keyboard / Control-C exception")
                event.set()
                break
        send_count = 0
    print("Shutting down pyserver thread!")
    pyserver.close()
    
    
thr1 = threading.Thread(target=pyserverthread, args=(HOST, PORT, HEADER_FORMAT, messagetype, messagenumber, msgqueue, msg, ))
thr2 = threading.Thread(target=mt.get_message, args=(msgqueue, ))
thr1.start()
thr2.start()
thr1.join()
thr2.join()


while not msgqueue.empty():
    msgqueue.get()
print(f"msgqueue is empty")

print("EOF")

