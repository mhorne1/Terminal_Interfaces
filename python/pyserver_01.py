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
serverevent = threading.Event()

#msg = "Server Message: 0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!\"#$%&\'()*+,-./:;<=>?@[\\]^_`{|}~ \t\n\r\x0b\x0c"
msg = "Server Message: 0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"

messagetype = 1 # String message

def server_thread(xhost, xport, xheaderformat, xmsgtype, xmsgqueue):
    '''
    Creates socket, waits for connection by client, and sends messages to clients
    Parameters
    ----------
    xhost : IP address
    xport : Port number
    xheaderformat : String describing header struct, e.g. "!2I2F"
    xmsgtype : Number indicating type of message
    xmsgqueue : Queue that can contain new messages to be sent
    Returns
    -------
    None
    '''
    print("Running pyserver thread...")
    pyserver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    pyserver.bind((xhost, xport))
    pyserver.listen(5)
    
    xmsg = ""
    
    msg_delay = 3 # Number of seconds between sending each message
    
    conn_count = 0
    send_count = 0
    
    xmsgnumber = 1
    
    while conn_count < CONN_COUNT_MAX:
        conn_count += 1
        print("Listening for client...")
        clientsocket, clientaddress = pyserver.accept()
        with clientsocket:
            print(f"Connection from {clientaddress}")
            send_count = 0
            while send_count < SEND_COUNT_MAX:
                fullmsg = ""
                if serverevent.is_set():
                    clientsocket.close()
                    conn_count = CONN_COUNT_MAX
                    break
                elif xmsgqueue.empty() == False:
                    print("Thread getting message from queue")
                    xmsg = xmsgqueue.get()
                elif xmsg == "":
                    print("Waiting for message")
                    time.sleep(1)
                    continue
                xmsglen = len(xmsg)
                headerpacked = struct.pack(xheaderformat, xmsglen, xmsgtype, xmsgnumber)
                xmsgnumber += 1
                if xmsgtype == 1:
                    packedmsg = mt.msg_type1_pack(xmsg)
                fullmsg = headerpacked + packedmsg
                try:
                    clientsocket.send(fullmsg)
                except socket.error as emsg:
                    print(f"socket.error exception: {emsg}")
                send_count += 1
                time.sleep(msg_delay)
        #send_count = 0
    print("Shutting down pyserver thread!")
    pyserver.close()
    serverevent.set()
    
thr1 = threading.Thread(target=server_thread, args=(HOST, PORT, HEADER_FORMAT, messagetype, msgqueue, ))
thr2 = threading.Thread(target=mt.get_message, args=(msgqueue, ))
thr1.start()
thr2.start()

# Main Thread
time.sleep(0.1)
print("Adding default message to queue")
msgqueue.put(msg)
while True:
    try:
        if serverevent.is_set():
            print("Pyserver event is set!")
            break
        time.sleep(1)
    except KeyboardInterrupt:
        serverevent.set()
        break

thr1.join()
thr2.join()

while not msgqueue.empty():
    msgqueue.get()
print(f"msgqueue is empty")

print("EOF")

