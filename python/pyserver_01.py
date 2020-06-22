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

msg_q = queue.Queue() # Queue for sending messages
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
    xheaderformat : String describing header struct, e.g. "!4I4d"
    xmsgtype : Number indicating type of message
    xmsgqueue : Queue that can contain new messages to be sent
    Returns
    -------
    None
    '''
    print("Running pyserver thread...")
    xmsg = ""
    msg_delay = 3 # Number of seconds between sending each message
    conn_count = 0
    xmsgnumber = 1 # Initial message number
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as pyserver:
        pyserver.bind((xhost, xport))
        pyserver.listen(5)

        while conn_count < CONN_COUNT_MAX:
            conn_count += 1
            print("Listening for client...")
            clientsocket, clientaddress = pyserver.accept()
            with clientsocket:
                print(f"Connection from {clientaddress}")
                while True:
                    fullmsg = b""
                    if serverevent.is_set():
                        clientsocket.close()
                        conn_count = CONN_COUNT_MAX
                        break
                    elif xmsgqueue.empty():
                        #time.sleep(1)
                        #continue
                        break
                    elif not xmsgqueue.empty():
                        #print("Thread getting message from queue")
                        xmsg = xmsgqueue.get()
                        packedmsg = mt.msg_packer(mt.msg_dict, xmsgtype, xmsg)
                        xmsglen = len(packedmsg)
                        packedheader = struct.pack(xheaderformat, xmsglen, xmsgtype, xmsgnumber)
                        xmsgnumber += 1
                        fullmsg = packedheader + packedmsg
                        try:
                            clientsocket.send(fullmsg)
                        except socket.error as emsg:
                            print(f"socket.error exception: {emsg}")
                            break
                        time.sleep(msg_delay)
    print("Shutting down pyserver thread!")
    #pyserver.close()
    if not serverevent.is_set():
        serverevent.set()

def input_thread(xmsgqueue):
    '''
    Prompts user to enter new messages that will be addeded to message queue
    Parameters
    ----------
    xmsgqueue : Queue that can contain new messages to be sent
    Returns
    -------
    None
    '''
    mt.get_message(xmsgqueue)

thr1 = threading.Thread(target=server_thread, args=(HOST, PORT, HEADER_FORMAT, messagetype, msg_q, ))
thr2 = threading.Thread(target=input_thread, args=(msg_q, ))
thr1.start()
thr2.start()

# Main Thread
for _ in range(5):
    msg_q.put(msg)
while True:
    try:
        if serverevent.is_set():
            print("Pyserver event is set!")
            break
        else:
            time.sleep(1)
    except KeyboardInterrupt:
        serverevent.set()
        break

thr1.join()
thr2.join()

while not msg_q.empty():
    msg_q.get()
print(f"msg_q is empty")

print("EOF")

