#!/usr/bin/env python
# coding: utf-8

# In[2]:


import socket
import struct
import threading
import time
import queue
#import crcmod

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
msg_type = 1 # String message
msg_t = (msg_type, msg) # Message tuple : (message_type, message)

def server_thread(xhost, xport, xheaderformat, xmsgqueue):
    '''
    Creates socket, waits for connection by client, receives and sends messages to clients
    Parameters
    ----------
    xhost : IP address
    xport : Port number
    xheaderformat : String describing header struct, e.g. "!4I4d"
    xmsgqueue : Queue that can contain new messages to be sent
    Returns
    -------
    None
    '''
    print("Running pyserver thread...")
    xmsg_t = (1,"")
    msg_delay = 0.1 # Number of seconds between sending each message
    conn_count = 0
    xmsgnumber = 1 # Initial message number
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as pyserver:
        pyserver.bind((xhost, xport))
        pyserver.listen(5)

        while conn_count < CONN_COUNT_MAX:
            conn_count += 1
            print("Listening for client...")
            clientsocket, clientaddress = pyserver.accept()
            
            clientsocket.settimeout(0.1) # Blocking recv timeout
            
            with clientsocket:
                print(f"Connection from {clientaddress}")
                while True:
                    if serverevent.is_set():
                        clientsocket.close()
                        conn_count = CONN_COUNT_MAX
                        break
                    
                    # Receive sequence
                    try:
                        msg_recv = clientsocket.recv(64)
                        if len(msg_recv) == 0:
                            print("Client closed connection... Stopping...")
                            break
                        else:
                            print(msg_recv)
                    except socket.timeout as emsg:
                        #print(f"RECV socket.timeout exception: {emsg}")
                        msg_recv = b""
                    except socket.error as emsg:
                        print(f"RECV socket.error exception: {emsg}")
                        break
                    
                    # Send sequence
                    send_status = mt.send_message(clientsocket, xheaderformat, xmsgnumber, xmsgqueue)
                    if send_status == -1: # Exception
                        break
                    elif send_status == 1: # Message sent
                        xmsgnumber += 1
                    
                    time.sleep(msg_delay) # Message loop delay
    
    print("Shutting down pyserver thread!")
    #pyserver.close()
    if not serverevent.is_set():
        serverevent.set()

def input_thread(message_queue):
    '''
    Prompts user to enter new messages that will be addeded to message queue
    Parameters
    ----------
    xmsgqueue : Queue that can contain new messages to be sent
    Returns
    -------
    None
    '''
    mt.get_message(message_queue)

thr1 = threading.Thread(target=server_thread, args=(HOST, PORT, HEADER_FORMAT, msg_q, ))
thr2 = threading.Thread(target=input_thread, args=(msg_q, ))
thr1.start()
thr2.start()

# Main Thread
msg_count = 0
while True:
    try:
        if serverevent.is_set():
            print("Pyserver event is set!")
            break
        else:
            time.sleep(4)
            if msg_count < 5:
                msg_count += 1
                msg_q.put(msg_t)
                #msg_q.put((2,(10, 20, 30, 40, 50.12, 60.34, 70.56, 80.78)))
    except KeyboardInterrupt:
        serverevent.set()
        break

thr1.join()
thr2.join()

while not msg_q.empty():
    msg_q.get()
print(f"msg_q is empty")

print("EOF")

