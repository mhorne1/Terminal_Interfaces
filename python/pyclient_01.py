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
HEADER_STRING = "!IHI"
BUFFER_SIZE = 32
CONN_ATTEMPTS_MAX = 20 # Temporary
MESSAGES_MAX = 10 # Temporary
RECV_TIMEOUT = 0.5

record_q = queue.Queue() # Queue for recording messages
clientevent = threading.Event()

def client_thread(xhost, xport, xheaderformat, xbuffersize, xrecqueue):
    '''
    Creates socket, connects to server, and processes messages from server
    Parameters
    ----------
    xhost : IP address
    xport : Port number
    xheaderformat : String describing header struct, e.g. "!4I4d"
    xbuffersize : Number of bytes allocated to buffer
    xrecqueue : Queue that can contain new messages to be recorded
    Returns
    -------
    None
    '''
    print("Running pyclient thread...")
    
    pyclient = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    conn_setup = False
    conn_attempts = 0
    total_messages = 0
    
    while conn_attempts < CONN_ATTEMPTS_MAX:
        conn_attempts += 1
        if clientevent.is_set():
            print("Pyclient event is set!")
            break
        else:
            try:
                pyclient.connect((HOST, PORT))
                conn_setup = True
                print(f"Connected to server!")
                break
            except socket.error as emsg:
                conn_setup = False
                print(f"socket.error exception: {emsg}")
                time.sleep(2)
    
    while (conn_setup == True) and (total_messages < MESSAGES_MAX):
        # Recv sequence
        recv_status = mt.recv_message(pyclient, RECV_TIMEOUT, xheaderformat, xbuffersize, xrecqueue)
        if recv_status == -1: # Exception
            break
        elif recv_status == -2: # Timeout
            break
        else: # Message received
            total_messages += 1
        # Send sequence
        pyclient.send(b"ACK") # Acknowledgement
    
    if conn_setup == True:
        print("Shutting down pyclient thread!")
        #pyclient.shutdown(1)
        pyclient.close()
        conn_setup = False
        if not clientevent.is_set():
            clientevent.set()

def record_thread(rec_queue):
    '''
    Records messages that are retrieved from a queue
    Parameters
    ----------
    rec_queue : Queue that can contain new messages to be recorded
    Returns
    -------
    None
    '''
    dir_name = mt.get_datetime_name()
    while True:
        if rec_queue.empty() == False:
            msg_t = rec_queue.get()
            msg_type = msg_t[0]
            msg = msg_t[1]
            if msg_type == 1:
                mt.record_text(dir_name, mt.get_timestamp(msg))
            elif msg_type == 2:
                mt.record_csv(dir_name, msg)
        if clientevent.is_set(): # Check after recording message
            break
        time.sleep(0.1)

thr1 = threading.Thread(target=client_thread, args=(HOST, PORT, HEADER_STRING, BUFFER_SIZE, record_q, ))
thr2 = threading.Thread(target=record_thread, args=(record_q, ))
thr1.start()
thr2.start()

# Main Thread
while True:
    try:
        if clientevent.is_set():
            print("Client event is set!")
            break
        time.sleep(0.1)
    except KeyboardInterrupt:
        clientevent.set()
        break

thr1.join()
thr2.join()

while not record_q.empty():
    record_q.get()
print(f"record_q is empty")

print("EOF")

