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
BUFFER_SIZE = 32
HEADER_FORMAT = "!IHI"
CONN_ATTEMPTS_MAX = 20 # Temporary
MESSAGES_MAX = 5 # Temporary
RECV_COUNT_MAX = 0x7FFF # Temporary

rec_q = queue.Queue() # Queue for recording messages
clientevent = threading.Event()

def client_thread(xhost, xport, xheaderformat, xheadersize, xbuffersize, xrecqueue):
    '''
    Creates socket, connects to server, and processes messages from server
    Parameters
    ----------
    xhost : IP address
    xport : Port number
    xheaderformat : String describing header struct, e.g. "!4I4d"
    xheadersize : Total number of bytes in header struct
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
    msglen = 0
    msgtype = 0
    msgnumber = 0
    total_messages = 0

    while conn_attempts < CONN_ATTEMPTS_MAX:
        conn_attempts += 1
        try:
            if clientevent.is_set():
                print("Client event is set!")
                break
            pyclient.connect((HOST, PORT))
            conn_setup = True
            print(f"Connected to server!")
            break
        except socket.error as emsg:
            print(f"socket.error exception: {emsg}")
            time.sleep(2)

    while (conn_setup == True) and (total_messages < MESSAGES_MAX):
        new_msg = True
        full_msg = b""
        recv_count = 0
        bytes_read = 0
        
        while recv_count < RECV_COUNT_MAX:
            try:
                msg = pyclient.recv(xbuffersize)
            except socket.error as emsg:
                print(f"socket.error exception: {emsg}")
                total_messages = MESSAGES_MAX
                break
            
            bytes_read += xbuffersize
            recv_count += 1

            if len(msg) == 0:
                print("Message length is 0... Skipping...")
                total_messages += 1
                time.sleep(1)
                break
            elif new_msg:
                new_msg = False
                msgheaderunpacked = struct.unpack(xheaderformat, msg[:xheadersize])
                msglen = msgheaderunpacked[0]
                msgtype = msgheaderunpacked[1]
                msgnumber = msgheaderunpacked[2]
                print(f"msglen is: {msglen}, msgtype is: {msgtype}, msgnumber is: {msgnumber}")
                full_msg = msg[xheadersize:]
            else:
                full_msg += msg
            
            if bytes_read >= msglen:
                if msgtype == 1:
                    text_msg = full_msg.decode("utf-8")
                    print(mt.get_timestamp(text_msg))
                    xrecqueue.put(text_msg)
                elif msgtype == 2:
                    print(struct.unpack("!4I4d", full_msg[:msglen]))
                break
        total_messages += 1
    
    if conn_setup == True:
        print("Shutting down pyclient thread!")
        pyclient.shutdown(1)
        pyclient.close()
        conn_setup = False
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
            mt.record_message(dir_name, mt.get_timestamp(rec_queue.get()))
        
        if clientevent.is_set(): # Check after recording
            break
        
        time.sleep(1)

thr1 = threading.Thread(target=client_thread, args=(HOST, PORT, HEADER_FORMAT, HEADER_SIZE, BUFFER_SIZE, rec_q, ))
thr2 = threading.Thread(target=record_thread, args=(rec_q, ))
thr1.start()
thr2.start()

# Main Thread
while True:
    try:
        if clientevent.is_set():
            print("Client event is set!")
            break
        time.sleep(1)
    except KeyboardInterrupt:
        clientevent.set()
        break

thr1.join()
thr2.join()

while not rec_q.empty():
    rec_q.get()
print(f"rec_q is empty")

print("EOF")

