#!/usr/bin/env python
# coding: utf-8

# In[2]:


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

recqueue = queue.Queue()
clientevent = threading.Event()

def client_thread(xhost, xport, xheaderformat, xheadersize, xbuffersize, xrecqueue):
    '''
    Creates socket, connects to server, and processes messages from server
    Parameters
    ----------
    xhost : IP address
    xport : Port number
    xheaderformat : String describing header struct, e.g. "!2I2F"
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

    msglen = 0
    msgtype = 0
    msgnumber = 0
    total_messages = 0

    while (conn_setup == True) and (total_messages < MESSAGES_MAX):
        new_msg = True
        full_msg = ''
        recv_count = 0
        bytes_read = 0
        while recv_count < RECV_COUNT_MAX:

            try:
                msg = pyclient.recv(xbuffersize)
            except socket.error as emsg:
                print(f"socket.error exception: {emsg}")
                recv_count = RECV_COUNT_MAX
                total_messages = MESSAGES_MAX
            
            recv_count += 1

            if len(msg) == 0:
                print("Skipping recv iterations...")
                total_messages += 1
                time.sleep(1)
                break

            bytes_read += xbuffersize

            if new_msg:
                new_msg = False
                msgheaderunpacked = struct.unpack(xheaderformat, msg[:xheadersize])
                msglen = msgheaderunpacked[0]
                msgtype = msgheaderunpacked[1]
                msgnumber = msgheaderunpacked[2]
                print(f"msglen is: {msglen}, msgtype is: {msgtype}, msgnumber is: {msgnumber}")

                if msgtype == 1:
                    full_msg += msg[xheadersize:].decode("utf-8")
            else:
                if msgtype == 1:
                    full_msg += msg.decode("utf-8")

            if bytes_read >= msglen:
                #mt.printstamp(full_msg)
                print(mt.get_timestamp(full_msg))
                xrecqueue.put(full_msg)
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
    Records received messages that are relayed with a queue
    Parameters
    ----------
    rec_queue : Queue that can contain new messages to be recorded
    Returns
    -------
    None
    '''
    rec_name = mt.get_datetime_name()
    #while clientevent.is_set() == False:
    while True:
        if rec_queue.empty() == False:
            mt.record_message(rec_name, mt.get_timestamp(rec_queue.get()))
        
        if clientevent.is_set(): # Check after recording
            break
        
        time.sleep(1)

thr1 = threading.Thread(target=client_thread, args=(HOST, PORT, HEADER_FORMAT, HEADER_SIZE, BUFFER_SIZE, recqueue, ))
thr2 = threading.Thread(target=record_thread, args=(recqueue, ))
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

while not recqueue.empty():
    recqueue.get()
print(f"recqueue is empty")

print("EOF")

