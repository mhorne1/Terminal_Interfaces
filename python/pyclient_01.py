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
MESSAGES_MAX = 10 # Temporary
RECV_COUNT_MAX = 0x7FFF # Temporary
RECV_TIMEOUT = 0.5
RECV_TIMEOUT_MAX = 10*RECV_TIMEOUT
RECV_TIMEOUT_COUNT = 0

record_q = queue.Queue() # Queue for recording messages
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
            conn_setup = False
            print(f"socket.error exception: {emsg}")
            time.sleep(2)

    pyclient.settimeout(0.5) # Blocking recv timeout
    recv_timeout_count = 0 # Total number of timeouts
    
    while (conn_setup == True) and (total_messages < MESSAGES_MAX):
        new_msg = True
        full_msg = b""
        recv_count = 0
        bytes_read = 0
        
        total_messages += 1
        
        while recv_count < RECV_COUNT_MAX:
            try:
                msg = pyclient.recv(xbuffersize)
            except socket.timeout as emsg:
                #print(f"socket.timeout exception: {emsg}")
                recv_timeout_count += RECV_TIMEOUT
                if recv_timeout_count < RECV_TIMEOUT_MAX:
                    continue
                else:
                    print("RECV timeout... Stopping...")
                    total_messages = MESSAGES_MAX
                    break
            except socket.error as emsg:
                print(f"socket.error exception: {emsg}")
                total_messages = MESSAGES_MAX
                break
            
            bytes_read += xbuffersize
            recv_count += 1
            
            if len(msg) == 0:
                print("Server closed connection... Stopping...")
                break
            elif new_msg:
                recv_timeout_count = 0
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
                    xrecqueue.put((1,text_msg))
                elif msgtype == 2:
                    msgtuple = struct.unpack("!4I4d", full_msg[:msglen])
                    print(msgtuple)
                    xrecqueue.put((2,msgtuple))
                # Send sequence
                pyclient.send(b"ACK") # Acknowledgement
                break
    
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

thr1 = threading.Thread(target=client_thread, args=(HOST, PORT, HEADER_FORMAT, HEADER_SIZE, BUFFER_SIZE, record_q, ))
thr2 = threading.Thread(target=record_thread, args=(record_q, ))
thr1.start()
thr2.start()

# Main Thread
while True:
    try:
        if clientevent.is_set():
            print("Pyclient event is set!")
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

