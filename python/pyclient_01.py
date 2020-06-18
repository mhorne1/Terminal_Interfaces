#!/usr/bin/env python
# coding: utf-8

# In[1]:


import socket
import struct
import threading
import time
#import queue

import messagetools as mt

HOST = socket.gethostname()
PORT = 5678
HEADER_SIZE = 10
BUFFER_SIZE = 32
HEADER_FORMAT = "!IHI"
CONN_ATTEMPTS_MAX = 20 # Temporary
MESSAGES_MAX = 5 # Temporary
RECV_COUNT_MAX = 0x7FFF # Temporary

#msgqueue = queue.Queue()

def pyclientthread(xhost, xport, xheaderformat, xheadersize, xbuffersize):
    '''
    Creates socket, connects to server, and processes messages from server
    Parameters
    ----------
    xhost : IP address
    xport : Port number
    xheaderformat : String describing header struct, e.g. "!2I2F"
    xheadersize : Total number of bytes in header struct
    xbuffersize : Number of bytes allocated to buffer
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
            pyclient.connect((HOST, PORT))
            conn_setup = True
            print(f"Connected to server!")
            break
        except socket.error as emsg:
            print(f"socket.error exception: {emsg}")
            time.sleep(2)
        except TypeError as emsg:
            print(f"TypeError exception: {emsg}")

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
            
            recv_count += 1

            if len(msg) == 0:
                recv_count += 1
                print("Skipping...")
                continue

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
                mt.printstamp(full_msg)
                break

        total_messages += 1

    if conn_setup == True:
        print("Shutting down pyclient thread!")
        pyclient.shutdown(1)
        pyclient.close()
        conn_setup = False
    
    
thr1 = threading.Thread(target=pyclientthread, args=(HOST, PORT, HEADER_FORMAT, HEADER_SIZE, BUFFER_SIZE, ))
#thr2 = threading.Thread(target=mt.get_message, args=(msg, msgqueue, ))
thr1.start()
#thr2.start()
thr1.join()
#thr2.join()

print("EOF")

