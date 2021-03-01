#!/usr/bin/env python
# coding: utf-8

# In[2]:


# "pyclient_01.py <IPv4 or IPv6 addr> <port>"
#
# Simple TCP/IP Client that can be used interact with the UUT running the
# Server. Once connected, the UUT sends telemetry messages to the Client at
# regular intervals.

# Messages are comprised of both a header and a body. The header itself is
# always 12 bytes. It specifies the number of bytes in the body, the type of
# message that the body pertains to, and an incremental message number.
# Currently, the message body can be either text data, numerical data, or
# acknowledgement data.
# In the future, a message footer is planned in which there will be an
# application layer checksum for authenticating the message.

# Currently, the Client can send acknowledgement messages to the Server.
# In the future, the Server will also be able to receive command messages
# from the Client, and the Server will respond to the command.

# The Client socket is run in an additional thread. Another thread is used to
# record telemetry messages sent from the UUT. Queues are used to share
# message data between threads.

# For simplicity's sake, blocking socket reads and timeouts are used.

import sys
import socket
import struct
import threading
import time
import queue
#import crcmod

import messagetools as mt

ARGV = sys.argv[1:]
ARGC = len(ARGV)
#print(f"arg0={ARGV[0]} arg1={ARGV[1]}")
HOST = socket.gethostname() # Default IP Address
PORT = 5001 # Default Port, iPerf
if ((ARGC >= 1) and (ARGV[0] != '-f')):
    if ((ARGC >= 1) and (isinstance(ARGV[0], str))):
        HOST = ARGV[0]
    if ((ARGC >= 2 and isinstance(ARGV[1], str))):
        PORT = int(ARGV[1])
print(f"host={HOST} port={PORT}")
HEADER_STRING = "!III" # No padding like with "!IHI"
BUFFER_SIZE = 32
CONN_ATTEMPTS_MAX = 20 # Temporary
MESSAGES_MAX = 20 # Temporary
RECV_TIMEOUT = 0.5

recv_q = queue.Queue() # Queue for receiving messages
send_q = queue.Queue() # Queue for sending messages
record_q = queue.Queue() # Queue for recording messages
clientevent = threading.Event()

def client_thread(xhost, xport, xheaderformat, xbuffersize, recvq, sendq,
                  recordq):
    '''
    Creates socket, connects to server, and receives and acknowledges
    messages from server.
    Parameters
    ----------
    xhost : IP address
    xport : Port number
    xheaderformat : String describing header struct, e.g. "!4I4d"
    xbuffersize : Number of bytes allocated to buffer
    recvq : Queue containing message tuple that was received
    sendq : Queue that can contain new messages to be sent
    recordq : Queue that can contain new messages to be recorded
    Returns
    -------
    None
    '''
    print("Running pyclient thread...")
    
    pyclient = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    conn_setup = False
    conn_attempts = 0
    total_messages = 0
    client_msgnumber = 1
    
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
        recv_status = mt.recv_message(pyclient, RECV_TIMEOUT,
                                      xheaderformat, xbuffersize,
                                      recvq, recordq)
        if recv_status == -1: # Exception
            break
        elif clientevent.is_set():
            break
        elif recv_status == -2: # Timeout
            #break
            # Send sequence
            if sendq.empty() == False:
                send_status = mt.send_message(pyclient, xheaderformat,
                                              client_msgnumber, sendq)
                client_msgnumber += 1
            continue
        else: # Message received
            total_messages += 1
        
        # ACK sequence
        #pyclient.send(b"ACK") # Acknowledgement
        if recvq.empty() == False:
            #msg_type = xsendqueue.get()
            msg_t = recvq.get()
            print(f"Sending ACK: {msg_t}")
            sendq.put(msg_t)
            send_status = mt.send_message(pyclient, xheaderformat,
                                          client_msgnumber, sendq)
            #print(f"send_status: {send_status}")
            client_msgnumber += 1
    
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
            elif (msg_type == 2) or (msg_type == 50):
                mt.record_csv(dir_name, msg)
        if clientevent.is_set(): # Check after recording message
            break
        time.sleep(0.1)

def input_thread(message_queue):
    '''
    Prompts user to enter new messages
    Enter Q in order to quit program
    New messages can be addeded to message queue
    Parameters
    ----------
    message_queue : Queue that can contain new messages to be sent
    Returns
    -------
    None
    '''
    status = True
    print("Please enter a new message or Q to quit")
    while True:
        status = mt.get_message(message_queue)
        if status == False:
            print("Quitting input_thread...")
            if not clientevent.is_set():
                clientevent.set()
            break
            
thr1 = threading.Thread(target=client_thread, args=(HOST, PORT, HEADER_STRING,
                        BUFFER_SIZE, recv_q, send_q, record_q))
thr2 = threading.Thread(target=input_thread, args=(send_q, ))
thr3 = threading.Thread(target=record_thread, args=(record_q, ))
thr1.start()
thr2.start()
thr3.start()

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
thr3.join()

# Empty each queue
while not recv_q.empty():
    recv_q.get()
print(f"recv_q is empty")

while not send_q.empty():
    send_q.get()
print(f"send_q is empty")

while not record_q.empty():
    record_q.get()
print(f"record_q is empty")

print("EOF")

