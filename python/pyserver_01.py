#!/usr/bin/env python
# coding: utf-8

# In[12]:


# Simple TCP/IP Server that can be used to mimic the UUT where telemetry
# messages are sent to the Client connection at regular intervals.

# Messages are comprised of both a header and a body. The header itself is
# always 12 bytes. It specifies the number of bytes in the body, the type of
# message that the body pertains to, and an incremental message number.
# Currently, the message body can be either text data, numerical data, or
# acknowledgement data.
# In the future, a message footer is planned in which there will be an
# application layer checksum for authenticating the message.

# Currently, the Server can receive acknowledgement messages from the Client.
# In the future, the Server will also be able to receive command messages
# from the Client, and the Server will respond to the command.

# The Server socket is run in an additional thread. Another thread is used to
# capture the keyboard input for specifying a custom text message. Queues
# are used to share message data between threads.

# For simplicity's sake, blocking socket reads and timeouts are used.

# In the future, the Server will be able to record its activities and its
# interaction with the Client.

import socket
import struct
import threading
import time
import queue
#import crcmod

import messagetools as mt

HOST = socket.gethostname()
PORT = 5001 # iPerf
HEADER_SIZE = 12
HEADER_FORMAT = "!III" # No padding like with "!IHI"
CONN_COUNT_MAX = 1 # Temporary
SEND_COUNT_MAX = 5 # Temporary

recv_q = queue.Queue() # Queue for receiving messages
send_q = queue.Queue() # Queue for sending messages
record_q = queue.Queue() # Queue for recording messages
serverevent = threading.Event() # Event to gracefully stop threads

# Specify a default text message for transmission
#msg = "Server Message: 0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!\"#$%&\'()*+,-./:;<=>?@[\\]^_`{|}~ \t\n\r\x0b\x0c"
msg = "Server Message: 0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"
msg_type = 1 # String message
msg_t = (msg_type, msg) # Message tuple : (message_type, message)

def server_thread(xhost, xport, xheaderformat, recvq, sendq, recordq):
    '''
    Creates socket, waits for connection by client, receives and sends
    messages to clients
    Parameters
    ----------
    xhost : IP address
    xport : Port number
    xheaderformat : String describing header struct, e.g. "!4I4d"
    recvq : Queue containing message tuple that was received
    sendq : Queue that can contain new messages to be sent
    recordq : Queue that can contain new messages to be recorded
    Returns
    -------
    None
    '''
    print("Running pyserver thread...")
    xmsg_t = (1,"")
    msg_delay = 0.1 # Number of seconds between sending each message
    conn_count = 0
    xmsgnumber = 1 # Initial message number
    
    RECV_TIMEOUT = 0.5
    xbuffersize = 64
    
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
                        #msg_recv = clientsocket.recv(64)
                        recv_status = mt.recv_message(clientsocket, 0.2,
                                                  xheaderformat, xbuffersize,
                                                  recvq, recordq)
                        #if len(msg_recv) == 0:
                        if recv_status == -3:
                            print("Client closed connection... Stopping...")
                            break
                        else:
                            #print(msg_recv)
                            #print(f"recv_status: {recv_status}")
                            if recvq.empty() == False:
                                msg_t = recvq.get()
                                print(f"Received ACK: {msg_t}")
                    except socket.timeout as emsg:
                        #print(f"RECV socket.timeout exception: {emsg}")
                        msg_recv = b""
                    except socket.error as emsg:
                        print(f"RECV socket.error exception: {emsg}")
                        break
                    
                    # Send sequence
                    send_status = mt.send_message(clientsocket, xheaderformat,
                                                  xmsgnumber, sendq)
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
            if not serverevent.is_set():
                serverevent.set()
            break

thr1 = threading.Thread(target=server_thread, args=(HOST, PORT, HEADER_FORMAT,
                                                    recv_q, send_q, record_q))
thr2 = threading.Thread(target=input_thread, args=(send_q, ))
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
            time.sleep(5)
            if msg_count < 5:
                msg_count += 1
                # Send default text message
                # send_q.put(msg_t)
                
                # Send default numerical message
                #send_q.put((2,(10, 20, 30, 40, 50.12, 60.34, 70.56, 80.78)))
    except KeyboardInterrupt:
        serverevent.set()
        break

thr1.join()
thr2.join()

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

