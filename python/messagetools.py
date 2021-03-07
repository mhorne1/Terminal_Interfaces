#!/usr/bin/env python
# coding: utf-8

# In[11]:


import datetime
import queue
import time
import os
import struct
import socket
from enum import Enum
#import crcmod

class Msgs(Enum):
    '''
    Enumeration of pre-defined message types
    '''
    text = 1
    num = 2
    ack = 3
    omnibus = 50

def get_message(myqueue):
    '''
    Requests text input and then places message in to queue 
    ----------
    myqueue : Queue that can receive new messages
    Returns
    -------
    status : False in order to quit
    '''
    status = True
    #time.sleep(0.25) # Slight delay to avoid overlapping printing from from other threads
    #new_msg = input("Please enter a new message...")
    new_msg = input()
    #print(f"Adding message '{new_msg}' to queue!")
    if new_msg == "":
        return status
    elif new_msg[0].isnumeric():
        if int(new_msg[0]) == Msgs.text.value:
            print(f"Adding message '{new_msg[2:]}' to queue!")
            myqueue.put((Msgs.text.value,new_msg[2:]))
            status = True
    elif new_msg[0] == 'Q' or new_msg[0] == 'q':
        status = False
    return status
    
def msg_packer(message_dict, message_type, message_length, message_pack, *message_args):
    '''
    Uses message_type to determine how message is packed
    Parameters
    ----------
    message_dict : Keys are the message types and values are the packing functions
    message_type : Specific type of message
    message_length: Number of bytes that comprise the message
    message_args : Text string or tuple
    Returns
    -------
    Bytes object of message
    '''
    if message_type in message_dict:
        return message_dict[message_type](message_length, message_pack, *message_args)
    else:
        return b""
    
def msg_type1_pack(length, pack_message, message):
    '''
    Encodes or decodes type 1 messages
    Parameters
    ----------
    length: Number of bytes that comprise the message
    pack_message : True or False
    message : Text string, or byte encoded text string
    Returns
    -------
    Byte encoded text string, or text string
    '''
    if (length != -1) and (length != len(message)):
        print(f"Specified message length {length} incompatible with " + \
        f"{struct.calcsize(STRUCT_FORMAT)} for message type 1")
        return b""
    
    if pack_message == True:
        return message.encode("utf-8")
    else:
        return message.decode("utf-8")

def msg_type2_pack(length, pack_message, message):
    '''
    Packs or unpacks type 2 messages
    Parameters
    ----------
    length: Number of bytes that comprise the message
    pack_message : True or False
    message : Tuple (4 uint32, 4 float), or byte encoded tuple
    Returns
    -------
    Network (big-endian) byte encoded tuple, or tuple
    '''
    STRUCT_FORMAT = "!4I4f"
    if (length != -1) and (length != struct.calcsize(STRUCT_FORMAT)):
        print(f"Specified message length {length} incompatible with " + \
        f"{struct.calcsize(STRUCT_FORMAT)} for message type 2")
        return b""
    
    if pack_message == True:
        return struct.pack(STRUCT_FORMAT, *message)
    else:
        return struct.unpack(STRUCT_FORMAT, message)

def msg_type3_pack(length, pack_message, message):
    '''
    Packs or unpacks type 3 acknowledge messages
    Parameters
    ----------
    length: Number of bytes that comprise the message
    pack_message : True or False
    message : Tuple (2 uint32, 1 int32, 1 float32), or byte encoded tuple
    Returns
    -------
    Network (big-endian) byte encoded tuple, or tuple
    '''
    STRUCT_FORMAT = "!2I1i1f"
    if (length != -1) and (length != struct.calcsize(STRUCT_FORMAT)):
        print(f"Specified message length {length} incompatible with " + \
        f"{struct.calcsize(STRUCT_FORMAT)} for message type 3")
        return b""
    
    if pack_message == True:
        return struct.pack(STRUCT_FORMAT, *message)
    else:
        return struct.unpack(STRUCT_FORMAT, message)

def msg_omnibus_pack(length, pack_message, message):
    '''
    Packs or unpacks omnibus telemetry messages
    Parameters
    ----------
    length: Number of bytes that comprise the message
    pack_message : True or False
    message : Tuple (1 uint64, 5 uint32...), or byte encoded tuple
    Returns
    -------
    Network (big-endian) byte encoded tuple, or tuple
    '''
    STRUCT_FORMAT = "!1Q5I2h4b7I"
    if (length != -1) and (length != struct.calcsize(STRUCT_FORMAT)):
        print(f"Specified message length {length} incompatible with " + \
        f"{struct.calcsize(STRUCT_FORMAT)} for message type Omnibus")
        return b""
    
    if pack_message == True:
        return struct.pack(STRUCT_FORMAT, *message)
    else:
        return struct.unpack(STRUCT_FORMAT, message)
    
def get_datetime_name():
    '''
    Returns string with formatted timestamp and message string
    Parameters
    ----------
    None
    Returns
    -------
    String
    '''
    #return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")+"_rec.txt"
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")+"_rec"

def get_timestamp(message):
    '''
    Returns string with formatted timestamp and message string 
    Parameters
    ----------
    message : String
    Returns
    -------
    String
    '''
    mystring = ""
    mytime = datetime.datetime.now()
    mystring = str(mytime.hour).zfill(2) \
    + ":" \
    + str(mytime.minute).zfill(2) \
    + ":" \
    + str(mytime.second).zfill(2) \
    + "." + str(mytime.microsecond).zfill(6)
    return mystring + " - " + message

def record_text(name, message):
    '''
    Writes message string to file with specified name
    Parameters
    ----------
    name : String
    message : String
    Returns
    -------
    None
    '''
    rec_dir = 'Records/'
    if not(os.path.isdir(rec_dir)):
        os.mkdir(rec_dir)
    with open(rec_dir+name+'.txt', 'a+') as my_file:
        my_file.write(message + '\n')

def record_csv(name, message):
    '''
    Writes message tuple to file with specified name
    Parameters
    ----------
    name : String
    message : Data contained in message tuple
    Returns
    -------
    None
    '''
    rec_dir = 'Records/'
    data_list = []
    if not(os.path.isdir(rec_dir)):
        os.mkdir(rec_dir)
    with open(rec_dir+name+'.csv', 'a+') as my_file:
        for _ in message:
            data_list.append(str(_))
        csv_line = ','.join(data_list) + '\n'
        my_file.write(csv_line)

def send_message(send_socket, header_format, msg_number, msg_queue):
    '''
    Sends header and message to socket. Header specifies message length, message type and message number
    Parameters
    ----------
    send_socket : Socket that will send message
    header_format : Formatted header string
    msg_number : Message number for header
    msg_queue : Queue containing message tuple; message type and message
    Returns
    -------
    Integer describing outcome of sending message; 1 sent, 0, nothing to send, -1 exception
    '''
    status = 0 # Empty msg_queue
    if not msg_queue.empty():
        full_msg = b""
        msg_tup = msg_queue.get()
        msg_type = msg_tup[0]
        msg = msg_tup[1]
        msg_len = -1 # Signify that message is being packed and that length is currently unknown
        msg_pack = True # Pack (True) or Unpack (False) message
        packedmsg = msg_packer(msg_dict, msg_type, msg_len, msg_pack, msg)
        msg_len = len(packedmsg)
        packedheader = struct.pack(header_format, msg_len, msg_type, msg_number)
        #msg_number += 1
        full_msg = packedheader + packedmsg
        try:
            #print(f"Sending message #{msg_number}, msg_type: {msg_type}, msg_len: {msg_len}")
            print(f"SEND: msg_len = {msg_len}, msg_type = {msg_type}, msg_number = {msg_number}")
            send_socket.send(full_msg)
            status = 1 # Message sent
        except socket.error as emsg:
            print(f"SEND socket.error exception: {emsg}")
            status = -1 # Exception
        #return status
    #else:
        #return 0 # Empty msg_queue
    return status

def recv_message(recv_socket, recv_timeout, header_format, buffer_size, recv_queue, record_queue):
    '''
    Receives packed messages from socket, unpacks them, creates message tuple, and places them in recording queue
    Parameters
    ----------
    recv_socket : Socket that will receive message
    recv_timeout: Time period that socket will wait to receive messages, in seconds
    header_format : Formatted header string
    buffer_size : Number of bytes that socket will receive
    recv_queue : Queue containing message tuple that was received
    record_queue :  Queue containing message tuple that will be recorded
    Returns
    -------
    Integer describing outcome of receiving message; 1 sent, 0, nothing, -1 exception, -2 timeout, -3 closed connection
    '''
    recv_timeout_count = 0
    recv_timeout_max = 10*recv_timeout
    header_size = struct.calcsize(header_format)
    new_msg = True
    full_msg = b""
    bytes_read = 0
    status = 0 # Default
    
    recv_socket.settimeout(recv_timeout) # Blocking recv timeout

    while True:
        try:
            msg = recv_socket.recv(buffer_size)
        except socket.timeout as emsg:
            #print(f"socket.timeout exception: {emsg}")
            recv_timeout_count += recv_timeout
            if recv_timeout_count < recv_timeout_max:
                continue
            else:
                #print("RECV timeout...")
                #total_messages = MESSAGES_MAX
                status = -2 # Timeout
                break
        except socket.error as emsg:
            print(f"socket.error exception: {emsg}")
            #total_messages = MESSAGES_MAX
            status = -1 # Exception
            break

        if len(msg) == 0:
            print("Peer closed connection... Stopping...")
            status = -3 # Closed connection
            break
        elif new_msg:
            recv_timeout_count = 0
            new_msg = False
            msgheaderunpacked = struct.unpack(header_format, msg[:header_size])
            msglen = msgheaderunpacked[0]
            msgtype = msgheaderunpacked[1]
            msgnumber = msgheaderunpacked[2]
            print(f"RECV: msglen = {msglen}, msgtype = {msgtype}, msgnumber = {msgnumber}")
            full_msg = msg[header_size:]
            bytes_read = buffer_size - header_size
        else:
            full_msg += msg
            bytes_read += buffer_size
        
        if bytes_read >= msglen:
            msgpack = False # Pack (True) or Unpack (False) message
            unpacked_message = msg_packer(msg_dict, msgtype, msglen, msgpack, full_msg[:msglen])
            print(f"RECV: {unpacked_message}")
            if msgtype == Msgs.ack.value: # Receive ACK
                recv_queue.put(unpacked_message)
            else: # Send ACK
                recv_queue.put((Msgs.ack.value,(msglen,msgtype,msgnumber,0)))
            record_queue.put((msgtype,unpacked_message))
            status = 1
            break
    return status

msg_dict = { # Dispatch table containing message types and corresponding functions
    Msgs.text.value : msg_type1_pack,
    Msgs.num.value : msg_type2_pack,
    Msgs.ack.value : msg_type3_pack,
    Msgs.omnibus.value : msg_omnibus_pack,
}