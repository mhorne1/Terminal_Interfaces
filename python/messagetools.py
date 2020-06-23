#!/usr/bin/env python
# coding: utf-8

# In[11]:


import datetime
import queue
import time
import os
import struct

def get_message(myqueue):
    '''
    Requests text input and then places message in to queue 
    ----------
    myqueue : Queue that can receive new messages
    Returns
    -------
    None
    '''
    time.sleep(0.25) # Slight delay to avoid overlapping printing from from other threads
    new_msg = input("Please enter a new message...")
    print(f"Adding message '{new_msg}' to queue!")
    myqueue.put((1,new_msg))

def msg_packer(message_dict, message_type, *message_args):
    '''
    Uses message_type to determine how message is packed
    Parameters
    ----------
    message_dict : Keys are the message types and values are the packing functions
    message_type : Specific type of message
    message_args : Text string or tuple
    Returns
    -------
    Bytes object of message
    '''
    if message_type in message_dict:
        return message_dict[message_type](*message_args)
    else:
        return b""
    
def msg_type1_pack(message_text):
    '''
    Encodes type 1 messages
    Parameters
    ----------
    message : Text string
    Returns
    -------
    Byte encoded text string
    '''
    return message_text.encode("utf-8")

def msg_type2_pack(message_tuple):
    '''
    Encodes type 2 messages
    Parameters
    ----------
    message_tuple : 4 uint32, 4 double
    Returns
    -------
    Network (big-endian) byte encoded tuple
    '''
    STRUCT_FORMAT = "!4I4d"
    return struct.pack(STRUCT_FORMAT, *message_tuple)

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
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")+"_rec.txt"

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
    with open(rec_dir+name, 'a+') as my_file:
        my_file.write(message + '\n')

def send_message(send_socket, format_string, msg_number, msg_queue):
    '''
    Sends header and message to socket. Header specifies message length, message type and message number
    Parameters
    ----------
    send_socket : Socket that will receive message
    format_string : Formatted header string
    msg_number : Message number for header
    msg_queue : Queue containing message tuple; message type and message
    Returns
    -------
    Integer describing outcome of sending message; 1 sent, 0, nothing to send, -1 exception
    '''
    if not msg_queue.empty():
        full_msg = b""
        msg_t = msg_queue.get()
        msg_type = msg_t[0]
        msg = msg_t[1] 
        packedmsg = msg_packer(msg_dict, msg_type, msg)
        msglen = len(packedmsg)
        packedheader = struct.pack(format_string, msglen, msg_type, msg_number)
        #msg_number += 1
        full_msg = packedheader + packedmsg
        try:
            print(f"Sending message #{msg_number}")
            send_socket.send(full_msg)
            return 1 # Message sent
        except socket.error as emsg:
            print(f"SEND socket.error exception: {emsg}")
            return -1 # Exception
    else:
        return 0 # Empty msg_queue

msg_dict = { # Dictionary containing message types and corresponding functions
    1 : msg_type1_pack,
    2 : msg_type2_pack,
}