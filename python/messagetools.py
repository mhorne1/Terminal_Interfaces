#!/usr/bin/env python
# coding: utf-8

# In[11]:


import datetime
import queue
import time
import os

def get_message(myqueue):
    '''
    Requests text input and then places message in to queue 
    ----------
    myqueue : Queue that can receive new messages
    Returns
    -------
    None
    '''
    time.sleep(0.25) # Delay to avoid overlapping printing from from other threads
    mymsg = input("Please enter a new message...")
    print(f"Adding message {mymsg} to queue!")
    myqueue.put(mymsg)

def msg_packer(message_type, message):
    '''
    Uses message_type to determine how message is packed
    Parameters
    ----------
    message_type : Specific type of message
    message : Text string or tuple
    Returns
    -------
    Bytes object of message
    '''
    if message_type == 1:
        return msg_type1_pack(message)
    else:
        return b""
    
def msg_type1_pack(message):
    '''
    Encodes type 1 messages
    Parameters
    ----------
    message : Text string
    Returns
    -------
    Byte encoded text string
    '''
    return message.encode("utf-8")

def get_timestamp(mytime):
    '''
    Returns string formatted from datetime object
    Parameters
    ----------
    mytime : Datetime object
    Returns
    -------
    Formatted time string
    '''
    mystring = ""
    mystring = str(mytime.hour).zfill(2) \
    + ":" \
    + str(mytime.minute).zfill(2) \
    + ":" \
    + str(mytime.second).zfill(2) \
    + "." + str(mytime.microsecond).zfill(6)
    return mystring

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

def record_message(name, message):
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
