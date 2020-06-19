#!/usr/bin/env python
# coding: utf-8

# In[11]:


import datetime
import queue
import time

def get_message( myqueue ):
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

def check_message( myqueue ):
    '''
    Returns message from queue that is not empty
    Parameters
    ----------
    myqueue : Message queue
    Returns
    -------
    Message
    '''
    if myqueue.empty() == False:
        print("Thread getting message from queue")
        return myqueue.get()
    
def get_timestamp( mytime ):
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

def printstamp( message ):
    '''
    Prints formatted timestamp along with message string 
    Parameters
    ----------
    message : String
    Returns
    -------
    None
    '''
    mystring = ""
    mytime = datetime.datetime.now()
    mystring = str(mytime.hour).zfill(2) \
    + ":" \
    + str(mytime.minute).zfill(2) \
    + ":" \
    + str(mytime.second).zfill(2) \
    + "." + str(mytime.microsecond).zfill(6)
    print(f"{mystring} - {message}")

	