#!/usr/bin/env python
# coding: utf-8

# In[11]:


import datetime
import queue
import time

def get_message(myqueue):
    '''
    Requests user input and then places message in to queue 
    ----------
    myqueue : Queue that can receive new messages
    Returns
    -------
    None
    '''
    time.sleep(1) # Delay so that printing does not overlap with other threads
    mymsg = input("Please enter a new message...")
    print(f"Adding message {mymsg} to queue!")
    myqueue.put(mymsg)

def get_timestamp( mytime ):
    mystring = ""
    mystring = str(mytime.hour).zfill(2) \
    + ":" \
    + str(mytime.minute).zfill(2) \
    + ":" \
    + str(mytime.second).zfill(2) \
    + "." + str(mytime.microsecond).zfill(6)
    return mystring

def printstamp( message ):
    mystring = ""
    mytime = datetime.datetime.now()
    mystring = str(mytime.hour).zfill(2) \
    + ":" \
    + str(mytime.minute).zfill(2) \
    + ":" \
    + str(mytime.second).zfill(2) \
    + "." + str(mytime.microsecond).zfill(6)
    print(f'{mystring} - {message}')

	