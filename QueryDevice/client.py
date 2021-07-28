# Import socket module 
import socket
import sys
import os
import time
s = socket.socket()           
try:  
# Create a socket object 
             
  
# Define the port on which you want to connect 
    port = 12347                
  
# connect to the server on local computer 
    s.connect(('127.0.0.1', port)) 
  
# receive data from the server 
    while True:

        print (s.recv(1024).decode() )
        
# close the connection 
except KeyboardInterrupt:
    s.close()  
    try:
        sys.exit(0)
    except SystemExit:
        os._exit(0)  