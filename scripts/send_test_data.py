#!/usr/bin/env python

import socket
import sys
import time

files = []
address = '127.0.0.1'
port = 5602
chunk = 256
sleep = 0.1

for arg in sys.argv:
  if arg.startswith('address='):
    address = arg.split('=',1)[1]
  elif arg.startswith('port='):
    port = int(arg.split('=',1)[1])
  elif arg.startswith('chunk='):
    chunk = int(arg.split('=',1)[1])
  elif arg.startswith('sleep='):
    sleep = float(arg.split('=',1)[1])
  else:
    files.append(arg)

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

for f in files:
  infile = open(f,'rb')
  while True:
    data = infile.read(chunk)
    if len(data):
      s.sendto(data,(address,port))
    if len(data) < chunk:
      break
    time.sleep(sleep)
  
  

