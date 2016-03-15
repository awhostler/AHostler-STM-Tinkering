#!/usr/bin/python

import serial
import time

packet = bytearray(['A','B',0,1,0,6])

ser = serial.Serial('/dev/cu.usbserial-AI02L70U', 115200, timeout=0)

ser.write(packet)

#time.sleep(3)

#print ser.read(1024)

ser.close()


