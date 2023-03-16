import serial
from time import sleep 
ser = serial.serial("/dev/ttys0",9600)
while(1):
	data = 'Hello world'
	print(data)
	ser.write(data)
	sleep(1)

