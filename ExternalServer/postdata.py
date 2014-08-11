import serial # if you have not already done so
import requests

local = False
ser = serial.Serial('/dev/tty.usbserial-DA00STAU', 9600)

while True:
	data = {"data": ser.readline()}
	if(local):
		r = requests.post("http://localhost:5000/post", params=data)
	else:
		r = requests.post("http://cubesatags.herokuapp.com/post", params=data)