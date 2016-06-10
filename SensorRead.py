import serial
import os 


#class SensorRead:

#	def __init__ ():
#os.system('sudo chmod 777 /dev/ttyACM0')
#ser = serial.Serial('/dev/ttyACM0',9600)
ser = serial.Serial('/dev/ttyACM1', timeout=None, baudrate=9600, xonxoff=False, rtscts=False, dsrdtr=False)
while True:
	#print ser.readline()
	readings = ser.readline()
#        [int(s) for s in re.findall(r'\b\d+\b', readings)]

	l = []
	for t in readings.split():
    		try:
       		 l.append(float(t))
    		except ValueError:
        	 pass
	print l
#return l

