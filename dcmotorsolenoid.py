import logging
import threading
import time
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

#GPIO.setup(32, GPIO.OUT) #Servo Motor
#s = GPIO.PWM(32,50)
#s.start(8.5)

GPIO.setup(33,GPIO.OUT) #DC Motor
p = GPIO.PWM(33,200) #GPIO13 as PWM output with 200 Hz frequency
p.start(0)

GPIO.setup(12,GPIO.OUT) #Solenoid Servo
ss = GPIO.PWM(12,50)
ss.start(7.5)

def servo():
	try:
		time.sleep(2)
		s.ChangeDutyCycle(12.5)
	except: 
		s.stop()
		GPIO.cleanup()

def dcmotor():
	try:
		p.ChangeDutyCycle(100)
	except KeyboardInterrupt:
		p.stop()
		GPIO.cleanup()
def solenoid():
	try:
		n1 = 180
		ang1 = (n1/180*10) +2.5
		ss.ChangeDutyCycle(ang1)
		time.sleep(3)
		ss.ChangeDutyCycle(7.5)
	except KeyboardInterrupt:
		ss.stop()
		GPIO.cleanup
	
def main():
	#x = threading.Thread(target=dcmotor)
	#x.start()
	#y = threading.Thread(target=solenoid)
	#y.start()
	while True:
#		servo()
		dcmotor()
		time.sleep(3)
		solenoid()
		p.stop()
	
if __name__ == '__main__':
	main()
	

