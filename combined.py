import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import RPi.GPIO as GPIO
import ast
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup(33,GPIO.OUT) #DC Motor
p = GPIO.PWM(33,200) #GPIO13 as PWM output with 200 Hz frequency
p.start(0)

GPIO.setup(32, GPIO.OUT) #Servo Motor
s = GPIO.PWM(32,50)
s.start(7.5)

GPIO.setup(12,GPIO.OUT) #Solenoid Servo
ss = GPIO.PWM(12,50)
ss.start(7.5)


class Sensor(Node):

	def __init__(self):
		super().__init__('sensor')

		self.ang = 0
		self.ang1 = 0

		self.tempPublisher = self.create_publisher(String, 'temperature', 10)
		self.ang_subscriber = self.create_subscription(String,'Angu', self.angular_callback, 8)
		self.ang1_subscriber = self.create_subscription(String, 'Angu1', self.angular1_callback, 9)
		print("I have subscribed")

	def angular_callback(self,msg):
		self.ang = ast.literal_eval(msg.data)
		#ang = int(msg.data)
		print("data received on ang")
		print(self.ang)

	def angular1_callback(self,msg):
		self.ang1 = ast.literal_eval(msg.data)
		#ang1 = int(msg.data)
		print("data received on ang1")
		print(self.ang1)

	def readTemp(self):
		for i in range(10):
			bytes_read, temperature = omron.read()
			toPublish = String()
			toPublish.data = "%s" % temperature #Convert temperature to string
			self.tempPublisher.publish(toPublish)

	def stayalive(self):
		while rclpy.ok():
			while True:
				print("reading 10 times")
				self.readTemp()
				if (self.ang > 0):
					state = 1
					s.ChangeDutyCycle(self.ang)
				self.ang = 0
				if (self.ang1 > 0):
					state = 2
					s.ChangeDutyCycle(self.ang1)
					dcmotor()
					time.sleep(6)
					solenoid()
					time.sleep(3)
					if state ==2:
						p.stop()
						p.start(0)
						s.stop()
						s.start(7.5)
						ss.stop()
						ss.start(7.5)
				self.ang1 = 0
				rclpy.spin_once(self, timeout_sec=0.1)
		print("i have exited")

#D6t Sensor
import pigpio, time, crcmod.predefined, smbus

def dcmotor():
	try:
		p.ChangeDutyCycle(100)
	except KeyboardInterrupt:
		p.stop()
		GPIO.cleanup()
def solenoid():
	try:
		n1 = 180
		ang2 = (n1/180*10) +2.5
		ss.ChangeDutyCycle(ang2)
		time.sleep(2)
		ss.ChangeDutyCycle(7.5)
	except KeyboardInterrupt:
		ss.ChangeDutyCycle(7.5)
		ss.stop()
		GPIO.cleanup

class OmronD6T(object):
   def __init__(self, rasPiChannel=1, omronAddress=0x0a, arraySize=16):
      self.MAX_RETRIES = 5
      self.roomTemp = 0
      self.omronAddress = omronAddress
      self.arraySize = arraySize
      self.BUFFER_LENGTH=(arraySize * 2) + 3       # data buffer size 
      self.CRC_ERROR_BIT = 0x04                    # the third bit
      self.CRC = 0xa4 / (16/arraySize)                             # if you take the CRC of the whole word including the PEC, this is what you get
      self.piGPIO = pigpio.pi()
      self.piGPIOver = self.piGPIO.get_pigpio_version()
      self.i2cBus = smbus.SMBus(1)
      time.sleep(0.1)                # Wait
      
      # initialize the device based on Omron's appnote 1
      self.retries = 0
      self.result = 0
      for i in range(0,self.MAX_RETRIES):
         time.sleep(0.05)                               # Wait a short time
         self.handle = self.piGPIO.i2c_open(rasPiChannel, omronAddress) # open Omron D6T device at address 0x0a on bus 1
	 #print (self.handle)
         if self.handle > 0:
            self.result = self.i2cBus.write_byte(self.omronAddress,0x4c)
            break
         else:
            print ('')
            print ('***** Omron init error ***** handle='+str(self.handle)+' retries='+str(self.retries))
            self.retries += 1
            
   # function to read the omron temperature array
   def read(self):
      self.temperature_data_raw=[0]*self.BUFFER_LENGTH
      self.temperature=[0.0]*self.arraySize         # holds the recently measured temperature
      self.values=[0]*self.BUFFER_LENGTH
      
      # read the temperature data stream - if errors, retry
      retries = 0
      for i in range(0,self.MAX_RETRIES):
         time.sleep(0.05)                               # Wait a short time
         (self.bytes_read, self.temperature_data_raw) = self.piGPIO.i2c_read_device(self.handle, self.BUFFER_LENGTH)
            
         # Handle i2c error transmissions
         if self.bytes_read != self.BUFFER_LENGTH:
            print ('')
            print ('***** Omron Byte Count error ***** - bytes read: '+str(self.bytes_read))
            
            self.retries += 1                # start counting the number of times to retry the transmission

         if self.bytes_read == self.BUFFER_LENGTH:
            # good byte count, now check PEC
            
            t = (self.temperature_data_raw[1] << 8) | self.temperature_data_raw[0]
            self.tPATc = float(t)/10
 #           if (degree_unit == 'F'):
            self.roomTemp = self.C_to_F(self.tPATc)

            # Convert Raw Values to Temperature ('F)
            a = 0
            for i in range(2, len(self.temperature_data_raw)-2, 2):
               self.temperature[a] = self.C_to_F(float((self.temperature_data_raw[i+1] << 8) | self.temperature_data_raw[i])/10)
               a += 1
            
            # Calculate the CRC error check code
            # PEC (packet error code) byte is appended at the end of each transaction. The byte is calculated as CRC-8 checksum, calculated over the entire message including the address and read/write bit. The polynomial used is x8+x2+x+1 (the CRC-8-ATM HEC algorithm, initialized to zero)
            self.crc8_func = crcmod.predefined.mkCrcFun('crc-8')
            
            for i in range(0,self.bytes_read):
               self.values[i] = self.temperature_data_raw[i]
                  
            self.string = "".join(chr(i) for i in self.values)
            self.crc = self.crc8_func(self.string.encode("utf-8"))
               
            if self.crc != self.CRC:
              # print ('***** Omron CRC error ***** Expected '+'%02e'%self.CRC+' Calculated: '+'%02e'%self.crc)
               self.retries += 1                # start counting the number of times to retry the transmission
               self.bytes_read = 0           # error is passed up using bytes read
            else:
               break    # crc is good and bytes_read is good

      return self.bytes_read, self.temperature

   # function for Celsius to Fahrenheit conversion
   def C_to_F(self, degreesCelsius):
      return 9.0*degreesCelsius/5.0 + 32

#if __name__ == '__main__':
 #  omron = OmronD6T()
  # while True:
   #   omron.read()
    #  print ("ICTemp:",omron.roomTemp)
     # for i in range(0,len(omron.temperature)):
      #   print ("Cell",str(i)+":",omron.temperature[i])
      #print ('')
      #time.sleep(1)
# End of Add
omron = OmronD6T(rasPiChannel=1, omronAddress=0x0a, arraySize=16) #Change arraysize to 16?
def main(args=None):
	rclpy.init(args=args)
	sensor = Sensor()
	#sensor.testcallback()
	state = 0
	sensor.stayalive()


if __name__ == '__main__':
	main()
