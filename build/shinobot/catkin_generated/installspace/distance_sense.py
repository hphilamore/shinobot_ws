#!/usr/bin/env python2
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
#import RPi.GPIO as GPIO
import time
import sys
import VL53L0X
import RPi.GPIO as GPIO

# # Set the GPIO modes
# GPIO.setmode(GPIO.BCM)

class DistanceSensor():
    def __init__(self, shut_pins):
        self.shut_pins = shut_pins
        # self.pinEcho = pinEcho
        # self.msg_name = msg_name
        self.GPIOsetup()
        self.timing = 0.0
        self.tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)


    
    def GPIOsetup(self):
        """ Set the GPIO Pin modes """
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for pin in self.shut_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        # Keep all low for 500 ms or so to make sure they reset
        time.sleep(0.50)


    def get_timing():
        """ Sets up timing used to measure distance """
        GPIO.output(self.shut_pins[0], GPIO.HIGH)

        self.tof.open()

        # Start ranging
        self.tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)

        self.timing = tof.get_timing()
        if self.timing < 20000:
            self.timing = 20000
        print("Timing %d ms" % (self.timing/1000))

        self.tof.close()

        GPIO.output(self.shut_pins[0], GPIO.LOW)

        # Keep all low for 500 ms or so to make sure they reset
        time.sleep(0.50)

     
    
    def measure(self): 
        """ Take a distance measurement from each sensor"""
        print("initialise distance sense") 
        # Set trigger to False (Low) 
        GPIO.output(self.pinTrigger, False) 
        # Allow module to settle 
        time.sleep(0.5)
        
        # Send 10us pulse to trigger
        GPIO.output(self.pinTrigger, True) 
        time.sleep(0.00001) 
        GPIO.output(self.pinTrigger, False) 

        # Start the timer
        inittime = time.time() 
        starttime = time.time() 
        #stoptime = starttime 

        # The start time is reset until the Echo pin is taken high (==1)
        while GPIO.input(self.pinEcho) == 0: 
            print("measuring...") 
            starttime = time.time() 
            # If the echo is not returned, quit the measurement
            if  starttime - inittime >= self.timeout:
                print("sensor timeout!")
                stoptime = starttime 
                break
            #stoptime = starttime 
        
        # Stop when the Echo pin is no longer high - the end time
        while GPIO.input(self.pinEcho) == 1: 
            stoptime = time.time() 
            # If the sensor is too close to an object, the Pi cannot 
            # see the echo quickly enough, so we have to detect that 
            # problem and say what has happened. 
            # if stoptime - starttime <= 0.04: 
            #     print("Hold on there! You're too close for me to see.") 
            #     stoptime = starttime 
            #     break 
        
        # Calculate pulse length
        elapsedtime = stoptime - starttime 

        # Distance pulse travelled in that time is 
        # time multiplied by the speed of sound (cm/s)
        # (there and back so halve the value)
        distance = (elapsedtime * 34300) / 2 
        print("Distance: %.1f cm" % distance) 
        return distance 


    def talker(self):
        #pub = rospy.Publisher('distance_sense', String, queue_size=10)
        pub = rospy.Publisher(self.msg_name, Float64)
        rospy.init_node(self.msg_name, anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            # hello_str = "hello world %s" % rospy.get_time()
            hello_str = "hello world %s" % str(self.measure())
            rospy.loginfo(hello_str)
            #pub.publish(hello_str)
            pub.publish(self.measure())
            rate.sleep()


# if __name__ == '__main__':
#     dist_sense = DistanceSensor(17, 18, 'distance_sense')
#     try:
#         dist_sense.talker()
#     except rospy.ROSInterruptException:
#         pass
#     except KeyboardInterrupt:
#         # Reset GPIO settings 
#         GPIO.cleanup()
#         sys.exit()
