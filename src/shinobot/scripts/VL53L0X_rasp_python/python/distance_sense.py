#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
#import RPi.GPIO as GPIO
import time
import sys
import VL53L0X
#import RPi.GPIO as GPIO

# # Set the GPIO modes
# GPIO.setmode(GPIO.BCM)

class DistanceSensor():
    def __init__(self, Nsensors):
        #self.shut_pins = shut_pins
        # self.pinEcho = pinEcho
        # self.msg_name = msg_name
        #self.GPIOsetup()
        self.timing = 0.0
        self.tof = []
        self.sensor_setup(Nsensors)
        self.get_timing()


    def sensor_setup(self, N):
        """ Create tof object for each LIDAR sensor """
        for n in range(N):
            self.tof.append(VL53L0X.VL53L0X(tca9548a_num=n, tca9548a_addr=0x70))
            self.tof[n].open()
            self.tof[n].start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)


    def get_timing():
        """ Sets up timing used to measure distance """

        self.timing = tof[1].get_timing()
        if self.timing < 20000:
            self.timing = 20000
        print("Timing %d ms" % (self.timing/1000))
     
    
    def measure(self): 
        """ Take a distance measurement from each sensor and return as array """
        distance = []

        for n in range(N):
            distance.append(tof[n].get_distance())
        
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


if __name__ == '__main__':
    dist_sense = DistanceSensor(3)
    try:
        print(measure())
        #dist_sense.talker()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        for n in range(N):
            tof[n].stop_ranging()
            tof[n].close()

        # Reset GPIO settings 
        #GPIO.cleanup()
        sys.exit()
