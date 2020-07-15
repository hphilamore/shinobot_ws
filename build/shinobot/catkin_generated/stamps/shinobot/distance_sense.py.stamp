#!/usr/bin/env python
# license removed for brevity
# PKG = 'numpy_tutorial'
# import roslib; roslib.load_manifest(PKG)
import rospy

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import String
from std_msgs.msg import Float64
#import RPi.GPIO as GPIO
import time
import sys
import VL53L0X
import numpy as np
#import RPi.GPIO as GPIO

# # Set the GPIO modes
# GPIO.setmode(GPIO.BCM)

class DistanceSensor():
    def __init__(self, Nsensors):
        #self.shut_pins = shut_pins
        # self.pinEcho = pinEcho
        # self.msg_name = msg_name
        #self.GPIOsetup()
        self.Nsensors = Nsensors
        self.timing = 0.0
        self.tof = []
        self.sensor_setup()
        self.get_timing()


    def sensor_setup(self):
        """ Create tof object for each LIDAR sensor """
        for n in range(self.Nsensors):
            self.tof.append(VL53L0X.VL53L0X(tca9548a_num=n, tca9548a_addr=0x70))
            self.tof[n].open()
            self.tof[n].start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)


    def get_timing(self):
        """ Sets up timing used to measure distance """

        self.timing = self.tof[1].get_timing()
        if self.timing < 20000:
            self.timing = 20000
        print("Timing %d ms" % (self.timing/1000))
     
    
#     def measure(self): 
#         """ Take a distance measurement from each sensor and return as array """
#         distance = []

#         for n in range(self.Nsensors):
#             distance.append(self.tof[n].get_distance())

#         # reverse order of list to make it intuitive left to right 
#         distance = np.array(distance[::-1], dtype=np.float32)
        
#         return distance 

#     def distance_sensor(self):
#     #def talker(self):
#         #pub = rospy.Publisher('distance_sense', String, queue_size=10)
#         #pub = rospy.Publisher('distance_sense', Float64, , queue_size=1)
#         pub = rospy.Publisher('floats', numpy_msg(Floats),queue_size=1)
#         rospy.init_node('distance_sense', anonymous=True)
#         rate = rospy.Rate(10) # 10hz
#         while not rospy.is_shutdown():
#             # hello_str = "hello world %s" % rospy.get_time()
#             hello_str = "hello world %s" % str(self.measure())
#             rospy.loginfo(hello_str)
#             #pub.publish(hello_str)
#             pub.publish(self.measure())
#             rate.sleep()


if __name__ == '__main__':
    dist_sense = DistanceSensor(3)
#     try:
#         dist_sense.distance_sensor()
#         #print(dist_sense.measure())
#         #dist_sense.talker()
#     except rospy.ROSInterruptException:
#         pass
#     except KeyboardInterrupt:
#         for n in range(dist_sense.Nsensors):
#             tof[n].stop_ranging()
#             tof[n].close()
#             pass


#         sys.exit()
