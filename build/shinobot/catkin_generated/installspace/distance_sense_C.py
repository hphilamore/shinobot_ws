#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
import RPi.GPIO as GPIO
import time
import sys
from distance_sense import DistanceSensor

if __name__ == '__main__':
    dist_sense = DistanceSensor(4, 7, 'distance_sense_C')
    try:
        dist_sense.talker()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        # Reset GPIO settings 
        GPIO.cleanup()
        sys.exit()
