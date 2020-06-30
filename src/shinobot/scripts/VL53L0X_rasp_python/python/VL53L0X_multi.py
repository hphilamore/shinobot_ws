#!/usr/bin/python

# MIT License
# 
# Copyright (c) 2017 John Bryan Moore
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import time
import VL53L0X
import RPi.GPIO as GPIO ##

# 'L', 'C', 'R'
shut_pins = [20, 17, 16]
timing = 0.0

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

for pin in shut_pins:
	GPIO.setup(pin, GPIO.OUT)
	GPIO.output(pin, GPIO.LOW)

# Keep all low for 500 ms or so to make sure they reset
time.sleep(0.50)

# Create a VL53L0X object
tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)

def get_timing():
	""" 
	Sets up timing used to measure distance 
	"""
	GPIO.output(shut_pins[0], GPIO.HIGH)

	tof.open()
	# Start ranging
	tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)

	timing = tof.get_timing()
	if timing < 20000:
	    timing = 20000
	print("Timing %d ms" % (timing/1000))

	tof.close()

	GPIO.output(shut_pins[0], GPIO.LOW)

	# Keep all low for 500 ms or so to make sure they reset
	time.sleep(0.50)

def distance_sense(pin):
	"""
	Measure distance each sensor
	"""

	GPIO.output(pin, GPIO.HIGH)
	# Keep all low for 500 ms or so to make sure they reset
	time.sleep(0.50)

	tof.open()
	# Start ranging
	tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)

	for count in range(1, 101):
	    distance = tof.get_distance()
	    if distance > 0:
	        print("%d mm, %d cm, %d" % (distance, (distance/10), count))

	    time.sleep(timing/1000000.00)

	tof.stop_ranging()
	tof.close()

	GPIO.output(pin, GPIO.LOW)
	# Keep all low for 500 ms or so to make sure they reset
	time.sleep(0.50)


get_timing()

for pin in shut_pins:
	distance_sense(pin)
