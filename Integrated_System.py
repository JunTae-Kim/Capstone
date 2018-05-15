__author__ = 'ataylor'

from smbus import SMBus

from math import floor
from time import sleep
import sys
import os
import cv2
import timeit
import numpy as np
import tensorflow as tf


# pwm
# The default address, used unless the address jumpers are bridged
DEFAULT_ADDRESS = 0x40
# The default value on modern Raspberry Pis
DEFAULT_I2C_BUS = 1
# Registers:
MODE1 = 0x00
MODE2 = 0x01
PRE_SCALE = 0xFE

# Base address which all LEDs are referenced from
# Each LED is 4 addresses:
#  On H
#  On L
#  Off H
#  Off L
LED_BASE = 0x06

# Internal oscillator frequency
OSC = 25000000
# Stock frequency of 60Hz
FREQ = 60
channel = int(0)
position = int(-100)
servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096

#pwm end


camera = cv2.VideoCapture("nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)320, height=(int)240,format=(string)I420, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")

#camera = cv2.VideoCapture(1)
#camera.set(6, 1)
#camera.set(4, 320)
#camera.set(5, 240)

framecount = 0
String1 = 'left'
String2 = 'right'
String3 = 'mid'
# Loads label file, strips off carriage return
label_lines = [line.rstrip() for line
               in tf.gfile.GFile("/home/nvidia/work/lane/retrained_labels.txt")]

#pwm
class pca9865_servo(object):

    def __init__(self, i2c_address=DEFAULT_ADDRESS, i2c_bus=DEFAULT_I2C_BUS):

        # Create a bus object for the given bus
        self.bus = SMBus(i2c_bus)

        # Use the supplied target address
        self.address = i2c_address

        # Turn all the outputs off
        self._set_all_off()

        # Send the unit to sleep so we can set the oscillator
        self._write_bytes(MODE1, 0x10)

        # Wait for the unit to stabilise after sending it to sleep
        sleep(0.1)

        # We will be working at 60Hz
        prescale_value = int(floor(OSC / (4096*FREQ)) - 1)
        # This has to be set whilst the unit is in sleep mode
        self._write_bytes(PRE_SCALE, prescale_value)

        # The oscillator takes a while to stabilise
        sleep(0.1)

        # Set default values in MODE1, bring it out of sleep
        self._write_bytes(MODE1, 0x00)
        # Set the outputs to open-dram
        self._write_bytes(MODE2, 0x04)

    # Set all outputs to off, handy for resetting
    def _set_all_off(self):
        self._write_bytes(0xFA, 0)
        self._write_bytes(0xFB, 0)
        self._write_bytes(0xFC, 0)
        self._write_bytes(0xFD, 0)

    def _write_bytes(self, register, value):
        self.bus.write_byte_data(self.address, int(register), int(value))

    def set_servo_value(self, channel, value):
        # We just set off the on times, the rest is spent being off
        self._write_bytes(LED_BASE+2+4*channel, value & 0xFF)
        self._write_bytes(LED_BASE+3+4*channel, value >> 8)
        pass


servo_controller = pca9865_servo()
#pwm end





def grabVideoFeed():
    grabbed, frame = camera.read()
    return frame if grabbed else None

def initialSetup():
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
    start_time = timeit.default_timer()

    # This takes 2-5 seconds to run
    # Unpersists graph from file
    with tf.gfile.FastGFile("/home/nvidia/work/lane/retrained_graph.pb", 'rb') as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())
        tf.import_graph_def(graph_def, name='')

    print 'Took {} seconds to unpersist the graph'.format(timeit.default_timer() - start_time)
initialSetup()
with tf.Session() as sess:
    start_time = timeit.default_timer()

    # Feed the image_data as input to the graph and get first prediction
    softmax_tensor = sess.graph.get_tensor_by_name('final_result:0')

    print 'Took {} seconds to feed data to graph'.format(timeit.default_timer() - start_time)
    while True:
        frame = grabVideoFeed()
	framecount = framecount + 1
        if frame is None:
            raise SystemError('Issue grabbing the frame')

        cv2.imshow('Main', frame)
	# adhere to TS graph input structure
	numpy_frame = np.asarray(frame)	
	start_time = timeit.default_timer()
	# This takes 2-5 seconds as well
	predictions = sess.run(softmax_tensor, {'DecodeJpeg:0': numpy_frame})
	prediction = predictions[0]
	print 'Took {} seconds to perform prediction'.format(timeit.default_timer() - start_time)
	start_time = timeit.default_timer()
	
	# Sort to show labels of first prediction in order of confidence
	top_k = predictions[0].argsort()[-len(predictions[0]):][::-1]
	
	print 'Took {} seconds to sort the predictions'.format(timeit.default_timer() - start_time)
	prediction = prediction.tolist()
	max_value = max(prediction)
	max_index = prediction.index(max_value)
	predicted_label = label_lines[max_index]
	print("%s (%.2f%%)" % (predicted_label, max_value * 100))
	if String1.lower() == predicted_label.lower():
		position = int(-180)
		mid_point = servo_max - servo_min
	        servo_position = mid_point + position
        	servo_controller.set_servo_value(channel, servo_position)
		print "left"
	elif String2.lower() == predicted_label.lower():
		position = int(-20)
		mid_point = servo_max - servo_min
	        servo_position = mid_point + position
        	servo_controller.set_servo_value(channel, servo_position)
		print "right"
	elif String3.lower() == predicted_label.lower():
		position = int(-100)
		mid_point = servo_max - servo_min
	        servo_position = mid_point + position
        	servo_controller.set_servo_value(channel, servo_position)
		print "mid"
	print '********* Session Ended *********'
	
	if cv2.waitKey(1) & 0xFF == ord('q'):
	     sess.close()
	     break

sess.close()
camera.release()
cv2.destroyAllWindows()
