import RPi.GPIO as GPIO
import time
import sys
import tty
import termios
import math

from digi.xbee.models.status import NetworkDiscoveryStatus
from digi.xbee.devices import ZigBeeDevice

PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

REMOTE_NODE_ID = "MAIN"

SOURCE_ENDPOINT = 0xE9
DESTINATION_ENDPOINT = 0xE8
CLUSTER_ID = 0xC
PROFILE_ID = 0x3332

def getkey():
    fd =sys.stdin.fileno()
    original_attributes = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN, original_attributes)
    return ch\

def encoderA(channel):
    global encoderPos
    if GPIO.input(pin_encA) == GPIO.input(pin_encB):
        encoderPos -= 1
    else:
        encoderPos += 1
#    print('PinA : %d, encoder : %lf\n\r' %(channel, encoderPos))
    
def encoderB(channel):
    global encoderPos
    if GPIO.input(pin_encB) == GPIO.input(pin_encA):
        encoderPos += 1
    else:
        encoderPos -= 1
#    print('PinB : %d, encoder : %lf\n\r' %(channel, encoderPos))

num = 17

def speed2dutyrate(speed):
    aaa = []
    for j in range(0, num): 
        bbb = float(speed-data_speed[j])
        if bbb <= 0:
            aaa.append(bbb)
        else:
            aaa.append(int(-10))
    max_ = max(aaa)
    print(aaa)
    print(max_)
    i = int(aaa.index(max_))
    if i > 16:
        i = 16
    print(i)
    dutyrate = ((data_duty[i+1]-data_duty[i])/(data_speed[i+1]-data_speed[i]))*(speed-data_speed[i]) + data_duty[i]
    return dutyrate


#def speed2dutyrate(speed):
#    i = int(math.floor(speed));
#    dutyrate = ((output[i+1]-output[i])/(input[i+1]-input[i]))*(speed-input[i]) + output[i]
#    return dutyrate

def callback_device_discovered(remote_device):
    print("Device discovered: %s" %remote_device)

def callback_discovery_finished(status):
    if status == NetworkDiscoveryStatus.SUCCESS:
        print("Discovery process finished successfully.")
    else:
        print("There was an error discovering devices: %s" %status.description)

def my_data_received_callback(xbee_message):
    global target_Speed
    global angle
    global k
    global t 
    address = remote_device.get_16bit_addr()
    key = xbee_message.data.decode("utf8")
    
    if key == "c":
        t=1
    elif key == "w":
        target_Speed = target_Speed + 1
    elif key == "s":
        target_Speed = target_Speed - 1
    elif key == "t":
        target_Speed = 15
    elif key == "e":
        target_Speed = 0
    elif key == "a": #Left
        angle = angle + 0.1
        k = 1
    elif key == "d": #Right
        angle = angle - 0.1
        k = 1
    elif key == "q":
        angle = standard
        k = 1
            
    print("Key : %s" %(key))
    print("Received data from %s : %s, %f" %(address, key, target_Speed))
    
device = ZigBeeDevice(PORT, BAUD_RATE)

data_speed = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
data_duty = [0,2.11,2.6,3.11,3.65,4.21,4.75,5.45,6.37,7.4,8.7,10.3,12.4,16.5,22.1,34.5,99]

pin_servo = 18
pin_dc = 12
pin_dir = 26
pin_encA = 23
pin_encB = 24
freq = 50

GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_servo,GPIO.OUT)
GPIO.setup(pin_dc,GPIO.OUT)
GPIO.setup(pin_dir,GPIO.OUT)
GPIO.setup(pin_encA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(pin_encB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(pin_encA, GPIO.BOTH, callback=encoderA)
GPIO.add_event_detect(pin_encB, GPIO.BOTH, callback=encoderB)

servo = GPIO.PWM(pin_servo,freq)
servo.start(0)
dc = GPIO.PWM(pin_dc,freq)
dc.start(0)

GPIO.output(pin_dir, True)

a = 0
t = 0
k = 0
speed = 0
standard = 6.65
angle = standard
target_Speed = 0
print('fuck')
encoderPos = 0
real_Speed = 0
dutyrate = 0
circle_length = 3.141592*0.095
err = 0
err_prev = 0
time_prev = 0

Kp = 1.5
Ki = 0.5
Kd = 0

servo.ChangeDutyCycle(standard) 
dc.ChangeDutyCycle(0)

try:
    device.open()

    xbee_network = device.get_network()
    remote_device = xbee_network.discover_device(REMOTE_NODE_ID)
    xbee_network.set_discovery_timeout(10)
    xbee_network.clear()
    xbee_network.add_device_discovered_callback(callback_device_discovered)
    xbee_network.add_discovery_process_finished_callback(callback_discovery_finished)
    xbee_network.start_discovery_process()

    print("Discovering remote XBee devices...")

    while xbee_network.is_discovery_running():
        time.sleep(0.1)

    if remote_device is None:
        print("Could not find the remote device")
        exit(1)
        
    device.add_data_received_callback(my_data_received_callback)
    device.send_data_async(remote_device, "START")
          
    while t!=1:
    
        if k == 1 :
            servo.ChangeDutyCycle(angle)
            time.sleep(0.2)
            k = 0
            
        start_time = time.time()
        start_pos = encoderPos
        #print('start_time = %d, start_pos = %d' %(start_time, start_pos))
            
        time.sleep(0.5)
            
        end_time = time.time()
        end_pos = encoderPos
        #print('end_time = %d, end_pos = %d' %(end_time, end_pos))
            
        pulse_per_sec = float((end_pos-start_pos)/(end_time-start_time))
        encoder_per_sec = pulse_per_sec/ float(11)
        motor_per_sec = encoder_per_sec / float(4.04)
        car_per_sec = motor_per_sec / float(4.5)
        wheel_per_sec = car_per_sec / float(2.5)
        real_Speed = wheel_per_sec * 60 * circle_length * 60 * 0.001
        
        err = target_Speed - real_Speed
        de = err - err_prev
        dt = time.time() - time_prev
        
        PID_Speed = target_Speed + Kp*err + Ki*err*dt + Kd*de/dt 
        
        dutyrate = speed2dutyrate(PID_Speed)
        
        if dutyrate < 0 :
            dutyrate = 0
        if dutyrate > 99 :
            dutyrate = 99
        
        
        err_prev = err
        time_prev = time.time()
        
        servo.ChangeDutyCycle(0)
        dc.ChangeDutyCycle(dutyrate);
        
        device.send_data_async(remote_device, str(real_Speed))
        #print('RPM : Motor = %f, Car = %f, Wheel = %f' %(motor_per_sec*60, car_per_sec*60, wheel_per_sec*60))
        print('Speed : %f km/h, Target Speed : %f km/h, PID Speed : %f km/h, DutyRate : %f' %(real_Speed, target_Speed, PID_Speed, dutyrate))

finally:
    
    if device is not None and device.is_open():
        print("fuck")
        device.close()