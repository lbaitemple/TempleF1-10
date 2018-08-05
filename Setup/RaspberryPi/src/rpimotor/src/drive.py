#!/usr/bin/env python

import sys, tty, termios, time, rospy
import RPi.GPIO as GPIO
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import Int32
from race.msg import drive_param

str_msg = Int32()
flagStop = False
pwm_center = 15
pwm_lowerlimit = 10
pwm_upperlimit = 20

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def messageDrive(pwm):
    if(flagStop == False):
	print pwm.velocity
	v = pwm.velocity * 5 + pwm_center
	print v
        if(v < pwm_lowerlimit):
	    m.ChangeDutyCycle(pwm_lowerlimit)
	elif(v > pwm_upperlimit):
	    m.ChangeDutyCycle(pwm_upperlimit)
	else:
	    m.ChangeDutyCycle(v)

	a = pwm.angle * 5 + pwm_center
	if(a < pwm_lowerlimit):
	    s.ChangeDutyCycle(pwm_lowerlimit)
	elif(a > pwm_upperlimit):
	    s.ChangeDutyCycle(pwm_upperlimit)
	else:
	    s.ChangeDutyCycle(a)
    else:
        m.ChangeDutyCycle(pwm_center)
	s.ChangeDutyCycle(pwm_center)

def messageEmergencyStop(flag):
	flagStop = flag.data
	if(flagStop == true):
		m.ChangeDutyCycle(pwm_center)
		s.ChangeDutyCycle(pwm_center)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("drive_parameters", drive_param, messageDrive)
    rospy.Subscriber("eStop", Bool, messageEmergencyStop)
    rospy.spin()

if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)
    sport=rospy.get_param('~steer_port', 12)
    mport=rospy.get_param('~motor_port', 33)
    freq=rospy.get_param('~frequency', 100)
    GPIO.setup(sport, GPIO.OUT)
    GPIO.setup(mport, GPIO.OUT)
    s = GPIO.PWM(sport, freq)  # channel=12 frequency=100Hz
    m = GPIO.PWM(mport, freq)
    s.start(15)
    m.start(15)

    print "ROS stuff initializing"
    listener()

    s.ChangeDutyCycle(15)
    s.stop()
    m.stop()
    GPIO.cleanup()
