#!/usr/bin/env python

import sys, tty, termios, time, rospy
import RPi.GPIO as GPIO
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import Int32
from race.msg import drive_values 

str_msg = Int32()
flagStop = False
pwm_center = 15
pwm_lowerlimit = 10
pwm_upperlimit = 20
chatter = rospy.Publisher('chatter', String, queue_size=10)

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
    if(flagStop == false):
        str_msg.data = pwm.pwm_drive
        chatter.publish(str_msg)

        if(pwm.pwm_drive < pwm_lowerlimit):
	    m.ChangeDutyCycle(pwm_lowerlimit)
	elif(pwm.pwm_drive > pwm_upperlimit):
	    m.ChangeDutyCycle(pwm_upperlimit)
	else:
	    m.ChangeDutyCycle(pwm.pwm_drive)

	if(pwm.pwm_angle < pwm_lowerlimit):
	    s.ChangeDutyCycle(pwm_lowerlimit)
	elif(pwm.pwm_angle > pwm_upperlimit):
	    s.ChangeDutyCycle(pwm_upperlimit)
	else:
	    s.ChangeDutyCycle(pwm.pwm_drive)
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
    rospy.Subscriber("drive_pwm", drive_values, messageDrive)
    rospy.Subscriber("eStop", Bool, messageEmergencyStop)
    rospy.spin()

if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(12, GPIO.OUT)
    GPIO.setup(33, GPIO.OUT)
    s = GPIO.PWM(12, 100)  # channel=12 frequency=100Hz
    m = GPIO.PWM(33, 100)
    s.start(15)
    m.start(15)

    print "ROS stuff initializing"
    listener()

    s.ChangeDutyCycle(15)
    s.stop()
    m.stop()
    GPIO.cleanup()
