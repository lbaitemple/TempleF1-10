import sys, tty, termios, time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(33, GPIO.OUT)
s = GPIO.PWM(12, 100)  # channel=12 frequency=100Hz
m = GPIO.PWM(33, 100)
s.start(15)
m.start(15)

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

#try:
while 1:
	char = getch()
	if(char == "a"):
	        s.ChangeDutyCycle(10)
	elif(char == "d"):
		s.ChangeDutyCycle(20)
	else:
		s.ChangeDutyCycle(15)
	if(char == "w"):
                m.ChangeDutyCycle(20)
        elif(char == "s"):
                m.ChangeDutyCycle(10)
	else:
		m.ChangeDutyCycle(15)
	if(char=="x"):
		print("Program end")
		break
	print(char)
	char = ""
        
#except KeyboardInterrupt:
 #   pass
s.ChangeDutyCycle(15)
s.stop()
m.stop()
GPIO.cleanup()

