import RPi.GPIO as GPIO
import time

servoPin = 14  # GPIO14 (physical pin 8)
#0º(1ms) -> dc=5, 90º(1.5ms)->dc=7.5, 180º(2ms)->dc=10
#rule: ((angle*1.5)/90)
#dc = (float(1)/((1/50)*1000))*100
GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme
GPIO.setup(servoPin, GPIO.OUT)  # PWM pin set as output
servo = GPIO.PWM(servoPin, 50)   # Initialize PWM on pwmPin 50Hz frequency

servo.start(0)

if __name__ == '__main__':
    servo.ChangeDutyCycle((0.5/20)*100)
    time.sleep(2)
    GPIO.cleanup()
