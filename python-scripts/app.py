import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt_client
from gpiozero import Servo

# Pin Definitions:
servoPin = 14  # GPIO14 (physical pin 8)
butPin = 2   # GPIO2 (physical pin 3)

dc = 10  # duty cycle (0-100) for PWM pin

# Pin Setup:
GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme
GPIO.setup(servoPin, GPIO.OUT)  # PWM pin set as output
#servo = AngularServo(14, min_angle=90, max_angle=-90)
#servo = Servo(14)
servo = GPIO.PWM(servoPin, 50)   # Initialize PWM on pwmPin 50Hz frequency
#GPIO.setup(butPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Button pin set as input w/ pull-up

clockwise_angle = 7.5
anticlockwise_angle = 2.5

mqtt_connected = False
servo.start(0)

def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def on_connect(client, userdata, flags, rc):
    global mqtt_connected
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe("Motor/move_clockwise")
        client.subscribe("Motor/move_anticlockwise")
        mqtt_connected = True
    else:
        print(f"Failed to connect, return code {rc}")
        exit(1)

def on_message(client, userdata, msg):
    
    payload = msg.payload.decode()
    print(f"Received message on topic {msg.topic}: {payload}")
    
    if msg.topic == "Motor/move_clockwise" or msg.topic == "Motor/move_anticlockwise":
        recieved_value = float(payload)
        #duty_cycle = map_value(recieved_value, 0, 100, 2, 12)
        #print("hallo world")
        move_servo(recieved_value)

def connect_mqtt():
    client = mqtt_client.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect('192.168.0.32', 1883, 60)
    return client

def move_servo(duty_cycle):
    
    servo.ChangeDutyCycle(1)
    #servo.value = 0
    time.sleep(1)
    #servo.set_servo(2,1500)
    servo.stop()

def hello():
    return 'Hello, Raspberry Pi Flask App!'

if __name__ == '__main__':
	while not mqtt_connected:
		try:
			print("[MQTT] Connecting to MQTT Broker...")
			mqtt_client = connect_mqtt()
			# start client loop to get messages, non-blocking 
			if mqtt_client:
				mqtt_client.loop_forever()
		except KeyboardInterrupt:
			GPIO.cleanup()
			#pwm.ChangeDutyCycle(0)
		except ConnectionRefusedError:
				print("Connection refused. Retrying in 5 seeconds...")
				time.sleep(5)
