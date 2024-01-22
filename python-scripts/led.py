import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt_client

# Pin Definitions:
servoPin = 14  # GPIO14 (physical pin 8)
redLedPin = 17   # GPIO2 (physical pin 11)
greenLedPin = 22   # GPIO2 (physical pin 15)
yellowLedPin = 27   # GPIO2 (physical pin 13)

# Pin Setup:
GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme
GPIO.setup(redLedPin, GPIO.OUT)  # PWM pin set as output
GPIO.setup(greenLedPin, GPIO.OUT)  # PWM pin set as output
GPIO.setup(yellowLedPin, GPIO.OUT)  # PWM pin set as output
GPIO.setup(servoPin, GPIO.OUT)  # PWM pin set as output

# Set up PWM for each LED
red_led_pwm = GPIO.PWM(redLedPin, 100)  # 100 Hz frequency
green_led_pwm = GPIO.PWM(greenLedPin, 100)
yellow_led_pwm = GPIO.PWM(yellowLedPin, 100)
servo = GPIO.PWM(servoPin, 50)   # Initialize PWM on pwmPin 50Hz frequency

mqtt_connected = False
servo.start(0)

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
    if msg:
        recieved_value = float(payload)
        print("recieved value: ", recieved_value)
        move_servo(recieved_value)
        update_leds(msg.topic)

def connect_mqtt():
    client = mqtt_client.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect('192.168.0.28', 1883, 60) #'192.168.86.45'
    return client

def update_leds(topic):
    if topic == 'Motor/move_clockwise':
        turn_on_leds([redLedPin, yellowLedPin, greenLedPin])
    elif topic == 'Motor/move_anticlockwise':
        turn_on_leds([greenLedPin, yellowLedPin, redLedPin])

def turn_on_leds(led_order):
    for led_pin in led_order:
        GPIO.output(led_pin, 1)
        time.sleep(1)
        GPIO.output(led_pin, 0)

def move_servo(pos):
    ms = ((2/180)*pos) + 0.5
    dc = (ms/20)*100
    servo.ChangeDutyCycle(dc)
    time.sleep(4)

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
		except ConnectionRefusedError:
				print("Connection refused. Retrying in 5 seeconds...")
				time.sleep(5)
