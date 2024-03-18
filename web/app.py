import RPi.GPIO as GPIO
import serial
from time import sleep
from flask import Flask, render_template, request

servo_comm = load.module("~/worktemp/rpiWebServer_RS485_ServoCtrl")
from servo_comm.crc import CRC16CCITT
from servo_comm.servoparams import ServoParams

def delay_ms(milliseconds):
      """
      Delay execution for a given number of milliseconds.
      Parameters:
      milliseconds (int): The number of milliseconds to delay.
      """
      seconds = milliseconds / 1000.0 # Convert milliseconds to seconds
      time.sleep(seconds)

# RSE TX/RX Control Pin
RS485_ENABLE_PIN = 4

app = Flask(__name__)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(RS485_ENABLE_PIN, GPIO.OUT)
GPIO.output(RS485_ENABLE_PIN, GPIO.HIGH) # Set High to Transimit

# Open Serial Port ttyAMA0 (Replace ttyS0 With ttyAM0 for Pi1,Pi2,Pi0)
# 57600, N, 8, 1
ser_port = serial.Serial("/dev/ttyS0",57600)
ser_port.bytesize = serial.EIGHTBITS
ser_port.parity = serial.PARITY_NONE
ser_port.stopbits = serial.STOPBITS_ONE

# ser_port.timeout = 0.5
# ser_port.write_timeout = 0.5

# define actuators GPIO
ledRed = 13
ledYlw = 19
ledGrn = 26

# initialize GPIO status variables
ledRedSts = 0
ledYlwSts = 0
ledGrnSts = 0

RS485_send = ""
RS485_read = ""

# Define led pins as output 
GPIO.setup(ledRed, GPIO.OUT)
GPIO.setup(ledYlw, GPIO.OUT)
GPIO.setup(ledGrn, GPIO.OUT)

# To create a PWM instance
# GPIO.PWM(pinNumber, Frequency)
p = GPIO.PWM(ledRed, 2)


# Turn leds OFF
GPIO.output(ledRed, GPIO.LOW)
GPIO.output(ledYlw, GPIO.LOW)
GPIO.output(ledGrn, GPIO.LOW)


@app.route("/")
def index():
	# Read Sensors Status
	ledRedSts = GPIO.input(ledRed)
	ledYlwSts = GPIO.input(ledYlw)
	ledGrnSts = GPIO.input(ledGrn)

	templateData = {
      'title' : 'GPIO output Status!',
      'ledRed'  : ledRedSts,
      'ledYlw'  : ledYlwSts,
      'ledGrn'  : ledGrnSts,
      'RS485_read':RS485_read,
      'RS485_send':RS485_send,
      }
	return render_template('index.html', **templateData)

@app.route("/<deviceName>/<action>")
def action(deviceName, action):
      if deviceName == 'ledRed':
            actuator = ledRed
      if deviceName == 'ledYlw':
            actuator = ledYlw
      if deviceName == 'ledGrn':
            actuator = ledGrn
      if deviceName == 'RS485':
            print("Send Msg!!")
      if deviceName == 'recieveMsg':
            print("Get Msg!!")

      if action == "on":
            if actuator == ledRed:
                  p.start(50)
                  print("Start PWM!")
            else:
                  GPIO.output(actuator, GPIO.HIGH)
      elif action == "off":
            if actuator == ledRed:
                  p.stop()
                  print("Stop PWM!")
            else: 
                  GPIO.output(actuator, GPIO.LOW)
      
      elif action == "servoOn":
            print("SERVO ON")            
            print("SET PARAM 2!")

            bytes_object = bytes(ServoParams.SET_PARAM_2)
            RS485_send = str(bytes_object)

            ser_port.write(ServoParams.SET_PARAM_2)
            sleep(0.1)
            print("Response: ")
            result_1 = ser_port.inWaiting()
            result_2 = ser_port.read(ser_port.inWaiting())
            print(result_1)
            print(result_2)

            RS485_read = str(result_2)

            print("SERVO ON")
            ser_port.write(ServoParams.SERVO_ON)
            # ser_port.write(result_data_with_crc_list)
            sleep(0.1)

            print("Response:")
            result_1 = ser_port.inWaiting()
            result_2 = ser_port.read(ser_port.inWaiting())
            print(result_1)
            print(result_2)

      elif action == "servoOff":
            print("SERVO OFF")
            bytes_object = bytes(ServoParams.SERVO_OFF)
            RS485_send = str(bytes_object)
            ser_port.write(ServoParams.SERVO_OFF)
            sleep(0.1)
            print("Response:")
            result_1 = ser_port.inWaiting()
            result_2 = ser_port.read(ser_port.inWaiting())
            print(result_1)
            print(result_2)
            RS485_read = str(result_2)
      
      elif action == "getMsg":
            print("GET VALUE")
            bytes_object = bytes(ServoParams.GET_STATE_VALUE_4)
            RS485_send = str(bytes_object)
            ser_port.write(ServoParams.GET_STATE_VALUE_4)
            sleep(0.1)
            print("Response:")
            result_1 = ser_port.inWaiting()
            result_2 = ser_port.read(ser_port.inWaiting())
            print(result_1)
            print(result_2)
            RS485_read = str(result_2)
      
      p.ChangeFrequency(2)
      ledRedSts = GPIO.input(ledRed)
      ledYlwSts = GPIO.input(ledYlw)
      ledGrnSts = GPIO.input(ledGrn)


      templateData = {
            'ledRed' : ledRedSts,
            'ledYlw' : ledYlwSts,
            'ledGrn' : ledGrnSts,
            'RS485_read':RS485_read,
            'RS485_send':RS485_send,
      }

      return render_template('index.html', **templateData)
if __name__ == "__main__":
   app.run(host='0.0.0.0', port=5000, debug=True)
