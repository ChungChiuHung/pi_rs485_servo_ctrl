'''
	Raspberry Pi GPIO Status and Control
'''
import RPi.GPIO as GPIO
import serial
from time import sleep
from flask import Flask, render_template, request

from checksum import CRC16CCITT

# RSE TX/RX Control Pin
RS485_ENABLE_PIN = 4

app = Flask(__name__)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(RS485_ENABLE_PIN, GPIO.OUT)
GPIO.output(RS485_ENABLE_PIN, GPIO.HIGH) # Set High to Transimit

# Open Serial Port tty50
# 57600, N, 8, 1
ser_port = serial.Serial("/dev/ttyAMA0",57600)
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

# Data Format
# A     B     C  D  E         F
# 24 01 00 04 00 04 ****...** ****
# 26 01 00 66 01 20 
# E (2~29 byte)
# 參數(No.) 寫入值 狀態編號 解鎖代碼
# F 錯誤檢出代碼
# SET_STATE_VALUE_WITHMASK_4
# E I/O Section
# 狀態編號            2 bytes
# 設定的狀態值        4 bytes
# 指定的位元/遮罩Mask 4 bytes
#    - 0 不修改
#    - 1 修改
# Example:
#   原始狀態  : 0F 00 00 00
#   設定值    : 60 00 00 01
#   遮罩      : 00 00 00 01
#   寫入後狀態: 0F 00 00 01

# Response: E I/O Section
# 處理結果 2 bytes
#    - 0x00 正常
#    - 0x01 異常
# 被寫入的值 4 bytes

SET_PARAM_2 = [0x26,0x01,
               0x00,0x07, # 數據代碼
               0x00,0x09, # 參數群組 
               0x00,0x01, # 設定值
               0x9B,0x98]
# 0x00, 0x07 Write 2 bytes
# 0x00, 0x08 Write 4 bytes


SERVO_ON = [0x2C,0x01,
            0x00,0x66,
            0x01,0x20,
            0x00,0x00,0x00,0x01,
            0x00,0x00,0x00,0x01,
            0x0A,0x9A]

SERVO_OFF = [0x2C,0x01,
             0x00,0x66,
             0x01,0x20,
             0x00,0x00,0x00,0x00,
             0x00,0x00,0x00,0x01,
             0xA0,0xCB]
            
GO_ZERO_cmd1 = [0x2C,0x01,
                0x00,0x66,
                0x01,0x20,
                0x00,0x08,0x00,0x00,
                0x00,0x08,0x00,0x00,
                0x8A,0xE6]
         
GO_ZERO_cmd2 = [0x2C,0x01,
                0x00,0x66,
                0x01,0x20,
                0x00,0x00,0x00,0x00,
                0x00,0x08,0x00,0x00,
                0x19,0x4B]

SET_ZERO_POINT = [0x2C,0x01,
                  0x00,0x66,
                  0x01,0x20,
                  '*','*',0x00,0x00,
                  0x3C,0x00,0x00,0x00,
                  '*','*']

Motor_Activate_1 = [0x2C,0x01,
                    0x00,0x66,
                    0x01,0x20,
                    0x01,0x00,0x00,0x00,
                    0x01,0x00,0x00,0x00,
                    0x81,0x8D]

Motor_Activate_2 = [0x2C,0x01,
                    0x00,0x66,
                    0x01,0x20,
                    0x00,0x00,0x00,0x00,
                    0x01,0x00,0x00,0x00,
                    0xC6,0x5E]

Motor_Pause_1 = [0x2C,0x01,
                 0x00,0x66,
                 0x01,0x20,
                 0x00,0x20,0x00,0x00,
                 0x01,0x20,0x00,0x00,
                 0x58,0xDA]

Motor_Pause_2 = [0x2C,0x01,
                 0x00,0x66,
                 0x01,0x20,
                 0x00,0x00,0x00,0x00,
                 0x01,0x20,0x00,0x00,
                 0x36,0x2C]

Motor_Restart_1 = [0x2C,0x01,
                   0x00,0x66,
                   0x01,0x20,
                   0x00,0x20,0x00,0x00,
                   0x01,0x20,0x00,0x00,
                   0x58,0xDA]

Motor_Restart_2 = [0x2C,0x01,
                   0x00,0x66,
                   0x01,0x20,
                   0x00,0x00,0x00,0x00,
                   0x01,0x20,0x00,0x00,
                   0x36,0x2C]

GET_STATE_VALUE_4 = [0x24,0x01,
                     0x00,0x11,
                     0x01,0x28,
                     0x75,0xE0]
# response
# 26 01 80 11 (* *) (* *) (* *) (* *) (* *)(* *)
#

# Example usage:
crc_calculator = CRC16CCITT()
test_data_list = [0x2C,0x01,
                  0x00,0x66,
                  0x01,0x20,
                  0x00,0x00,0x00,0x01,
                  0x00,0x00,0x00,0x01]
result_data_with_crc_list = crc_calculator.append_crc_as_list(test_data_list)

print("Original data as list:", test_data_list)
print("Data with CRC-16-CCITT appended as list:", result_data_with_crc_list)


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
      RS485_read=""
      RS485_send=""
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
      if action == "off":
            if actuator == ledRed:
                  p.stop()
                  print("Stop PWM!")
            else: 
                  GPIO.output(actuator, GPIO.LOW)
      
      if action == "servoOn":
            print("SERVO ON")            
            print("SET PARAM 2!")

            bytes_object = bytes(SET_PARAM_2)
            RS485_send = str(bytes_object)

            ser_port.write(SET_PARAM_2)
            sleep(0.05)
            print("Response: ")
            result_1 = ser_port.inWaiting()
            result_2 = ser_port.read(ser_port.inWaiting())
            print(result_1)
            print(result_2)

            RS485_read = str(result_2)

            print("SERVO ON")
            ser_port.write(SERVO_ON)
            # ser_port.write(result_data_with_crc_list)
            sleep(0.05)

            print("Response:")
            result_1 = ser_port.inWaiting()
            result_2 = ser_port.read(ser_port.inWaiting())
            print(result_1)
            print(result_2)

      if action == "servoOff":
            print("SERVO OFF")
            bytes_object = bytes(SERVO_OFF)
            RS485_send = str(bytes_object)
            ser_port.write(SERVO_OFF)
            sleep(0.05)
            print("Response:")
            result_1 = ser_port.inWaiting()
            result_2 = ser_port.read(ser_port.inWaiting())
            print(result_1)
            print(result_2)
            RS485_read = str(result_2)
      
      if action == "getMsg":
            print("GET VALUE")
            bytes_object = bytes(GET_STATE_VALUE_4)
            RS485_send = str(bytes_object)
            ser_port.write(GET_STATE_VALUE_4)
            sleep(0.05)
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