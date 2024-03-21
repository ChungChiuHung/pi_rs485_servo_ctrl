import RPi.GPIO as GPIO
import serial
import time
from time import sleep
from flask import Flask, render_template, request
from servo_params import ServoParams
from base_msg_generator import BaseMsgGenerator
from response_parsing import ResponseMsgParser
from command_code import CmdCode
from set_servo_io_status import SetServoIOStatus
from set_servo_io_status import BitMap
from cal_cmd_response_time import CmdDelayTime
from io_status_fetcher import IOStatusFetcher

# Define your GPIO pins upfront
RS485_ENABLE_PIN = 4  # pin for RS485 transmission enable
LED_RED_PIN = 13           # pin assignments for LEDs
LED_YLW_PIN = 19
LED_GRN_PIN = 26

app = Flask(__name__)

# Initailize global variables for RS485 messages
RS485_send = ''
RS485_read = ''
ser_port = None

def initialize_gpio():
      GPIO.setmode(GPIO.BCM)
      GPIO.setwarnings(False)
      GPIO.setup(RS485_ENABLE_PIN, GPIO.OUT)
      GPIO.output(RS485_ENABLE_PIN, GPIO.HIGH) # Set High to Transmit

      GPIO.setup(LED_RED_PIN, GPIO.OUT)
      GPIO.setup(LED_YLW_PIN, GPIO.OUT)
      GPIO.setup(LED_GRN_PIN, GPIO.OUT)
      GPIO.output(LED_RED_PIN, GPIO.LOW)
      GPIO.output(LED_YLW_PIN, GPIO.LOW)
      GPIO.output(LED_GRN_PIN, GPIO.LOW)

      # To create a PWM instance
      # GPIO.PWM(pinNumber, Frequency)
      pwm_red_led = GPIO.PWM(LED_RED_PIN, 2)
      pwm_red_led.start(0) # Start PWM with 0% duty cycle (off)

      return pwm_red_led

def cleanup_gpio():
      GPIO.cleanup()


def initialize_serial():
      global ser_port
      ser_port = serial.Serial("/dev/ttyS0", 57600)
      ser_port.bytesize = serial.EIGHTBITS
      ser_port.parity = serial.PARITY_NONE
      ser_port.stopbits = serial.STOPBITS_ONE
      return ser_port

def delay_ms(milliseconds):
      seconds = milliseconds / 1000.0 # Convert milliseconds to seconds
      sleep(seconds)

def print_byte_array_as_spaced_hex(byte_array, data_name):
    hex_string = ' '.join(f"{byte:02X}" for byte in byte_array)
    print(f"{data_name}: {hex_string}")

@app.route("/")
def index():
      ledRedSts = GPIO.input(LED_RED_PIN)
      ledYlwSts = GPIO.input(LED_YLW_PIN)
      ledGrnSts = GPIO.input(LED_GRN_PIN)

      templateData = {
        'title': 'GPIO output Status!',
        'ledRed': ledRedSts,
        'ledYlw': ledYlwSts,
        'ledGrn': ledGrnSts,
        'RS485_read': RS485_read,
        'RS485_send': RS485_send,
      }
      return render_template('index.html', **templateData) 

@app.route("/<deviceName>/<action>")
def action(deviceName, action):
      global RS485_send, RS485_read, pwm_red_led, ser_port

      # Initialize or update the device statuses
      ledRedSts = GPIO.input(LED_RED_PIN)
      ledYlwSts = GPIO.input(LED_YLW_PIN)
      ledGrnSts = GPIO.input(LED_GRN_PIN)

      # After performing actions, update the statuses if needed
      if deviceName == 'ledRed' and action in ['on', 'off']:
            ledRedSts = GPIO.input(LED_RED_PIN)
      elif deviceName == 'ledYlw' and action in ['on', 'off']:
            ledYlwSts = GPIO.input(LED_YLW_PIN)
      elif deviceName == 'ledGrn' and action in ['on', 'off']:
            ledGrnSts = GPIO.input(LED_GRN_PIN)

      cmd_generator = BaseMsgGenerator()
      setter = SetServoIOStatus()
      parser = ResponseMsgParser()
      fetcher = IOStatusFetcher(ser_port)

      pause_toggle_bit = True

      cmd_delay_time = CmdDelayTime(57600)

      # Config the waiting reponse timeout
      timeout = 1 # Timeout in second
      deadline = time.time() + timeout

      protocol_id=1
      destination_address = 1
      dir_bit = 0
      error_code = 0


      if action == "servoOn":
            # SET_PARAM_2 command        
            set_param_2_command = ServoParams.SET_PARAM_2
            RS485_send = print_byte_array_as_spaced_hex(set_param_2_command, f"{ServoParams.SET_PARAM_2}")
            ser_port.write(set_param_2_command)

            delay_ms(50)
            cmd_delay_time.calculate_transmission_time_ms(set_param_2_command)

            print("Response: ")
            result = b''
            num_bytes_available = ser_port.inWaiting()
            while time.time() < deadline:
                  if num_bytes_available > 0:
                        result = ser_port.read(num_bytes_available)
                  time.sleep(0.05) 
            print(result)
            RS485_read = print_byte_array_as_spaced_hex(result, f"{set_param_2_command}")

            # SERVO_ON Command
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.SVON, 1)
            servo_on_command = cmd_generator.generate_message(
                  protocol_id,
                  destination_address,
                  dir_bit, error_code, cmd_code, parameter_data)
            
            RS485_send = print_byte_array_as_spaced_hex(servo_on_command, f"{cmd_code}")
            ser_port.write(servo_on_command)

            # Fixed delay plus transmission delay calculation
            delay_ms(50)
            cmd_delay_time.calculate_transmission_time_ms(servo_on_command)
            
            print("Response: ")
            result = b''
            num_bytes_available = ser_port.inWaiting()
            while time.time() < deadline:
                  if num_bytes_available > 0:
                        result = ser_port.read(num_bytes_available)
                  delay_ms(50) 
            print(result)
            RS485_read = print_byte_array_as_spaced_hex(result, f"{cmd_code}")

      elif action == "servoOff":
            print("SERVO OFF")
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.SVON, 0)
            servo_off_command = cmd_generator.generate_message(
                  protocol_id,
                  destination_address,
                  dir_bit, error_code, cmd_code, parameter_data)
            
            RS485_send = print_byte_array_as_spaced_hex(servo_off_command, f"{cmd_code}")
            ser_port.write(servo_off_command)

            delay_ms(50)
            cmd_delay_time.calculate_transmission_time_ms(servo_off_command)

            print("Response:")
            result = b''
            num_bytes_available = ser_port.inWaiting()
            while time.time() < deadline:
                  if num_bytes_available > 0:
                        result = ser_port.read(num_bytes_available)
                  delay_ms(50) 
            print(result)
            RS485_read = print_byte_array_as_spaced_hex(servo_off_command, f"{cmd_code}")
      
      elif action == "getMsg":            
            get_io_command = ServoParams.GET_INPUT_IO
            RS485_send = print_byte_array_as_spaced_hex(get_io_command, f"{CmdCode.GET_STATE_VALUE_4}")
            ser_port.write(get_io_command)

            #delay_ms(50)
            #cmd_delay_time.calculate_transmission_time_ms(get_io_command)
            
            print("Response:")

            response = b''

            while ser_port.in_waiting > 0 or not response:
                  response_part = ser_port.readline()
                  response += response_part
                  if response.endswith('\n'):
                        break
                  delay_ms(100)

            print(response)
            # RS485_read = parser.parse_message(result)

      elif action == "getIOOutput":
            get_io_output_command = ServoParams.GET_OUTPUT_IO
            RS485_send = print_byte_array_as_spaced_hex(get_io_output_command, f"GET OUTPUT IO")
            ser_port.write(get_io_output_command)

            delay_ms(50)
            cmd_delay_time.calculate_transmission_time_ms(get_io_output_command)

            print("Response: ")
            result= b''
            num_bytes_available = ser_port.inWaiting()
            while time.time() < deadline:
                  if num_bytes_available > 0:
                        result = ser_port.read(num_bytes_available)
                  delay_ms(50) 
            print(result)
            #RS485_read = fetcher.get_output_io_status(result)

      elif action == "setPoint_1":
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.SEL_NO, 5)
            point_sel_command = cmd_generator.generate_message(
                  protocol_id,
                  destination_address,
                  dir_bit, error_code, cmd_code, parameter_data)
            
            RS485_send = print_byte_array_as_spaced_hex(point_sel_command, f"{cmd_code}")
            ser_port.write(point_sel_command)

            # Fixed delay plus transmission delay calculation
            delay_ms(50)
            cmd_delay_time.calculate_transmission_time_ms(point_sel_command)

      elif action == "setPoint_2":
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.SEL_NO, 6)
            point_sel_command = cmd_generator.generate_message(
                  protocol_id,
                  destination_address,
                  dir_bit, error_code, cmd_code, parameter_data)
            
            RS485_send = print_byte_array_as_spaced_hex(point_sel_command, f"{cmd_code}")
            ser_port.write(point_sel_command)

            # Fixed delay plus transmission delay calculation
            delay_ms(50)
            cmd_delay_time.calculate_transmission_time_ms(point_sel_command)
      
      elif action == "Home":
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.SEL_NO, 1)
            point_sel_command = cmd_generator.generate_message(
                  protocol_id,
                  destination_address,
                  dir_bit, error_code, cmd_code, parameter_data)
            
            RS485_send = print_byte_array_as_spaced_hex(point_sel_command, f"{cmd_code}")
            ser_port.write(point_sel_command)

            # Fixed delay plus transmission delay calculation
            delay_ms(50)
            cmd_delay_time.calculate_transmission_time_ms(point_sel_command)
            
      elif action == "motionStart":
            print("START")
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.START1, 0)
            motion_start_command = cmd_generator.generate_message(
                  protocol_id,
                  destination_address,
                  dir_bit, error_code, cmd_code, parameter_data)
            RS485_send = print_byte_array_as_spaced_hex(motion_start_command, f"{cmd_code}")
            ser_port.write(motion_start_command)

            delay_ms(50)
            cmd_delay_time.calculate_transmission_time_ms(motion_start_command)

            parameter_data = setter.set_bit_status(BitMap.START1, 1)
            motion_start_command = cmd_generator.generate_message(
                  protocol_id,
                  destination_address,
                  dir_bit, error_code, cmd_code, parameter_data)
            RS485_send = print_byte_array_as_spaced_hex(motion_start_command, f"{cmd_code}")
            ser_port.write(motion_start_command)

            delay_ms(50)
            cmd_delay_time.calculate_transmission_time_ms(motion_start_command)

      elif action == "motionPause":
            print("PAUSE")
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.PAUSE, pause_toggle_bit)
            pause_toggle_bit ^= 1
            motion_start_command = cmd_generator.generate_message(
                  protocol_id,
                  destination_address,
                  dir_bit, error_code, cmd_code, parameter_data)
            RS485_send = print_byte_array_as_spaced_hex(motion_start_command, f"{cmd_code}")
            ser_port.write(motion_start_command)

            delay_ms(50)
            cmd_delay_time.calculate_transmission_time_ms(motion_start_command)

      templateData = {
            'title':'GPIO output Status!',
            'ledRed' : ledRedSts,
            'ledYlw' : ledYlwSts,
            'ledGrn' : ledGrnSts,
            'RS485_read':RS485_read,
            'RS485_send':RS485_send,
      }

      return render_template('index.html', **templateData)

if __name__ == "__main__":
   pwm_red_led = initialize_gpio()
   initialize_serial()
   try:
         app.run(host='0.0.0.0', port=5000, debug = True)
   finally:
         pwm_red_led.stop()
         cleanup_gpio()
         if ser_port:
            ser_port.close()
