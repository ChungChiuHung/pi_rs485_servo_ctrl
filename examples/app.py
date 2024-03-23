import RPi.GPIO as GPIO
import serial
import os
from time import sleep
from flask import Flask, render_template, request, session
from servo_params import ServoParams
from base_msg_generator import BaseMsgGenerator, MessageCommander
from response_parsing import ResponseMsgParser
from command_code import CmdCode
from set_servo_io_status import SetServoIOStatus
from set_servo_io_status import BitMap
from cal_cmd_response_time import CmdDelayTime
from io_status_fetcher import IOStatusFetcher
from serial_communication import SerialCommunication

# Define your GPIO pins upfront
RS485_ENABLE_PIN = 4  # pin for RS485 transmission enable
LED_RED_PIN = 13           # pin assignments for LEDs
LED_YLW_PIN = 19
LED_GRN_PIN = 26

app = Flask(__name__)
app.secret_key = os.getenv('FLASK_SECRET_KEY', 'JOJO0912956011')

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
      global ser_port # Indicate that we're using the global variable
      serial_ports = ["/dev/ttyS0", "/dev/ttyAMA0", "/dev/serial0", "/dev/ttyUSB0"]
      baud_rate = 57600
      timeout = 1

      for port in serial_ports:
            try:
                  ser_port = serial.Serial(port, baud_rate, timeout=timeout)
                  ser_port.bytesize = serial.EIGHTBITS
                  ser_port.parity = serial.PARITY_NONE
                  ser_port.stopbits = serial.STOPBITS_ONE

                  print(f"Successfully connected to {port}")
                  break
            except (OSError, serial.SerialException):
                  continue

      if ser_port is None:
            raise IOError("No available serial port found.")

def delay_ms(milliseconds):
      seconds = milliseconds / 1000.0 # Convert milliseconds to seconds
      sleep(seconds)

def print_byte_array_as_spaced_hex(byte_array, data_name):
    hex_string = ' '.join(f"{byte:02X}" for byte in byte_array)
    print(f"{data_name}: {hex_string}")

@app.route("/")
def index():
      # Initialize session variables if they don't exist
      session.setdefault('SERVO_ON', False)
      session.setdefault('SERVO_OFF', False)
      session.setdefault('GET_MSG', False)
      session.setdefault('GET_IO_OUTPUT', False)
      session.setdefault('SET_POINT_1', False)
      session.setdefault('SET_POINT_2', False)
      session.setdefault('SET_POINT_HOME', False)
      session.setdefault('MOTION_START', False)
      session.setdefault('MOTION_PAUSE', False)

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
        'SERVO_ON' : session['SERVO_ON'],
        'SERVO_OFF' : session['SERVO_OFF'],
        'GET_MSG' : session['GET_MSG'],
        'GET_IO_OUTPUT' : session['GET_IO_OUTPUT'],
        'SET_POINT_1' : session['SET_POINT_1'],
        'SET_POINT_2' : session['SET_POINT_2'],
        'SET_POINT_HOME' : session['SET_POINT_HOME'],
        'MOTION_START' : session['MOTION_START'],
        'MOTION_PAUSE' : session['MOTION_PAUSE'],
      }
      return render_template('index.html', **templateData) 

@app.route("/<deviceName>/<action>")
def action(deviceName, action):
      global RS485_send, RS485_read, pwm_red_led, ser_port
      global SERVO_ON, SERVO_OFF, GET_MSG,GET_IO_OUTPUT
      global SET_POINT_1,SET_POINT_2,SET_POINT_HOME
      global MOTION_START,MOTION_PAUSE
      
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

      # Config the waiting reponse timeout
      delay_time_before_read_ms = 50
      timeout = 1 # Timeout in second

      #fetcher = IOStatusFetcher(ser_port)
      serial_comm = SerialCommunication(ser_port, delay_time_before_read_ms, timeout)
      cmd_delay_time = CmdDelayTime(ser_port.baudrate)
      
      pause_toggle_bit = True
      protocol_id=1
      destination_address = 1
      dir_bit = 0
      error_code = 0


      if action == "servoOn":
            # SET_PARAM_2 command        
            set_param_2_command = ServoParams.SET_PARAM_2      
            serial_comm.send_command_and_wait_for_response(set_param_2_command,
                                                           "SET_PARAM_2")

            # SERVO_ON Command
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.SVON, 1)
            servo_on_command = cmd_generator.generate_message(
                  protocol_id,
                  destination_address,
                  dir_bit, error_code, cmd_code, parameter_data)
            
            serial_comm.send_command_and_wait_for_response(servo_on_command,
                                                           "SERVO_ON")
            session['SERVO_ON'] = True
            session['SERVO_OFF']= False

      elif action == "servoOff":
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.SVON, 0)
            servo_off_command = cmd_generator.generate_message(
                  protocol_id,
                  destination_address,
                  dir_bit, error_code, cmd_code, parameter_data)
            
            serial_comm.send_command_and_wait_for_response(servo_off_command,
                                                           "SERVO_OFF")
            
            session['SERVO_ON'] = False
            session['SERVO_OFF']= True
      
      elif action == "getMsg":
            session['GET_MSG'] = True
            print("GET_MSG: ",GET_MSG)
            get_state_value_command = ServoParams.GET_STATE_VALUE_4
            result, response_received = serial_comm.send_command_and_wait_for_response(get_state_value_command,
                                                           "GET_STATE_VALUE_4")

            session['GET_MSG'] = not response_received

            try:
                  parsed_data = parser.parse_message(result)
                  print("Parsed response data:", parsed_data)
                  RS485_read = parsed_data

            except ValueError as e:
                  print("Error parsing response message:", e)

      elif action == "getIOOutput":
            session['GET_IO_OUTPUT'] = True
            # RS485_read = fetcher.get_output_io_status()
            # print(RS485_read)
            result, response_received = serial_comm.get_output_io_status()

            session['GET_IO_OUTPUT'] = not response_received

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

            SET_POINT_1 = True
            SET_POINT_2 = False
            SET_POINT_HOME = False

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

            SET_POINT_1 = False
            SET_POINT_2 = True
            SET_POINT_HOME = False
      
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

            SET_POINT_1 = False
            SET_POINT_2 = False
            SET_POINT_HOME = True
            
      elif action == "motionStart":
            MOTION_START = True
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

            MOTION_START = False

      elif action == "motionPause":
            print("PAUSE")
            MOTION_PAUSE = pause_toggle_bit
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
            'SERVO_ON' : session['SERVO_ON'],
            'SERVO_OFF' : session['SERVO_OFF'],
            'GET_MSG' : session['GET_MSG'],
            'GET_IO_OUTPUT' : session['GET_IO_OUTPUT'],
            'SET_POINT_1' : session['SET_POINT_1'],
            'SET_POINT_2' : session['SET_POINT_2'],
            'SET_POINT_HOME' : session['SET_POINT_HOME'],
            'MOTION_START' : session['MOTION_START'],
            'MOTION_PAUSE' : session['MOTION_PAUSE'],
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
