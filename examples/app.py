import os
from flask import Flask, render_template, request, session, jsonify
from gpio_utils import GPIOUtils
from servo_params import ServoParams
from base_msg_generator import BaseMsgGenerator, MessageCommander
from response_parsing import ResponseMsgParser
from command_code import CmdCode
from set_servo_io_status import SetServoIOStatus
from set_servo_io_status import BitMap
from cal_cmd_response_time import CmdDelayTime
from io_status_fetcher import IOStatusFetcher
from serial_communication import SerialCommunication, SerialPortHandler

app = Flask(__name__)
app.secret_key = os.getenv('FLASK_SECRET_KEY', 'JOJO0912956011')


# Instantiate GPIOUtils and SerialPortHandler
gpio_utils = GPIOUtils()
serial_port_handler = SerialPortHandler(baud_rate=57600, timeout=1)
if not serial_port_handler.open_serial():
      print("Failed to open serial port. Existing...")
      exit(1)

# Initialize Serial Communication with the port handler
serial_comm = SerialCommunication(port_handler = serial_port_handler, command_timeout=2)

# Initailize global variables for RS485 messages
RS485_send = ''
RS485_read = ''
ser_port = serial_port_handler

SERVO_ON = False
SERVO_OFF = False
GET_MSG = False
GET_IO_OUTPUT = False
SET_POINT_1 = False
SET_POINT_2 = False
SET_POINT_HOME = False
MOTION_START = False
MOTION_PAUSE = False

cmd_generator = BaseMsgGenerator()
setter = SetServoIOStatus()
parser = ResponseMsgParser()

# Config the waiting reponse timeout
delay_time_before_read_ms = 50
timeout = 1 # Timeout in second

#fetcher = IOStatusFetcher(ser_port)
cmd_delay_time = CmdDelayTime(ser_port.get_baud_rate)
      
pause_toggle_bit = True
protocol_id=1
destination_address = 1
dir_bit = 0
error_code = 0

@app.route('/')
def home():
      return render_template('home.html')

@app.route('/action', methods=['POST'])
def handle_action():
      action_type = request.get_json('action')
      response = {}
      print(f"Received action: {action_type}")
      # Perform the raspi action here based on action type

      if action_type == "servoOn":
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
            response['message']="Servo turned on successfully."

      elif action_type == "servoOff":
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.SVON, 0)
            servo_off_command = cmd_generator.generate_message(
                  protocol_id,
                  destination_address,
                  dir_bit, error_code, cmd_code, parameter_data)
            
            serial_comm.send_command_and_wait_for_response(servo_off_command,
                                                           "SERVO_OFF")
            
            response['message'] = "Servo turned off successfully."
      
      elif action_type == "getMsg":
            get_state_value_command = ServoParams.GET_STATE_VALUE_4
            result, response_received = serial_comm.send_command_and_wait_for_response(get_state_value_command,
                                                           "GET_STATE_VALUE_4")

            try:
                  parsed_data = parser.parse_message(result)
                  print("Parsed response data:", parsed_data)
                  RS485_read = parsed_data

            except ValueError as e:
                  print("Error parsing response message:", e)

            response['message'] = "Get Servo IO Input Status Value."

      elif action_type == "getIOOutput":
            # RS485_read = fetcher.get_output_io_status()
            # print(RS485_read)
            result, response_received = serial_comm.get_output_io_status()

            response['message'] = "Get Servo IO Output Status Value."

      elif action_type == "setPoint_1":
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.SEL_NO, 5)
            point_sel_command = cmd_generator.generate_message(
                   protocol_id,
                   destination_address,
                   dir_bit, error_code, cmd_code, parameter_data)
            serial_comm.send_command_and_wait_for_response(point_sel_command,
                                                           "SEL_POINT_1")

            response['message'] = "Set the Postion in Point 1."

      elif action_type == "setPoint_2":
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.SEL_NO, 6)
            point_sel_command = cmd_generator.generate_message(
                  protocol_id,
                  destination_address,
                  dir_bit, error_code, cmd_code, parameter_data)
            
            serial_comm.send_command_and_wait_for_response(point_sel_command,
                                                           "SEL_POINT_2")

            response['message'] = "Set the Postion in Point 2."

      elif action_type == "Home":
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.SEL_NO, 1)
            point_sel_command = cmd_generator.generate_message(
                  protocol_id,
                  destination_address,
                  dir_bit, error_code, cmd_code, parameter_data)
            
            serial_comm.send_command_and_wait_for_response(point_sel_command,
                                                           "SEL_POINT_HOME")
            
            response['message'] = "Set the Postion in HOME."
            
      elif action_type == "motionStart":
            print("START")
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.START1, 0)
            motion_start_command = cmd_generator.generate_message(
                  protocol_id,
                  destination_address,
                  dir_bit, error_code, cmd_code, parameter_data)
            
            serial_comm.send_command_and_wait_for_response(motion_start_command,
                                                           "MOTION_START")
            

            parameter_data = setter.set_bit_status(BitMap.START1, 1)
            motion_start_command = cmd_generator.generate_message(
                  protocol_id,
                  destination_address,
                  dir_bit, error_code, cmd_code, parameter_data)
            
            serial_comm.send_command_and_wait_for_response(motion_start_command,
                                                           "MOTION_START")
            response['message'] = "Motion Start."


      elif action_type == "motionPause":
            temp = pause_toggle_bit
            print("PAUSE: ",temp)
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.PAUSE, temp)
            pause_toggle_bit = not temp
            print("PAUSE: ",pause_toggle_bit)
            
            motion_puase_command = cmd_generator.generate_message(
                  protocol_id,
                  destination_address,
                  dir_bit, error_code, cmd_code, parameter_data)
            
            serial_comm.send_command_and_wait_for_response(motion_puase_command,
                                                           "MOTION_PAUSE")
            response['message'] = "Motion Pause."
      else:
            response['error'] = "Action not recognized."

      return jsonify(response)

@app.route('/index')
def index():
      templateData = {
        'title': 'Servo Control Panel',
        'RS485_read': serial_comm.last_send_message,
        'RS485_send': serial_comm.last_received_message,
        'SERVO_ON' : SERVO_ON,
        'SERVO_OFF' : SERVO_OFF,
        'GET_MSG' : GET_MSG,
        'GET_IO_OUTPUT' : GET_IO_OUTPUT,
        'SET_POINT_1' : SET_POINT_1,
        'SET_POINT_2' : SET_POINT_2,
        'SET_POINT_HOME' : SET_POINT_HOME,
        'MOTION_START' : MOTION_START,
        'MOTION_PAUSE' : MOTION_PAUSE,
      }
      return render_template('index.html', **templateData) 

if __name__ == "__main__":
   gpio_utils.initialize_gpio()
   serial_port_handler.open_serial()
   try:
         app.run(host='0.0.0.0', port=5000, debug = True)
   finally:
         gpio_utils.cleanup_gpio()
         serial_port_handler.close_serial()
