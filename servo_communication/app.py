import os
from flask import Flask, render_template, request, jsonify, Response
from functools import wraps
import traceback
from gpio_utils import GPIOUtils
from serial_port_manager import SerialPortManager
from servo_command_code import CmdCode
from status_bit_mapping import BitMap
from servo_serial_protocol_handler import SerialProtocolHandler
from servo_control import ServoController


app = Flask(__name__)
app.secret_key = os.getenv('FLASK_SECRET_KEY', 'your keys')


# Instantiate GPIOUtils for GPIO opeartions
gpio_utils = GPIOUtils()

# Configure the serial port
port_manager = SerialPortManager(baud_rate=57600)
port_manager.connect()
if port_manager.get_serial_instance():
      print(f"Connected port: {port_manager.get_connected_port()}")
      print(f"Current baud rate: {port_manager.get_baud_rate()}")
      servo_ctrller = ServoController(port_manager.get_serial_instance())
else:
      print("Could not configure any serial port. Exiting.")
      exit()

# Servo Command Handler
command_format = SerialProtocolHandler()

# Initailize global variables for RS485 messages

START, STOP = False, False
RS485_send, RS485_read = "00 00 FF FF", "FF FF 00 00"

monitoring_active = False
monitoring_thread = None

def convert_bytes_to_hex(data):
      return data.hex() if isinstance(data, bytes) else data

def json_response(f):
      @wraps(f)
      def decorated_function(*args, **kwargs):
            try:
                  result = f(*args, **kwargs)
                  if isinstance(result, Response):
                        return result
                  return jsonify(Response)
            except Exception as e:
                  traceback.print_exc()
                  return jsonify({"error": "An error occurred", "details":str(e)}),500
      return decorated_function

@app.route('/')
def home():
      return render_template('home.html')

@app.route('/index')
def index():
      return render_template('index.html', title='Servo Control Panel', RS485_read=RS485_read, RS485_send=RS485_send)

@app.route('/action', methods=['POST'])
@json_response
def handle_action():
      global START, STOP, RS485_send, RS485_read

      data = request.json
      action = data.get('action')
      response = {"status":"success","action":action}

      print(f"Received action: {action}")

      # Perform the raspi action here based on action type

      if action == "start":
            points=[3,4]
            # SET_PARM_2 command
            servo_ctrller.send_servo_command(CmdCode.SET_PARAM_2, b'\x00\x09\x00\x01')
            # SERVO ON
            servo_ctrller.send_servo_command(CmdCode.SET_STATE_VALUE_WITHMASK_4, b'\x01\x20\x00\x00\x00\x01\x00\x00\x00\x01')
            servo_ctrller.start_motion_sequence(points)
            response['message'] = "Motion sequence started."
      elif action == "stop":
            servo_ctrller.stop_motion_sequence()
            response['message'] = "Motion sequence stopped."
      elif action == "servoOn":
            # SET_PARAM_2 command        
            command_code = CmdCode.SET_PARAM_2
            set_param_2_command = command_format.construct_packet(1,command_code, b'\x00\x09\x00\x01', is_response=False)
            servo_ctrller.send_command_and_wait_for_response(set_param_2_command, f"{command_code.name}", 0.05)
            RS485_send = servo_ctrller.last_send_message
            RS485_read = servo_ctrller.last_received_message

            # SERVO_ON Command
            command_code = CmdCode.SET_STATE_VALUE_WITHMASK_4
            servo_on_command = command_format.construct_packet(1,command_code, b'\x01\x20\x00\x00\x00\x01\x00\x00\x00\x01', is_response=False)
            servo_ctrller.send_command_and_wait_for_response(servo_on_command, f"{command_code.name}", 0.05)
  
            response['message']="Servo turned on successfully."

      elif action == "servoOff":
            command_code = CmdCode.SET_STATE_VALUE_WITHMASK_4
            servo_off_command = command_format.construct_packet(1,command_code, b'\x01\x20\x00\x00\x00\x00\x00\x00\x00\x01', is_response=False)
            servo_ctrller.send_command_and_wait_for_response(servo_off_command, f"{command_code.name}", 0.05)
            RS485_send = servo_ctrller.last_send_message
            RS485_read = servo_ctrller.last_received_message
            
            response['message'] = "Servo turned off successfully."
      
      elif action == "getMsg":

            # Alarm
            command_code = CmdCode.GET_STATE_VALUE_4
            get_io_alarm_state = command_format.construct_packet(1,command_code, b'\x00\x00', is_response=False)
            servo_ctrller.send_command_and_wait_for_response(get_io_alarm_state, f"{command_code.name}", 0.05)
            RS485_send = servo_ctrller.last_send_message
            RS485_read = servo_ctrller.last_received_message

            # Logic I/O Input
            command_code = CmdCode.GET_STATE_VALUE_4
            get_io_input_state = command_format.construct_packet(1,command_code, b'\x01\x20', is_response=False)
            servo_ctrller.send_command_and_wait_for_response(get_io_input_state, f"{command_code.name}", 0.05)
            RS485_send = servo_ctrller.last_send_message
            RS485_read = servo_ctrller.last_received_message

            # Logic I/O Output
            command_code = CmdCode.GET_STATE_VALUE_4
            get_io_output_state = command_format.construct_packet(1,command_code, b'\x01\x28', is_response=False)
            servo_ctrller.send_command_and_wait_for_response(get_io_output_state, f"{command_code.name}", 0.05)
            RS485_send = servo_ctrller.last_send_message
            RS485_read = servo_ctrller.last_received_message

            response['message'] = "Get Servo IO Input Status Value."

      elif action == "getIOOutput":

            # Logic I/O Output
            command_code = CmdCode.GET_STATE_VALUE_4
            get_io_output_state = command_format.construct_packet(1,command_code, b'\x01\x28', is_response=False)
            servo_ctrller.send_command_and_wait_for_response(get_io_output_state, f"{command_code.name}", 0.05)
            RS485_send = servo_ctrller.last_send_message
            RS485_read = servo_ctrller.last_received_message

            response['message'] = "Get Servo IO Output Status Value."

      elif action == "setPoint_1":

            command_code = CmdCode.SET_STATE_VALUE_WITHMASK_4
            set_point_1 = command_format.construct_packet(1, command_code,b'', BitMap.SEL_NO, 3, is_response=False)
            servo_ctrller.send_command_and_wait_for_response(set_point_1, f"{command_code.name}", 0.05)
            RS485_send = servo_ctrller.last_send_message
            RS485_read = servo_ctrller.last_received_message

            response['message'] = "Set the Postion in Point 1."

      elif action == "setPoint_2":
            command_code = CmdCode.SET_STATE_VALUE_WITHMASK_4
            set_point_2 = command_format.construct_packet(1, command_code,b'', BitMap.SEL_NO, 4, is_response=False)
            servo_ctrller.send_command_and_wait_for_response(set_point_2, f"{command_code.name}", 0.05)
            RS485_send = servo_ctrller.last_send_message
            RS485_read = servo_ctrller.last_received_message

            response['message'] = "Set the Postion in Point 2."

      elif action == "Home":
            command_code = CmdCode.SET_STATE_VALUE_WITHMASK_4
            set_home_position = command_format.construct_packet(1, command_code,b'', BitMap.SEL_NO, 1, is_response=False)
            servo_ctrller.send_command_and_wait_for_response(set_home_position, f"{command_code.name}", 0.05)
            RS485_send = servo_ctrller.last_send_message
            RS485_read = servo_ctrller.last_received_message

            response['message'] = "Set the Postion in HOME."
            
      elif action == "motionStart":
            print("START")
            command_code = CmdCode.SET_STATE_VALUE_WITHMASK_4
            set_home_position = command_format.construct_packet(1, command_code,b'', BitMap.START1, 0, is_response=False)
            servo_ctrller.send_command_and_wait_for_response(set_home_position, f"{command_code.name}", 0.05)
            RS485_send = servo_ctrller.last_send_message
            RS485_read = servo_ctrller.last_received_message

            set_home_position = command_format.construct_packet(1, command_code,b'', BitMap.START1, 1, is_response=False)
            servo_ctrller.send_command_and_wait_for_response(set_home_position, f"{command_code.name}", 0.05)
            RS485_send = servo_ctrller.last_send_message
            RS485_read = servo_ctrller.last_received_message

            response['message'] = "Motion Start."


      elif action == "motionPause":

            command_code = CmdCode.SET_STATE_VALUE_WITHMASK_4
            set_motion_pause = command_format.construct_packet(1, command_code, b'', BitMap.PAUSE, 1)
            servo_ctrller.send_command_and_wait_for_response(set_motion_pause, f"{command_code.name}", 0.05)
            set_motion_pause = command_format.construct_packet(1, command_code, b'', BitMap.PAUSE, 0)
            servo_ctrller.send_command_and_wait_for_response(set_motion_pause, f"{command_code.name}", 0.05)
            RS485_send = servo_ctrller.last_send_message
            RS485_read = servo_ctrller.last_received_message
            response['message'] = "Motion Pause."

      elif action == "motionCancel":
            command_code = CmdCode.SET_STATE_VALUE_WITHMASK_4
            set_motion_pause = command_format.construct_packet(1, command_code, b'', BitMap.CANCEL, 1)
            servo_ctrller.send_command_and_wait_for_response(set_motion_pause, f"{command_code.name}", 0.05)
            set_motion_pause = command_format.construct_packet(1, command_code, b'', BitMap.CANCEL, 0)
            servo_ctrller.send_command_and_wait_for_response(set_motion_pause, f"{command_code.name}", 0.05)
            RS485_send = servo_ctrller.last_send_message
            RS485_read = servo_ctrller.last_received_message
            response['message'] = "Motion Cancel"
      else:
            response['error'] = "Action not recognized."

      RS485_send = convert_bytes_to_hex(RS485_send)
      RS485_read = convert_bytes_to_hex(RS485_read)

      return jsonify({
            "status": "success",
            "action": action,
            "RS485_send": RS485_send,
            "RS485_read": RS485_read,
            "message":f"Action {action} completed successfully."  
      })

if __name__ == "__main__":
   gpio_utils.initialize_gpio()
   try:
         app.run(host='0.0.0.0', port=5000, debug = True)
   finally:
         gpio_utils.cleanup_gpio()
