import os
import time
import logging
import threading
import traceback
from datetime import datetime, timezone
from functools import wraps

from flask import Flask, render_template, request, jsonify, Response

from serial_port_manager import SerialPortManager
from servo_control import ServoController
from motor_profile import load_profiles, resolve_profile
from hardware_lock import hardware_serialized

# GPIO only imports successfully on real Raspberry Pi hardware (RPi.GPIO).
# Made optional so this app can be developed/tested off-Pi (e.g. this
# Windows dev machine over a USB-RS485 adapter) -- see
# docs/servo_comm_shihlin_merge_design.md's web-UI plan, §2.1.
try:
    from gpio_utils import GPIOUtils
except ImportError:
    GPIOUtils = None

app = Flask(__name__)
app.secret_key = os.getenv('FLASK_SECRET_KEY', 'your keys')

# Dedicated logger for the /alarm/clear endpoint (CLAUDE.md hardware-safety
# section: every call that can write live state to the motor must be logged).
alarm_logger = logging.getLogger("alarm_clear")

gpio_utils = None
if GPIOUtils is not None:
    try:
        gpio_utils = GPIOUtils()
    except Exception as e:
        logging.warning(f"GPIO init failed ({e}); GPIO features disabled.")
else:
    logging.warning("RPi.GPIO not available (expected off-Pi); GPIO features disabled.")

# --- Motor profile / serial connection state ---------------------------------
# Single process, single ServoController instance at a time -- this is also
# what keeps a future OSC/Art-Net server (Milestone 3/4) from needing its own
# serial connection: it will call methods on this same servo_ctrller.
_state_lock = threading.RLock()
_profiles_data = load_profiles()
current_profile_name = _profiles_data["active_profile"]
serial_manager = None
servo_ctrller = None
# None | "osc" | "artnet" -- set by the Milestone 3/4 server-lifecycle
# endpoints. Declared here so /profile's mutual-exclusion check already
# works correctly once those endpoints exist, without revisiting this file.
active_input_server = None


def _connect_profile(profile_name: str) -> None:
    """(Re)open the serial port for profile_name and rebuild ServoController
    against it. Raises RuntimeError if the port can't be opened. Caller must
    hold _state_lock.

    Closes any existing connection FIRST: both profiles share the same
    physical port (just at a different baud rate), so a second
    SerialPortManager can't open it while the first is still holding it
    exclusively. This does mean a failed reconnect leaves the app
    disconnected rather than falling back to the previous profile --
    acceptable here since there's only one physical port to share.
    """
    global serial_manager, servo_ctrller, current_profile_name

    profile = resolve_profile(_profiles_data, profile_name)

    if serial_manager is not None:
        serial_manager.disconnect()
        serial_manager = None
        servo_ctrller = None

    new_serial_manager = SerialPortManager(baud_rate=profile["baud_rate"])
    new_serial_manager.connect()
    if not new_serial_manager.get_serial_instance():
        raise RuntimeError(
            f"Could not open a serial port at {profile['baud_rate']} baud "
            f"for profile '{profile_name}'."
        )

    serial_manager = new_serial_manager
    servo_ctrller = ServoController(serial_manager, profile)
    current_profile_name = profile_name
    logging.info(
        f"Active profile: {profile_name} -- connected "
        f"{serial_manager.get_connected_port()} @ {profile['baud_rate']} baud"
    )


with _state_lock:
    _connect_profile(current_profile_name)

# Kept for template/JS compatibility with servo_comm_shihlin's index.html;
# handle_action() doesn't actually populate these from real RS485 traffic
# (neither does the version this was ported from).
RS485_send, RS485_read = "00 00 FF FF", "FF FF 00 00"


def json_response(f):
    @wraps(f)
    def decorated_function(*args, **kwargs):
        try:
            result = f(*args, **kwargs)
            if isinstance(result, (Response, tuple)):
                return result
            return jsonify(result)
        except Exception as e:
            traceback.print_exc()
            return jsonify({"error": "An error occurred", "details": str(e)}), 500
    return decorated_function


@app.route('/')
def home():
    return render_template('home.html')


@app.route('/index')
def index():
    return render_template(
        'index.html', title='Servo Control Panel',
        RS485_read=RS485_read, RS485_send=RS485_send
    )


@app.route('/profile', methods=['GET'])
def get_profile():
    return jsonify({
        "active_profile": current_profile_name,
        "available_profiles": list(_profiles_data["profiles"].keys()),
    })


@app.route('/status', methods=['GET'])
def get_status():
    """Read-only status snapshot for the web UI's live feedback panel.
    current_angle/current_encoder/reading_active are read straight from the
    ServoController's already-tracked state (zero extra serial traffic);
    alarm_code is a fresh read each call (locked against the
    continuous-reading thread inside read_current_alarm_code()). Never
    triggers any write/motion command -- safe to poll on an interval.
    """
    alarm_code = servo_ctrller.read_current_alarm_code()
    return jsonify({
        "profile": current_profile_name,
        "connected_port": serial_manager.get_connected_port(),
        "baud_rate": serial_manager.get_baud_rate(),
        "reading_active": servo_ctrller.reading_active,
        "current_angle": servo_ctrller.current_angle,
        "current_encoder": servo_ctrller.current_encoder,
        "alarm_code": alarm_code,
    })


@app.route('/profile', methods=['POST'])
@hardware_serialized
def set_profile():
    payload = request.get_json(silent=True) or {}
    requested = payload.get("profile")

    if requested not in _profiles_data["profiles"]:
        return jsonify({
            "status": "error",
            "message": f"Unknown profile: {requested!r}. Available: "
                       f"{list(_profiles_data['profiles'].keys())}",
        }), 400

    if active_input_server is not None:
        return jsonify({
            "status": "error",
            "message": f"Cannot switch profile while the '{active_input_server}' "
                       "server is running. Stop it first.",
        }), 409

    if requested == current_profile_name:
        return jsonify({"status": "success", "active_profile": current_profile_name})

    with _state_lock:
        try:
            _connect_profile(requested)
        except Exception as e:
            logging.error(f"Failed to switch to profile '{requested}': {e}")
            return jsonify({"status": "error", "message": str(e)}), 503

    return jsonify({"status": "success", "active_profile": current_profile_name})


@app.route('/alarm/clear', methods=['POST'])
@hardware_serialized
def clear_alarm_12_endpoint():
    """Clear Alarm 12 (AL.12, Emergency stop) only. Not a general-purpose
    Modbus write endpoint -- see README for the safety precondition this
    requires before calling it.
    """
    caller_ip = request.remote_addr
    started_at = datetime.now(timezone.utc).isoformat()

    payload = request.get_json(silent=True) or {}
    if payload.get('confirm') is not True:
        alarm_logger.warning(
            "Rejected /alarm/clear from %s at %s: missing confirm=true.",
            caller_ip, started_at
        )
        return jsonify({
            "status": "error",
            "message": 'Missing or false "confirm" field. POST {"confirm": true} to proceed.'
        }), 400

    before_code = servo_ctrller.read_current_alarm_code()
    alarm_logger.info(
        "Alarm-12 clear requested by %s at %s. Alarm code before: %s",
        caller_ip, started_at, before_code
    )

    if before_code is None:
        alarm_logger.error(
            "Could not read alarm status before clearing (comm failure) -- "
            "aborting, no clear command was sent."
        )
        return jsonify({
            "status": "error",
            "message": "Could not read current alarm status (communication failure). No clear command was sent.",
            "before_alarm_code": None,
            "caller_ip": caller_ip,
            "timestamp": started_at,
        }), 503

    # Primary mechanism: clear_alarm_12(). NOTE -- this switches the drive's
    # DI control source to communication mode (PD16) and writes the virtual
    # EMG DI bit to its "released" state. If a physical E-Stop circuit is
    # still engaged, this can make the drive treat EMG as released without
    # the physical condition actually being resolved. See README "Alarm 12
    # clear -- safety precondition" before using this.
    servo_ctrller.write_PD_16_Enable_DI_Control()
    servo_ctrller.clear_alarm_12()
    time.sleep(0.1)
    after_code = servo_ctrller.read_current_alarm_code()
    mechanism_used = "clear_alarm_12"

    if after_code != 0:
        # Fallback: official 0x0130 "Alarm clearance" register (write
        # 0x1EA5). Does not touch DI control source / virtual EMG state.
        servo_ctrller.clear_alarm_via_register()
        time.sleep(0.1)
        after_code = servo_ctrller.read_current_alarm_code()
        mechanism_used = "clear_alarm_12+0x0130_fallback"

    success = after_code == 0
    alarm_logger.info(
        "Alarm-12 clear result for %s: before=%s after=%s success=%s mechanism=%s",
        caller_ip, before_code, after_code, success, mechanism_used
    )

    return jsonify({
        "status": "success" if success else "failed",
        "before_alarm_code": before_code,
        "after_alarm_code": after_code,
        "mechanism_used": mechanism_used,
        "caller_ip": caller_ip,
        "timestamp": started_at,
    }), (200 if success else 502)


@app.route('/action', methods=['POST'])
@json_response
@hardware_serialized
def handle_action():
    data = request.json
    action = data.get('action')

    print(f"Received action: {action}")

    # Enable the Digital I/O Writable
    servo_ctrller.write_PD_16_Enable_DI_Control()

    if action == "start":
        print("start")
    elif action == "stop":
        print("stop")
    elif action == "servoOn":
        servo_ctrller.clear_alarm_12()
        time.sleep(0.1)
        servo_ctrller.servo_on()
        time.sleep(0.1)
    elif action == "servoOff":
        servo_ctrller.servo_off()
    elif action == "getMsg":
        servo_ctrller.Read_Pos_Related_Paremters()
    elif action == "clearAlarm12":
        servo_ctrller.clear_alarm_12()
    elif action == "enablePosMode":
        servo_ctrller.Enable_Position_Mode(True)
        time.sleep(0.05)
        servo_ctrller.config_acc_dec_0x0902(0)
        time.sleep(0.05)
        servo_ctrller.config_speed_0x0903(10)
        time.sleep(0.05)
        servo_ctrller.config_pulses_0x0905_low_byte(0x0000)
        time.sleep(0.05)
        servo_ctrller.config_pulses_0x0906_high_byte(0x0780)
        time.sleep(0.05)
        servo_ctrller.start_continuous_reading(0.1)
    elif action == "posTestStart_CW":
        servo_ctrller.pos_step_motion_test(CW=True)
    elif action == "posTestStart_CCW":
        servo_ctrller.pos_step_motion_test(CW=False)
    elif action == "setPoint_1":
        servo_ctrller.post_step_motion_by(90)
    elif action == "setPoint_2":
        servo_ctrller.post_step_motion_by(180)
    elif action == "Home":
        servo_ctrller.post_step_motion_by(0)
    elif action == "enableSpeedCtrlMode":
        servo_ctrller.enable_speed_ctrl(100)
    elif action == "motionStart_CW":
        servo_ctrller.speed_ctrl_action(1)
    elif action == "motionStart_CCW":
        servo_ctrller.speed_ctrl_action(2)
    elif action == "motionPause":
        servo_ctrller.speed_ctrl_action(0)
    elif action == "motionCancel":
        servo_ctrller.stop_continuous_reading()
        servo_ctrller.Enable_Position_Mode(False)
    else:
        return jsonify({
            "status": "error",
            "action": action,
            "message": f"Action '{action}' not recognized.",
        }), 400

    return jsonify({
        "status": "success",
        "action": action,
        "RS485_send": RS485_send,
        "RS485_read": RS485_read,
        "message": f"Action {action} completed successfully.",
    })


if __name__ == "__main__":
    try:
        # use_reloader=False: the reloader re-executes this module in a
        # second process, which would try to open the serial port (and
        # initialize GPIO) twice.
        app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
    finally:
        if gpio_utils is not None:
            gpio_utils.cleanup_gpio()
