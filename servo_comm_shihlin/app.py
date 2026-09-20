import hmac
import os
import time
import logging
import threading
from datetime import datetime, timezone
from flask import Flask, render_template, request, jsonify, Response, redirect
from functools import wraps
import traceback
from serial_port_manager import SerialPortManager
from servo_control import ServoController, PositionUnavailableError, is_alarm_active
from hardware_lock import hardware_serialized
from input_validation import validate_int_range
from activity_log import ActivityLog, ActivityLogHandler
from osc_server import OSCInputServer

# GPIO only imports on a real Raspberry Pi (RPi.GPIO); optional so the app can
# also run on a PC/Mac with a USB-RS485 adapter.
try:
    from gpio_utils import GPIOUtils
except ImportError:
    GPIOUtils = None

app = Flask(__name__)
app.secret_key = os.getenv('FLASK_SECRET_KEY', 'your keys')

# The web UI can move a motor and set home, and has no login of its own.
# Setting SERVO_WEB_PASSWORD turns on HTTP Basic auth for every route (user:
# SERVO_WEB_USER, default "servo"). Off by default so local development keeps
# working; the startup log warns when the UI is reachable from the network
# without it.
WEB_USER = os.getenv('SERVO_WEB_USER', 'servo')
WEB_PASSWORD = os.getenv('SERVO_WEB_PASSWORD', '')


@app.before_request
def _require_web_password():
    if not WEB_PASSWORD:
        return None
    auth = request.authorization
    if (auth is not None
            and hmac.compare_digest((auth.username or '').encode('utf-8'), WEB_USER.encode('utf-8'))
            and hmac.compare_digest((auth.password or '').encode('utf-8'), WEB_PASSWORD.encode('utf-8'))):
        return None
    return Response('Authentication required.', 401,
                    {'WWW-Authenticate': 'Basic realm="Servo Control"'})


# Dedicated logger for the /alarm/clear endpoint (CLAUDE.md hardware-safety
# section: every call that can write live state to the motor must be logged).
alarm_logger = logging.getLogger("alarm_clear")

# Web UI activity feed for a user who isn't watching this process's own
# console -- captures every logging call across the codebase (see
# activity_log.py), including what OSC messages did. Attached before anything
# else runs so startup messages are captured too.
activity_log = ActivityLog()
_activity_log_handler = ActivityLogHandler(activity_log)
_activity_log_handler.setFormatter(logging.Formatter('%(message)s'))
logging.getLogger().addHandler(_activity_log_handler)

# Instantiate GPIOUtils for GPIO opeartions
gpio_utils = None
if GPIOUtils is not None:
    try:
        gpio_utils = GPIOUtils()
    except Exception as e:
        logging.warning(f"GPIO init failed ({e}); GPIO features disabled.")
else:
    logging.warning("RPi.GPIO not available (expected off-Pi); GPIO features disabled.")

# --- Serial connection state ---------------------------------------------------
_state_lock = threading.RLock()
serial_manager = None
servo_ctrller = None
# Why the serial port is not open (None while connected). The web UI still
# starts without a port so the operator can see this and retry.
_connection_error = None
# None | "osc" -- the OSC server started from the web UI, and its object.
active_input_server = None
_input_server_instance = None


def _connect() -> None:
    """(Re)open the serial port and build the ServoController. Raises
    RuntimeError if the port can't be opened (the reason is also kept in
    _connection_error). Caller must hold _state_lock."""
    global serial_manager, servo_ctrller, _connection_error

    if serial_manager is not None:
        serial_manager.disconnect()
        serial_manager = None
        servo_ctrller = None

    # SERVO_SERIAL_PORT pins the port (e.g. /dev/cu.usbserial-XXXX on macOS,
    # set by start_server.command); unset = auto-detect.
    new_manager = SerialPortManager(port=os.getenv('SERVO_SERIAL_PORT') or None)
    try:
        if not new_manager.get_serial_instance():
            raise RuntimeError("Could not configure any serial port.")
        new_controller = ServoController(new_manager)
    except Exception as e:
        # Don't leave a half-opened port behind: it would block the retry.
        try:
            new_manager.disconnect()
        except Exception:
            pass
        _connection_error = str(e)
        raise

    serial_manager = new_manager
    servo_ctrller = new_controller
    _connection_error = None
    logging.info(f"Connected port: {serial_manager.get_connected_port()} "
                 f"at {serial_manager.get_baud_rate()} baud")
    # Read-only: angle and positioning math assume a 1:1 electronic gear ratio
    # (PA06 = PA07); warn loudly if the drive says otherwise.
    try:
        servo_ctrller.check_electronic_gear_ratio()
    except Exception as e:
        logging.warning(f"Could not check the electronic gear ratio at connect time ({e}).")


with _state_lock:
    try:
        _connect()
    except Exception as e:
        # No serial port must not stop the web UI from starting: it shows the
        # problem and offers a retry (POST /reconnect). Nothing can move the
        # motor without a connection -- see _require_serial_connection().
        logging.error(
            f"Starting WITHOUT a serial connection ({e}). The web UI is up but motor "
            "control is unavailable until the port is connected and Reconnect is pressed."
        )

# Endpoints that work without a serial connection: the page itself, the
# read-only status/log views, and the way to (re)connect.
_ENDPOINTS_WITHOUT_SERIAL = frozenset({
    'static', 'home', 'index', 'get_status', 'get_activity_log',
    'get_input_server_status', 'reconnect_serial',
})


@app.before_request
def _require_serial_connection():
    if servo_ctrller is not None or request.endpoint is None \
            or request.endpoint in _ENDPOINTS_WITHOUT_SERIAL:
        return None
    return jsonify({
        "status": "error",
        "connected": False,
        "message": "No RS-485 serial port connection"
                   + (f" ({_connection_error})" if _connection_error else "")
                   + ". Nothing was sent to the drive. Connect the adapter, then press Reconnect.",
    }), 503


def _current_rs485_traffic():
    """The most recent raw Modbus ASCII frames, for the web UI's "Last RS-485
    transaction" boxes (empty until the first transaction, or while there is
    no serial connection)."""
    controller = servo_ctrller
    if controller is None:
        return "", ""
    client = controller.modbus_client
    return client.format_frame(client.last_sent), client.format_frame(client.last_received)


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
    # home.html was an unused "Walking Lamp Control" test page (a START/STOP
    # pair wired to no-op actions); index.html is the control panel.
    return redirect('/index')


@app.route('/index')
def index():
    rs485_send, rs485_read = _current_rs485_traffic()
    return render_template('index.html', title='Servo Control Panel',
                           RS485_read=rs485_read, RS485_send=rs485_send)


@app.route('/status', methods=['GET'])
def get_status():
    """Read-only status snapshot for the web UI. Uses only state the
    ServoController already tracks (no serial traffic of its own, so it
    cannot interleave with a command or the continuous-reading thread -- this
    folder has no serial lock for that), and answers {"connected": false, ...}
    with the reason while there is no serial connection."""
    controller = servo_ctrller
    if controller is None:
        return jsonify({
            "connected": False,
            "connection_error": _connection_error,
            "connected_port": None,
            "baud_rate": None,
        })
    return jsonify({
        "connected": True,
        "connected_port": serial_manager.get_connected_port(),
        "baud_rate": serial_manager.get_baud_rate(),
        "reading_active": controller.reading_active,
        "current_angle": controller.current_angle,
        "current_encoder": controller.current_encoder,
        # None means "never recorded" -- see ServoController.record_set_point().
        "set_point_1": controller.set_point_1,
        "set_point_2": controller.set_point_2,
        # PA06/PA07 as [cmx, cdv] (None = unread); the angle and positioning
        # math assume 1:1, electronic_gear_ok is False when the drive disagrees.
        "electronic_gear": list(controller.electronic_gear) if controller.electronic_gear else None,
        "electronic_gear_ok": controller.electronic_gear_unity,
        # SET HOME has succeeded since this process started. The incremental
        # counter restarts at every drive power-on, so the UI asks the
        # operator to SET HOME after each start.
        "home_set_since_start": controller.home_set_since_start,
        "home_reminder_needed": not controller.home_set_since_start,
    })


@app.route('/log', methods=['GET'])
def get_activity_log():
    """Incremental activity feed for a user not watching this process's own
    console -- pass `since` (an entry id from a previous call) to get only
    what's new. Never triggers any write/motion command."""
    since = request.args.get('since', default=0, type=int) or 0
    return jsonify({"entries": activity_log.get_since(since)})


@app.route('/reconnect', methods=['POST'])
@hardware_serialized
def reconnect_serial():
    """Retries opening the serial port after the app started (or lost the
    connection) without one. Read-only on the drive apart from what a normal
    start does; it never moves the motor."""
    if servo_ctrller is not None:
        return jsonify({"status": "success", "connected": True, "message": "Already connected."})
    with _state_lock:
        try:
            _connect()
        except Exception as e:
            logging.error(f"Reconnect failed: {e}")
            return jsonify({"status": "error", "connected": False, "message": str(e)}), 503
    return jsonify({"status": "success", "connected": True})


@app.route('/server/status', methods=['GET'])
def get_input_server_status():
    return jsonify({
        "active_input_server": active_input_server,
        "is_running": _input_server_instance.is_running if _input_server_instance else False,
    })


@app.route('/server/start', methods=['POST'])
def start_input_server():
    """Starts the OSC server (the handlers osc.py runs as a separate script)
    as a background thread against this app's own ServoController. It does
    not itself send anything to the driver; hardware I/O only happens later,
    if and when a message actually arrives -- the same as any /action click."""
    global active_input_server, _input_server_instance

    if active_input_server is not None:
        return jsonify({
            "status": "error",
            "message": f"'{active_input_server}' server is already running. Stop it first.",
        }), 409

    payload = request.get_json(silent=True) or {}
    server_type = payload.get("type", "osc")
    if server_type != "osc":
        return jsonify({"status": "error",
                        "message": f"Unknown server type: {server_type!r}. Expected 'osc'."}), 400

    listen_ip = payload.get("listen_ip", "0.0.0.0")
    listen_port, error = validate_int_range(payload.get("listen_port", 5005), 1, 65535, "listen_port")
    if error:
        return jsonify({"status": "error", "message": error}), 400

    with _state_lock:
        server = OSCInputServer(servo_ctrller, listen_ip=listen_ip, listen_port=listen_port)
        try:
            server.start()
        except Exception as e:
            logging.error(f"Failed to start OSC server: {e}")
            return jsonify({"status": "error", "message": str(e)}), 503
        _input_server_instance = server
        active_input_server = "osc"

    return jsonify({"status": "success", "active_input_server": "osc",
                    "listen_ip": listen_ip, "listen_port": listen_port})


@app.route('/server/stop', methods=['POST'])
def stop_input_server():
    global active_input_server, _input_server_instance

    if active_input_server is None:
        return jsonify({"status": "error", "message": "No input server is currently running."}), 400

    with _state_lock:
        stopped_type = active_input_server
        try:
            _input_server_instance.stop()
        except Exception as e:
            logging.error(f"Error stopping {stopped_type} server: {e}")
        _input_server_instance = None
        active_input_server = None

    return jsonify({"status": "success", "stopped": stopped_type})


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

    # Primary mechanism: existing clear_alarm_12(). NOTE -- this switches the
    # drive's DI control source to communication mode (PD16) and writes the
    # virtual EMG DI bit to its "released" state. If a physical E-Stop
    # circuit is still engaged, this can make the drive treat EMG as
    # released without the physical condition actually being resolved.
    # See README "Alarm 12 clear -- safety precondition" before using this.
    servo_ctrller.write_PD_16_Enable_DI_Control()
    servo_ctrller.clear_alarm_12()
    time.sleep(0.1)
    after_code = servo_ctrller.read_current_alarm_code()
    mechanism_used = "clear_alarm_12"

    # "No alarm" is 0xFF on this driver, not 0 (see servo_control.NO_ALARM_CODES).
    if is_alarm_active(after_code):
        # Fallback: official 0x0130 "Alarm clearance" register (write
        # 0x1EA5). Does not touch DI control source / virtual EMG state.
        servo_ctrller.clear_alarm_via_register()
        time.sleep(0.1)
        after_code = servo_ctrller.read_current_alarm_code()
        mechanism_used = "clear_alarm_12+0x0130_fallback"

    success = not is_alarm_active(after_code)
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


# enablePosMode used to hard-code these; they stay the defaults so a client
# that sends nothing behaves as before. 0x07800000 pulses, 10 rpm / 100 rpm.
DEFAULT_POS_MODE_PULSES = 0x07800000
DEFAULT_POS_MODE_SPEED_RPM = 10
DEFAULT_JOG_SPEED_RPM = 100


def _refused(action, message, status):
    return jsonify({"status": "error", "action": action, "message": message}), status


@app.route('/action', methods=['POST'])
@json_response
@hardware_serialized
def handle_action():
    data = request.get_json(silent=True) or {}
    action = data.get('action')

    print(f"Received action: {action}")

    # Enable the Digital I/O Writable
    servo_ctrller.write_PD_16_Enable_DI_Control()

    # Perform the raspi action here based on action type
    if action in ("start", "stop"):
        print(action)
    elif action == "servoOn":
        # SET_PARAM_2 command
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
        # Command pulses (0x0905/0x0906): 0~(2^31-1); speed (0x0903): 0~3000 rpm.
        pulses, error = validate_int_range(
            data.get('pulses', DEFAULT_POS_MODE_PULSES), 0, 2**31 - 1, 'pulses')
        if error:
            return _refused(action, error, 400)
        speed_rpm, error = validate_int_range(
            data.get('speed_rpm', DEFAULT_POS_MODE_SPEED_RPM), 0, 3000, 'speed_rpm')
        if error:
            return _refused(action, error, 400)
        servo_ctrller.Enable_Position_Mode(True)
        time.sleep(0.05)
        servo_ctrller.config_acc_dec_0x0902(0)
        time.sleep(0.05)
        servo_ctrller.config_speed_0x0903(speed_rpm)
        time.sleep(0.05)
        servo_ctrller.config_pulses_0x0905_low_byte(pulses & 0xFFFF)
        time.sleep(0.05)
        servo_ctrller.config_pulses_0x0906_high_byte((pulses >> 16) & 0xFFFF)
        time.sleep(0.05)
        # (was start_continuous_reading(0x0900, 0.1): two arguments to a
        # method that takes only the interval -- a TypeError.)
        servo_ctrller.start_continuous_reading(0.1)

    elif action == "disablePosMode":
        # Explicit exit: same calls as "motionCancel".
        servo_ctrller.stop_continuous_reading()
        servo_ctrller.Enable_Position_Mode(False)

    elif action == "posTestStart_CW":
        servo_ctrller.pos_step_motion_test(CW=True)

    elif action == "posTestStart_CCW":
        servo_ctrller.pos_step_motion_test(CW=False)

    elif action in ("setPoint_1", "setPoint_2"):
        # Records the drive's CURRENT angle as Set Point 1/2 -- does not move
        # the motor. (These buttons used to move to a hard-coded 90 / 180 deg.)
        try:
            servo_ctrller.record_set_point(int(action[-1]))
        except PositionUnavailableError as e:
            return _refused(action, str(e), 502)

    elif action in ("gotoSetPoint_1", "gotoSetPoint_2"):
        try:
            servo_ctrller.move_to_set_point(int(action[-1]))
        except PositionUnavailableError as e:
            return _refused(action, str(e), 502)
        except ValueError as e:
            return _refused(action, str(e), 400)

    elif action == "setHome":
        # The current position becomes 0 deg and is saved as the home
        # reference. No motor motion, but it must not run mid-move.
        if servo_ctrller.reading_active:
            return _refused(
                action,
                "Stop motion (MOTION CANCEL / wait for the move to finish) before setting home.",
                409)
        servo_ctrller.set_home_position()
        if not servo_ctrller.home_set_since_start:
            return _refused(
                action, "Home was NOT set: the position could not be read from the drive.", 502)

    elif action == "Home":
        try:
            servo_ctrller.post_step_motion_by(0)
        except PositionUnavailableError as e:
            # Refused without moving: the real position could not be read.
            return _refused(action, str(e), 502)

    elif action == "enableSpeedCtrlMode":
        speed_rpm, error = validate_int_range(
            data.get('speed_rpm', DEFAULT_JOG_SPEED_RPM), 0, 3000, 'speed_rpm')
        if error:
            return _refused(action, error, 400)
        servo_ctrller.enable_speed_ctrl(speed_rpm)

    elif action == "motionStart_CW":
        servo_ctrller.speed_ctrl_action(1)

    elif action == "motionStart_CCW":
        servo_ctrller.speed_ctrl_action(2)

    elif action == "motionPause":

        print("motion pause")
        servo_ctrller.speed_ctrl_action(0)

    elif action == "motionCancel":

        print("motion cancel")
        servo_ctrller.stop_continuous_reading()
        servo_ctrller.Enable_Position_Mode(False)
    else:
        return _refused(action, f"Action '{action}' not recognized.", 400)

    rs485_send, rs485_read = _current_rs485_traffic()
    return jsonify({
        "status": "success",
        "action": action,
        "RS485_send": rs485_send,
        "RS485_read": rs485_read,
        "message": f"Action {action} completed successfully."
    })


if __name__ == "__main__":
    web_host = os.getenv('SERVO_WEB_HOST', '0.0.0.0')
    web_port = int(os.getenv('SERVO_WEB_PORT', '5000'))
    # Flask's debug mode exposes an interactive debugger (arbitrary code
    # execution) to anyone who can reach the port -- opt-in only.
    web_debug = os.getenv('SERVO_WEB_DEBUG') == '1'
    if not WEB_PASSWORD and web_host not in ('127.0.0.1', 'localhost', '::1'):
        logging.warning(
            f"Web UI is reachable from the network ({web_host}:{web_port}) WITHOUT a password: "
            "anyone on the LAN can move the motor. Set SERVO_WEB_PASSWORD (and optionally "
            "SERVO_WEB_HOST=127.0.0.1) to restrict it."
        )
    if web_debug:
        logging.warning("SERVO_WEB_DEBUG=1: Flask debugger enabled -- never do this on a shared network.")
    if gpio_utils is not None:
        gpio_utils.initialize_gpio()
    try:
        # use_reloader=False: the reloader re-executes this module in a
        # second process, which would try to open the serial port (and
        # initialize GPIO) twice.
        app.run(host=web_host, port=web_port, debug=web_debug, use_reloader=False)
    finally:
        if active_input_server is not None and _input_server_instance is not None:
            _input_server_instance.stop()
        if gpio_utils is not None:
            gpio_utils.cleanup_gpio()
