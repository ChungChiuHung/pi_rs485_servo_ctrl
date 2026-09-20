import hmac
import ipaddress
import os
import time
import logging
import threading
import traceback
from datetime import datetime, timezone
from functools import wraps

from flask import Flask, render_template, request, jsonify, Response, redirect

from serial_port_manager import SerialPortManager
from servo_control import ServoController, PositionUnavailableError, is_alarm_active, alarm_name
from motor_profile import load_profiles, resolve_profile
from hardware_lock import hardware_serialized, run_when_idle
from input_validation import validate_int_range
from activity_log import ActivityLog, ActivityLogHandler
from osc_server import OSCInputServer
from artnet_server import ArtNetInputServer, DEFAULT_UNIVERSE, DEFAULT_SIGNAL_TIMEOUT_S

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

# The web UI can move a motor, set home and rewrite drive parameters, and has
# no login of its own. Setting SERVO_WEB_PASSWORD turns on HTTP Basic auth for
# every route (user: SERVO_WEB_USER, default "servo"). Off by default so local
# development keeps working; the startup log warns when the UI is reachable
# from the network without it.
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
# Same requirement for PA28 (encoder mode) writes: hardware-affecting, must be logged.
encoder_mode_logger = logging.getLogger("encoder_mode")

# Web UI activity feed for a user who isn't watching this process's own
# console -- captures every existing logging call across the codebase (see
# activity_log.py). Attached to the root logger before anything else runs
# so startup messages (profile connection, GPIO availability) are captured
# too.
activity_log = ActivityLog()
_activity_log_handler = ActivityLogHandler(activity_log)
_activity_log_handler.setFormatter(logging.Formatter('%(message)s'))
logging.getLogger().addHandler(_activity_log_handler)

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
# None | "osc" | "artnet" -- guards /profile's mutual-exclusion check and
# reports which input source (if any) is live.
active_input_server = None
# The actual running OSCInputServer/ArtNetInputServer object, or None.
_input_server_instance = None


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
    # First communication after connect: make sure PA23 inhibits EEPROM
    # writes BEFORE anything below (or any later action) writes PD16/PD25 --
    # on older firmware PA23=1 reverts to 0 at every power-off, so this is
    # needed after every boot, not just once.
    try:
        servo_ctrller.ensure_eeprom_write_protection()
    except Exception as e:
        logging.warning(f"Could not verify EEPROM write protection at connect time ({e}).")
    # Read-only: sync the controller's absolute/incremental mode with the
    # drive's actual PA28 (a fresh process after a power cycle is the normal
    # way a mode change takes effect). Unreadable -> stays incremental.
    try:
        servo_ctrller.refresh_encoder_mode()
    except Exception as e:
        logging.warning(f"Could not read PA28 at connect time ({e}); assuming incremental mode.")
    # Read-only: angle and positioning math assume a 1:1 electronic gear ratio
    # (PA06 = PA07); warn loudly if the drive says otherwise.
    try:
        servo_ctrller.check_electronic_gear_ratio()
    except Exception as e:
        logging.warning(f"Could not check the electronic gear ratio at connect time ({e}).")
    logging.info(
        f"Active profile: {profile_name} -- connected "
        f"{serial_manager.get_connected_port()} @ {profile['baud_rate']} baud"
    )


with _state_lock:
    _connect_profile(current_profile_name)

def _current_rs485_traffic():
    """The most recent raw Modbus transaction's bytes, for the web UI's
    "RS-485 Send/Receive" boxes. These used to be a hardcoded placeholder
    ("00 00 FF FF" / "FF FF 00 00") never updated from real traffic --
    now reads ModbusRTUClient.last_sent/last_received directly, which
    every send()/receive() call updates (see modbus_rtu_client.py)."""
    client = servo_ctrller.modbus_client
    return client.format_hex(client.last_sent), client.format_hex(client.last_received)


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
    # home.html was an unused legacy "Walking Lamp Control" test page (a
    # START/STOP pair wired to no-op actions) -- removed 2026-09-18. index.html
    # is the actual control panel this project uses.
    return redirect('/index')


@app.route('/index')
def index():
    rs485_send, rs485_read = _current_rs485_traffic()
    return render_template(
        'index.html', title='Servo Control Panel',
        RS485_read=rs485_read, RS485_send=rs485_send
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
        # None means "communication failure, unknown" -- never assume it
        # means "off". See ServoController.read_servo_state()'s docstring.
        "servo_on": servo_ctrller.read_servo_state(),
        # Diagnostic: what CTRL_MODE_SEL (0x0901) actually reads back as
        # right now -- 0=idle, 3=JOG test, 4=Positioning test. Confirms
        # whether the drive is really latched into a mode a button just
        # tried to enter (see read_test_mode_0x0901()'s docstring).
        "ctrl_mode_sel": servo_ctrller.read_test_mode_0x0901(),
        "current_angle": servo_ctrller.current_angle,
        "current_encoder": servo_ctrller.current_encoder,
        # None means "never recorded for this profile" -- see
        # ServoController.record_set_point()'s docstring.
        "set_point_1": servo_ctrller.set_point_1,
        "set_point_2": servo_ctrller.set_point_2,
        "alarm_code": alarm_code,
        # Raw alarm_code alone is misleading: this driver reports 0xFF
        # (255), not 0, for "no alarm" (see servo_control.NO_ALARM_CODES).
        # The UI should key off this, not "alarm_code != 0".
        "alarm_active": is_alarm_active(alarm_code),
        # Known absolute-system alarm explanation (None for other codes).
        "alarm_name": alarm_name(alarm_code),
        # Cached, no serial traffic. True only once PA28 == 1 has been read
        # from the drive (see /encoder_mode).
        "absolute_mode": servo_ctrller.absolute_mode,
        # Last known PA23 (EEPROM write inhibit): 0 = NOT protected, 1/2 = protected.
        "eeprom_protection": servo_ctrller.eeprom_protection,
        # PA06/PA07 as [cmx, cdv] (None = unread). Angle/positioning math
        # assumes 1:1; electronic_gear_ok is False when the drive disagrees.
        "electronic_gear": list(servo_ctrller.electronic_gear) if servo_ctrller.electronic_gear else None,
        "electronic_gear_ok": servo_ctrller.electronic_gear_unity,
        # SET HOME has succeeded since this process started. The incremental
        # counter restarts at every drive power-on, so the UI asks the operator
        # to SET HOME after each start -- except in absolute mode with a saved
        # absolute home, whose position survives power-off.
        "home_set_since_start": servo_ctrller.home_set_since_start,
        "home_reminder_needed": (
            not servo_ctrller.home_set_since_start
            and not (servo_ctrller.absolute_mode
                     and servo_ctrller.abs_home_pos_absolute is not None)),
        "absolute_home_set": servo_ctrller.abs_home_pos_absolute is not None,
    })


# Seconds between PA23 (EEPROM write-inhibit) re-checks. OSC/Art-Net handlers
# call ServoController directly, bypassing /action, so a background guard
# (started in __main__) keeps PA23 applied for them too -- e.g. after the
# drive is power-cycled while the app keeps running.
EEPROM_GUARD_INTERVAL_S = 30


def _eeprom_guard_loop():
    while True:
        time.sleep(EEPROM_GUARD_INTERVAL_S)
        try:
            run_when_idle(lambda: servo_ctrller.ensure_eeprom_write_protection())
        except Exception as e:
            logging.warning(f"EEPROM protection guard failed: {e}")


# Set by a successful PA28 write; cleared once the user says they power-cycled
# the drive and the mode is re-read. Until then the running process keeps
# using the mode the drive is actually in.
_encoder_mode_pending_power_cycle = None


@app.route('/encoder_mode', methods=['GET'])
@hardware_serialized
def get_encoder_mode():
    """Read-only diagnostic: PA28 as configured in the drive, the mode this
    process is actually using, and (when configured absolute) PA31's
    absolute-position health flags. Never writes to the drive."""
    from servo_p_register import PA
    pa28 = servo_ctrller.read_PA28_Encoder_Mode()
    apst = servo_ctrller.read_PA31_Abs_Position_Status() if pa28 == 1 else None
    return jsonify({
        "pa28": pa28,
        "active_absolute_mode": servo_ctrller.absolute_mode,
        "pending_power_cycle": _encoder_mode_pending_power_cycle,
        "absolute_status": None if apst is None else PA.decode_APST(apst),
        "absolute_home_set": servo_ctrller.abs_home_pos_absolute is not None,
    })


@app.route('/encoder_mode', methods=['POST'])
@hardware_serialized
def set_encoder_mode():
    """Writes PA28 (0 = incremental, 1 = absolute). HARDWARE-AFFECTING --
    see ServoController.write_PA28_Encoder_Mode()'s docstring for the
    prerequisites (absolute-encoder motor, backup battery) and the required
    power cycles. Requires {"absolute": bool, "confirm": true}. Takes effect
    only after a drive power cycle; the running process keeps its current
    mode until /encoder_mode/adopt is called afterwards."""
    global _encoder_mode_pending_power_cycle
    caller_ip = request.remote_addr
    payload = request.get_json(silent=True) or {}

    if payload.get('confirm') is not True:
        encoder_mode_logger.warning("Rejected PA28 write from %s: missing confirm=true.", caller_ip)
        return jsonify({"status": "error",
                        "message": 'Missing or false "confirm" field. POST {"absolute": bool, "confirm": true}.'}), 400
    if not isinstance(payload.get('absolute'), bool):
        return jsonify({"status": "error", "message": '"absolute" must be true or false.'}), 400
    if servo_ctrller.reading_active or active_input_server is not None:
        return jsonify({"status": "error",
                        "message": "Stop motion and any OSC/Art-Net server before changing the encoder mode."}), 409

    absolute = payload['absolute']
    encoder_mode_logger.warning("PA28 write requested by %s: absolute=%s", caller_ip, absolute)
    if not servo_ctrller.write_PA28_Encoder_Mode(absolute):
        return jsonify({"status": "error",
                        "message": "PA28 write was not confirmed by the drive. Nothing was changed."}), 502

    _encoder_mode_pending_power_cycle = absolute
    if absolute:
        next_steps = ("Power-cycle the drive. AL.2A is then expected: power-cycle again. "
                      "AL.2C is then expected: press SET HOME (or write PA29=1) to initialise. "
                      "Finally press 'I POWER-CYCLED - RE-READ MODE'.")
    else:
        next_steps = "Power-cycle the drive, then press 'I POWER-CYCLED - RE-READ MODE'."
    return jsonify({"status": "success", "pa28_written": 1 if absolute else 0, "message": next_steps})


@app.route('/encoder_mode/adopt', methods=['POST'])
@hardware_serialized
def adopt_encoder_mode():
    """Re-reads PA28 and makes this process use it. Call after a power cycle."""
    global _encoder_mode_pending_power_cycle
    mode = servo_ctrller.refresh_encoder_mode()
    if mode is None:
        return jsonify({"status": "error", "message": "Could not read PA28 (communication failure)."}), 503
    _encoder_mode_pending_power_cycle = None
    return jsonify({"status": "success", "active_absolute_mode": mode})


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


@app.route('/server/status', methods=['GET'])
def get_input_server_status():
    return jsonify({
        "active_input_server": active_input_server,
        "is_running": _input_server_instance.is_running if _input_server_instance else False,
    })


@app.route('/server/artnet_channels', methods=['GET'])
def get_artnet_channels():
    """Interpreted state of the most recently received Art-Net DMX frame,
    for the web UI's Channel Monitor -- lets a user confirm what a
    console/controller actually sent without an external DMX tool.
    Read-only; never touches the driver. `active: false` (with
    `channels: null`) whenever Art-Net isn't the running input server."""
    if active_input_server != "artnet" or _input_server_instance is None:
        return jsonify({"active": False, "channels": None, "stats": None})
    return jsonify({
        "active": True,
        "channels": _input_server_instance.get_channel_snapshot(),
        "stats": _input_server_instance.get_stats(),
    })


@app.route('/log', methods=['GET'])
def get_activity_log():
    """Incremental activity feed for a user not watching this process's
    own console -- pass `since` (an entry id from a previous call) to get
    only what's new. Never triggers any write/motion command."""
    since = request.args.get('since', default=0, type=int) or 0
    return jsonify({"entries": activity_log.get_since(since)})


def _parse_artnet_options(payload):
    """Validates the Art-Net start request. Returns (ArtNetInputServer kwargs,
    None) or (None, error message). The safe values are the defaults:
    universe 1, a 2s loss-of-signal stop, channels 10-12 ignored."""
    def number(key, default, lo, hi, kind=int):
        raw = payload.get(key, default)
        try:
            value = kind(raw)
        except (TypeError, ValueError):
            return None, f"{key} must be a number."
        if not (lo <= value <= hi):
            return None, f"{key} must be between {lo} and {hi}."
        return value, None

    listen_ip = payload.get("listen_ip") or "0.0.0.0"
    try:
        ipaddress.IPv4Address(listen_ip)
    except ValueError:
        return None, "listen_ip must be an IPv4 address (e.g. 192.168.1.50 or 0.0.0.0)."

    sources = payload.get("allowed_sources") or []
    if isinstance(sources, str):
        sources = [item.strip() for item in sources.split(",") if item.strip()]
    if not isinstance(sources, (list, tuple)):
        return None, "allowed_sources must be a list or comma-separated IPv4 addresses."
    for item in sources:
        try:
            ipaddress.IPv4Address(item)
        except ValueError:
            return None, f"allowed_sources contains an invalid IPv4 address: {item!r}."

    dangerous = payload.get("enable_dangerous_channels", False)
    if not isinstance(dangerous, bool):
        return None, "enable_dangerous_channels must be true or false."

    options = {"listen_ip": listen_ip, "allowed_sources": list(sources),
               "enable_dangerous_channels": dangerous}
    for key, default, lo, hi, kind in (
            ("listen_port", 6454, 1, 65535, int),
            ("universe", DEFAULT_UNIVERSE, 0, 32767, int),
            ("max_speed_rpm", 100, 1, 3000, int),
            ("acc_time", 5000, 0, 60000, int),
            ("position_mode_max_angle", 360, 1, 100000, float),
            ("signal_timeout_s", DEFAULT_SIGNAL_TIMEOUT_S, 0, 60, float)):
        value, error = number(key, default, lo, hi, kind)
        if error:
            return None, error
        options[key] = value
    return options, None


@app.route('/server/start', methods=['POST'])
def start_input_server():
    """Starts OSC or Art-Net as the live continuous-motion input source.
    Only one may run at a time -- both would otherwise be able to issue
    conflicting motion commands to the same ServoController concurrently.
    Does not itself send anything to the driver; it only registers event
    listeners and starts a UDP listener thread. Real hardware I/O only
    happens later, if and when a message actually arrives and a handler
    calls a ServoController method -- same as any /action button click.
    """
    global active_input_server, _input_server_instance

    if active_input_server is not None:
        return jsonify({
            "status": "error",
            "message": f"'{active_input_server}' server is already running. Stop it first.",
        }), 409

    payload = request.get_json(silent=True) or {}
    server_type = payload.get("type")

    if server_type == "osc":
        listen_ip = payload.get("listen_ip", "0.0.0.0")
        listen_port = int(payload.get("listen_port", 5005))
        feedback_ip = payload.get("feedback_ip")
        feedback_port = payload.get("feedback_port")
        feedback_port = int(feedback_port) if feedback_port else None

        with _state_lock:
            server = OSCInputServer(
                servo_ctrller, listen_ip=listen_ip, listen_port=listen_port,
                feedback_ip=feedback_ip, feedback_port=feedback_port,
            )
            try:
                server.start()
            except Exception as e:
                logging.error(f"Failed to start OSC server: {e}")
                return jsonify({"status": "error", "message": str(e)}), 503
            _input_server_instance = server
            active_input_server = "osc"

        return jsonify({
            "status": "success",
            "active_input_server": "osc",
            "listen_ip": listen_ip,
            "listen_port": listen_port,
        })

    elif server_type == "artnet":
        options, error = _parse_artnet_options(payload)
        if error:
            return jsonify({"status": "error", "message": error}), 400
        listen_ip = options["listen_ip"]
        listen_port = options["listen_port"]
        universe = options["universe"]

        with _state_lock:
            server = ArtNetInputServer(servo_ctrller, **options)
            try:
                server.start()
            except Exception as e:
                logging.error(f"Failed to start Art-Net server: {e}")
                return jsonify({"status": "error", "message": str(e)}), 503
            _input_server_instance = server
            active_input_server = "artnet"

        return jsonify({
            "status": "success",
            "active_input_server": "artnet",
            "listen_ip": listen_ip,
            "listen_port": listen_port,
            "universe": universe,
            "signal_timeout_s": options["signal_timeout_s"],
            "allowed_sources": sorted(options["allowed_sources"]),
            "enable_dangerous_channels": options["enable_dangerous_channels"],
        })

    else:
        return jsonify({
            "status": "error",
            "message": f"Unknown server type: {server_type!r}. Expected 'osc' or 'artnet'.",
        }), 400


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

    # Primary mechanism: clear_alarm_12(). NOTE -- this switches the drive's
    # DI control source to communication mode (PD16) and writes the virtual
    # EMG DI bit to its "released" state. If a physical E-Stop circuit is
    # still engaged, this can make the drive treat EMG as released without
    # the physical condition actually being resolved. See README "Alarm 12
    # clear -- safety precondition" before using this.
    servo_ctrller.ensure_eeprom_write_protection(max_age_s=EEPROM_GUARD_INTERVAL_S)
    servo_ctrller.write_PD_16_Enable_DI_Control()
    servo_ctrller.clear_alarm_12()
    time.sleep(0.1)
    after_code = servo_ctrller.read_current_alarm_code()
    mechanism_used = "clear_alarm_12"

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


@app.route('/action', methods=['POST'])
@json_response
@hardware_serialized
def handle_action():
    data = request.json
    action = data.get('action')

    print(f"Received action: {action}")

    # Populated only by getMsg -- see the final response below.
    state_values = None

    # PD16/PD25 are EEPROM-backed parameters and are written by nearly every
    # action; confirm PA23 protection first (throttled: at most one PA23 read
    # per 30s) so a drive power-cycled mid-session is re-protected.
    servo_ctrller.ensure_eeprom_write_protection(max_age_s=EEPROM_GUARD_INTERVAL_S)

    # Enable the Digital I/O Writable
    servo_ctrller.write_PD_16_Enable_DI_Control()

    if action == "servoOn":
        servo_ctrller.clear_alarm_12()
        time.sleep(0.1)
        servo_ctrller.servo_on()
        time.sleep(0.1)
    elif action == "servoOff":
        servo_ctrller.servo_off()
    elif action == "getMsg":
        state_values = servo_ctrller.Read_Pos_Related_Paremters()
    elif action == "enablePosMode":
        # Command pulses (0x0905/0x0906): manual's documented range is
        # 0~(2^31-1) -- see docs/en_manual.txt:10416-10422.
        pulses, error = validate_int_range(data.get('pulses', 1920), 0, 2**31 - 1, 'pulses')
        if error:
            return jsonify({"status": "error", "action": action, "message": error}), 400
        # Positioning speed command (0x0903): manual's documented range is
        # 0~3000 rpm -- see docs/en_manual.txt:10367-10373.
        speed_rpm, error = validate_int_range(data.get('speed_rpm', 10), 0, 3000, 'speed_rpm')
        if error:
            return jsonify({"status": "error", "action": action, "message": error}), 400
        # Positioning-test mode requires "no alarm occurrence or Servo ON
        # activated" (docs/en_manual.txt:10390) -- same precondition as JOG
        # mode; see _execute_positioning()'s comment in servo_control.py.
        servo_ctrller.clear_alarm_12()
        time.sleep(0.1)
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
        servo_ctrller.start_continuous_reading(0.1)
    elif action == "disablePosMode":
        # Explicit exit -- pairs with "enablePosMode" as a toggle in the
        # web UI. Note that position-test mode also exits on its own once
        # a triggered move (POS TEST START CW/CCW) settles (the software
        # auto-stop detects stillness and stops the keep-alive polling,
        # after which the drive's own ~1s communication-timeout drops it
        # out of test mode) -- this action is for exiting deliberately
        # before that happens, or for cleanliness afterward. Same calls as
        # "motionCancel" (which serves the JOG/speed-control section);
        # Enable_Position_Mode(False) is the documented generic "quit
        # test mode" write regardless of which mode was active.
        servo_ctrller.stop_continuous_reading()
        servo_ctrller.Enable_Position_Mode(False)
    elif action == "posTestStart_CW":
        servo_ctrller.pos_step_motion_test(CW=True)
    elif action == "posTestStart_CCW":
        servo_ctrller.pos_step_motion_test(CW=False)
    elif action == "setPoint_1":
        # Records the CURRENT tracked angle as Set Point 1 -- does not move
        # the motor. Persisted per-profile (servo_config_<profile>.json),
        # so it survives a restart.
        servo_ctrller.record_set_point(1)
    elif action == "setPoint_2":
        servo_ctrller.record_set_point(2)
    elif action == "gotoSetPoint_1":
        try:
            servo_ctrller.move_to_set_point(1)
        except ValueError as e:
            return jsonify({"status": "error", "action": action, "message": str(e)}), 400
    elif action == "gotoSetPoint_2":
        try:
            servo_ctrller.move_to_set_point(2)
        except ValueError as e:
            return jsonify({"status": "error", "action": action, "message": str(e)}), 400
    elif action == "setHome":
        # The current position becomes 0 deg and is persisted as the home
        # reference (all recorded Set Points are measured from it). No motor
        # motion, but it must not run mid-move.
        if servo_ctrller.reading_active:
            return jsonify({
                "status": "error", "action": action,
                "message": "Stop motion (MOTION CANCEL / wait for the move to finish) before setting home.",
            }), 409
        servo_ctrller.set_home_position()
        if not servo_ctrller.home_set_since_start:
            return jsonify({
                "status": "error", "action": action,
                "message": "Home was NOT set: the position could not be read from the drive.",
            }), 502
    elif action == "Home":
        try:
            servo_ctrller.post_step_motion_by(0)
        except PositionUnavailableError as e:
            return jsonify({"status": "error", "action": action, "message": str(e)}), 502
    elif action == "enableSpeedCtrlMode":
        # JOG speed command (0x0903): manual's documented range is 0~3000
        # rpm -- see docs/en_manual.txt:10367-10373.
        speed_rpm, error = validate_int_range(data.get('speed_rpm', 100), 0, 3000, 'speed_rpm')
        if error:
            return jsonify({"status": "error", "action": action, "message": error}), 400
        servo_ctrller.enable_speed_ctrl(speed_rpm)
    elif action == "motionStart_CW":
        # Per docs/en_manual.txt:10380-10382 (JOG_OPERATION, 0x0904):
        # 1 = forward rotation (CCW), 2 = reverse rotation (CW).
        # speed_ctrl_action() refuses (returns False) a direct reversal
        # while still running the other direction -- see its own comment.
        if not servo_ctrller.speed_ctrl_action(2):
            return jsonify({
                "status": "error", "action": action,
                "message": "Press MOTION PAUSE before switching direction.",
            }), 409
    elif action == "motionStart_CCW":
        if not servo_ctrller.speed_ctrl_action(1):
            return jsonify({
                "status": "error", "action": action,
                "message": "Press MOTION PAUSE before switching direction.",
            }), 409
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

    rs485_send, rs485_read = _current_rs485_traffic()
    return jsonify({
        "status": "success",
        "action": action,
        "RS485_send": rs485_send,
        "RS485_read": rs485_read,
        # Only non-null for getMsg -- decoded PA/PD parameter registers,
        # see Read_Pos_Related_Paremters()'s docstring.
        "state_values": state_values,
        "message": f"Action {action} completed successfully.",
    })


if __name__ == "__main__":
    threading.Thread(target=_eeprom_guard_loop, name="eeprom-guard", daemon=True).start()
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
    try:
        # use_reloader=False: the reloader re-executes this module in a
        # second process, which would try to open the serial port (and
        # initialize GPIO) twice.
        app.run(host=web_host, port=web_port, debug=web_debug, use_reloader=False)
    finally:
        if gpio_utils is not None:
            gpio_utils.cleanup_gpio()
