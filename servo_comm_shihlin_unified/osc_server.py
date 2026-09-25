"""
OSC input server for continuous/discrete motion control.

Ported from servo_comm_shihlin_50W/osc_2.py's handlers, but as a class that
app.py can start/stop as a background thread against its own existing
ServoController instance -- not a standalone script that opens its own
serial connection. This is what avoids the "OSC server and app.py both hold
the serial port" problem noted in docs/servo_comm_shihlin_merge_design.md's
web-UI plan: there is only ever one ServoController per process, and
whichever input source (web UI buttons, OSC, Art-Net) is active just calls
methods on it.

Differences from the ported original, both deliberate:
- Host/port and the optional feedback (TouchDesigner-style) IP/port are
  constructor arguments, not hardcoded constants -- the original had
  TOUCHDESIGNER_IP/PORT baked in for one specific studio network, which
  doesn't belong in code meant to be reused across installations.
- ctrl_continuous_motion_handler() drops an extra, unconditional
  `speed_ctrl_action("CW")` call that existed right after the original's
  if/elif (a string where an int action_value is expected, firing
  regardless of the actual requested direction) -- flagged as likely
  dead/buggy code during an earlier review; not carried forward.
- set_initial_abs_position_handler() is not ported: it called
  ServoController.set_initial_abs_position(), which doesn't exist on
  either servo_comm_shihlin or servo_comm_shihlin_50W's ServoController --
  that OSC address never worked in the original either.
"""
import logging
import threading
import time

from pythonosc.dispatcher import Dispatcher
from pythonosc import osc_server as pythonosc_server
from pythonosc import udp_client

logger = logging.getLogger(__name__)

# How often a repeated /servo value is re-checked against the drive's real
# servo state (see OSCInputServer._check_duplicated). A control surface that
# resends /servo every frame must not cost a serial read per frame.
SERVO_STATE_RECHECK_S = 1.0


class OSCInputServer:
    def __init__(self, servo_ctrller, listen_ip="0.0.0.0", listen_port=5005,
                 feedback_ip=None, feedback_port=None):
        self.servo_ctrller = servo_ctrller
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self._osc_client = None
        if feedback_ip and feedback_port:
            self._osc_client = udp_client.SimpleUDPClient(feedback_ip, feedback_port)
        self._previous_data = None
        self._last_servo_verify = 0.0
        self._server = None
        self._thread = None

    @property
    def is_running(self) -> bool:
        return self._server is not None

    def _send_feedback(self, address, *args):
        if self._osc_client is None:
            return
        try:
            self._osc_client.send_message(address, args)
        except Exception as e:
            logger.error(f"Error sending OSC feedback to {address}: {e}")

    def _check_duplicated(self, data):
        """Only _servo_handler calls this. Dedup exists so a control surface
        that resends /servo 1.0 on every frame doesn't call servo_on()
        repeatedly.

        A repeated value is NOT necessarily redundant: the drive's servo
        state can change without this handler being involved (/cancel_loop
        leaves JOG mode, which drops Servo ON; the web UI; an alarm). So a
        repeat is re-checked against read_servo_state() -- at most once per
        SERVO_STATE_RECHECK_S -- and acted on if the drive disagrees. An
        unreadable state (None) counts as "can't tell" and stays suppressed."""
        now = time.monotonic()
        if data != self._previous_data:
            self._previous_data = data
            self._last_servo_verify = now
            return False
        if data not in (1.0, 0.0) or now - self._last_servo_verify < SERVO_STATE_RECHECK_S:
            return True
        self._last_servo_verify = now
        actual = self.servo_ctrller.read_servo_state()
        if actual is None or actual == (data == 1.0):
            return True
        logger.info(f"/servo {data}: repeated value but the drive is {'ON' if actual else 'OFF'}; applying it.")
        return False

    def _servo_handler(self, unused_addr, args, data):
        try:
            if not self._check_duplicated(data):
                if data == 1.0:
                    self.servo_ctrller.servo_on()
                    self._send_feedback("/servo_on", "on")
                elif data == 0.0:
                    self.servo_ctrller.servo_off()
                    self._send_feedback("/servo_off", "off")
        except Exception as e:
            logger.error(f"Error in servo_handler: {e}")

    def _clear_handler(self, unused_addr, *args):
        try:
            self.servo_ctrller.clear_alarm_12()
            self._send_feedback("/clear", "cleared")
        except Exception as e:
            logger.error(f"Error in clear_handler: {e}")

    def _set_continuous_motion_handler(self, unused_addr, args, speed_rpm, acc_time, enable=True):
        try:
            speed_rpm = int(speed_rpm)
            acc_time = int(acc_time)
            self.servo_ctrller.enable_speed_ctrl(speed_rpm, acc_time, enable)
            self._send_feedback("/continuous_mode_start", speed_rpm, acc_time)
        except Exception as e:
            logger.error(f"Error in set_continuous_motion_handler: {e}")

    def _ctrl_continuous_motion_handler(self, unused_addr, args, action, cw_ccw):
        try:
            if action == "stop":
                self.servo_ctrller.speed_ctrl_action(0)
                self._send_feedback("/continuous_mode_stop", "stop")
            elif action == "start":
                # Per docs/en_manual.txt:10380-10382 (JOG_OPERATION, 0x0904):
                # 1 = forward rotation (CCW), 2 = reverse rotation (CW).
                if cw_ccw == "CW":
                    self.servo_ctrller.speed_ctrl_action(2)
                elif cw_ccw == "CCW":
                    self.servo_ctrller.speed_ctrl_action(1)
                self._send_feedback("/continuous_mode_start", cw_ccw)
        except Exception as e:
            logger.error(f"Error in ctrl_continuous_motion_handler: {e}")

    def _jog_speed_adjust_handler(self, unused_addr, args, delta_rpm):
        """Nudges the running JOG speed by delta_rpm (e.g. +1/-1), the same
        API the web UI's Up/Down arrow keys use -- see
        ServoController.change_jog_speed_by()'s docstring. Distinct from
        /set_continous_motion, which sets an absolute starting speed before
        motion begins; this requires JOG mode to already be armed."""
        try:
            delta_rpm = int(delta_rpm)
            new_speed = self.servo_ctrller.change_jog_speed_by(delta_rpm)
            self._send_feedback("/jog_speed_adjust", new_speed)
        except Exception as e:
            logger.error(f"Error in jog_speed_adjust_handler: {e}")

    def _set_point_handler(self, unused_addr, args, angle, acc_time, rpm):
        try:
            angle = float(angle)
            acc_time = int(acc_time)
            rpm = int(rpm)
            self.servo_ctrller.post_step_motion_by(angle, acc_time, rpm)
            self._send_feedback("/set_point", angle, acc_time, rpm)
        except Exception as e:
            logger.error(f"Error in set_point_handler: {e}")

    def _reset_initial_abs_position_handler(self, unused_addr, *args):
        try:
            self.servo_ctrller.write_PA29_Initial_Abs_Pos()
            self._send_feedback("/reset_initial_abs_position", "reset")
        except Exception as e:
            logger.error(f"Error in reset_initial_abs_position_handler: {e}")

    def _back_home_handler(self, unused_addr, *args):
        try:
            self.servo_ctrller.initial_abs_home()
            self._send_feedback("/back_home", "back_home")
        except Exception as e:
            logger.error(f"Error in back_home_handler: {e}")

    def _set_home_position_handler(self, unused_addr, *args):
        try:
            self.servo_ctrller.set_home_position()
            self._send_feedback("/set_home_position", "set_home_position")
        except Exception as e:
            logger.error(f"Error in set_home_position_handler: {e}")

    def _cancel_loop_handler(self, unused_addr, *args):
        try:
            self.servo_ctrller.cancel_continuous_reading()
            # Leaving JOG mode drops Servo ON on the drive, so the next /servo
            # value must never be treated as a repeat of the last one.
            self._previous_data = None
            self._send_feedback("/cancel_loop", self.servo_ctrller.current_angle)
        except Exception as e:
            logger.error(f"Error in cancel_loop_handler: {e}")

    def _on_motion_completed(self):
        self._send_feedback("/motion_complete", "complete")

    def _on_moving(self, diff_angle):
        self._send_feedback("/moving", diff_angle)

    def _build_dispatcher(self) -> Dispatcher:
        dispatcher = Dispatcher()
        dispatcher.map("/servo", self._servo_handler, "data")
        dispatcher.map("/clear", self._clear_handler, "clear")
        dispatcher.map("/set_point", self._set_point_handler, "angle", "acc_time", "rpm")
        dispatcher.map("/back_home", self._back_home_handler)
        dispatcher.map("/set_home", self._set_home_position_handler)
        dispatcher.map("/reset_initial_abs_position", self._reset_initial_abs_position_handler)
        dispatcher.map("/set_continous_motion", self._set_continuous_motion_handler,
                        "speed_rpm", "acc_time", "enable")
        dispatcher.map("/ctrl_continuous_motion", self._ctrl_continuous_motion_handler,
                        "action", "CW_CCW")
        dispatcher.map("/jog_speed_adjust", self._jog_speed_adjust_handler, "delta_rpm")
        dispatcher.map("/cancel_loop", self._cancel_loop_handler)
        return dispatcher

    def start(self) -> None:
        if self._server is not None:
            raise RuntimeError("OSC server is already running.")

        self.servo_ctrller.register_event_listener("on_motion_completed", self._on_motion_completed)
        self.servo_ctrller.register_event_listener("on_moving", self._on_moving)

        dispatcher = self._build_dispatcher()
        self._server = pythonosc_server.ThreadingOSCUDPServer(
            (self.listen_ip, self.listen_port), dispatcher
        )
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        logger.info(f"OSC server listening on {self._server.server_address}")

    def stop(self) -> None:
        if self._server is None:
            return
        self.servo_ctrller.unregister_event_listener("on_motion_completed", self._on_motion_completed)
        self.servo_ctrller.unregister_event_listener("on_moving", self._on_moving)
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2)
        self._server = None
        self._thread = None
        logger.info("OSC server stopped.")
