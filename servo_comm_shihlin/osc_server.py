"""
OSC input server for servo_comm_shihlin, as a class the web UI can start and
stop (POST /server/start, /server/stop) against the app's own
ServoController -- instead of running osc.py as a second process that opens its
own serial connection and competes with app.py for the port.

The handlers and addresses are osc.py's, unchanged in behaviour:
  /servo <1.0|0.0>                  servo on / off
  /clear <1.0>                      clear alarm 12
  /set_point, /set_point_2 <angle> <acc_time> <rpm>
                                    move to an absolute angle
  /back_home <1.0>                  back to the saved home
  /pr_step_path <path>              PR mode: run PATH# (write_PF82)
  /set_home <1.0>                   save the current position as home
As in osc.py, a message whose value equals the previous one for the same
address is ignored (a control surface that resends the same value every frame
must not fire again). One deliberate difference: osc.py kept a SINGLE
"previous value" slot for every address, so e.g. /back_home 1.0 followed by
/set_home 1.0 silently dropped the second message; here each address has its
own slot.

Feedback (status output back to a control surface such as TouchDesigner) is
optional and separate from the command-input socket above -- pass
feedback_ip/feedback_port (e.g. from the /server/start request payload, not
hardcoded) to enable it; leave them None (the default) to run command-only,
exactly like osc.py. When enabled it sends:
  /servo_on, /servo_off <"on"|"off">   echo of a handled /servo command
  /clear <"cleared">                   echo of a handled /clear command
  /back_home <"back_home">             echo of a handled /back_home command
  /set_home_position <"set_home_position">
                                        echo of a handled /set_home command
  /pr_step_path <path>                 echo of a handled /pr_step_path command
  /motion_complete <"complete">        ServoController's on_motion_completed
  /moving <diff_angle>                 ServoController's on_moving
  /alarm <code>                        ServoController's on_alarm -- fires
                                        only off existing on-demand alarm
                                        reads (see read_current_alarm_code()),
                                        deliberately not a new polling loop:
                                        this board (Pi 3 B) has no CPU
                                        headroom to spare for one (see
                                        CLAUDE.md §2).
  /error <message>                     a handler caught an exception
There's no separate /status/angle or /status/position address: /moving
already carries the current angle for any handler that's actively moving,
and a static duplicate would just be more traffic for the same number.
"""
import logging
import threading

from pythonosc.dispatcher import Dispatcher
from pythonosc import osc_server as pythonosc_server
from pythonosc import udp_client

logger = logging.getLogger(__name__)


class OSCInputServer:
    def __init__(self, servo_ctrller, listen_ip="0.0.0.0", listen_port=5005,
                 feedback_ip=None, feedback_port=None):
        self.servo_ctrller = servo_ctrller
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.feedback_ip = feedback_ip
        self.feedback_port = feedback_port
        self._previous = {}
        self._server = None
        self._thread = None
        self._feedback_client = None
        self._feedback_wired = False

    @property
    def is_running(self) -> bool:
        return self._server is not None

    @property
    def feedback_enabled(self) -> bool:
        return self.feedback_ip is not None and self.feedback_port is not None

    def _check_duplicated(self, address, data) -> bool:
        if data != self._previous.get(address):
            self._previous[address] = data
            return False
        return True

    def _send_feedback(self, address, *args) -> None:
        """Best-effort status output; never raises into the caller -- a
        dropped feedback packet must not break the command path, nor (for
        on_moving) the background reading thread that calls it."""
        if self._feedback_client is None:
            return
        try:
            self._feedback_client.send_message(address, list(args))
        except Exception as e:
            logger.error(f"Error sending feedback {address}: {e}")

    def _send_error_feedback(self, context, exc) -> None:
        self._send_feedback("/error", f"{context}: {exc}")

    def _on_motion_completed_feedback(self) -> None:
        self._send_feedback("/motion_complete", "complete")

    def _on_moving_feedback(self, diff_angle) -> None:
        self._send_feedback("/moving", diff_angle)

    def _on_alarm_feedback(self, code) -> None:
        self._send_feedback("/alarm", code)

    def _servo_handler(self, unused_addr, args, data):
        try:
            if not self._check_duplicated("/servo", data):
                logger.info(f"Received: {data}")
                if data == 1.0:
                    self.servo_ctrller.servo_on()
                    self._send_feedback("/servo_on", "on")
                    logger.info("Servo turned on.")
                elif data == 0.0:
                    self.servo_ctrller.servo_off()
                    self._send_feedback("/servo_off", "off")
                    logger.info("Servo turned off.")
        except Exception as e:
            logger.error(f"Error in servo_handler: {e}")
            self._send_error_feedback("servo_handler", e)

    def _clear_handler(self, unused_addr, args, clear):
        try:
            if not self._check_duplicated("/clear", clear):
                if clear == 1.0:
                    self.servo_ctrller.clear_alarm_12()
                    self._send_feedback("/clear", "cleared")
                    logger.info("Cleared alarm 12.")
        except Exception as e:
            logger.error(f"Error in clear_handler: {e}")
            self._send_error_feedback("clear_handler", e)

    def _move_to_angle(self, address, angle, acc_time, rpm, name):
        try:
            if not self._check_duplicated(address, angle):
                logger.info("Sending command.")
                # Reads the real position first and refuses (raising) if it
                # cannot, so a stale angle never produces a wrong move.
                self.servo_ctrller.post_step_motion_by(angle, acc_time, rpm)
                logger.info(f"Set point to {angle} degrees. Acc time: {acc_time} ms. RPM: {rpm}")
        except Exception as e:
            logger.error(f"Error in {name}: {e}")
            self._send_error_feedback(name, e)

    def _set_point_handler(self, unused_addr, args, angle, acc_time, rpm):
        self._move_to_angle("/set_point", angle, acc_time, rpm, "set_point_handler")

    def _set_point_handler_2(self, unused_addr, args, angle, acc_time, rpm):
        self._move_to_angle("/set_point_2", angle, acc_time, rpm, "set_point_handler_2")

    def _back_home_handler(self, unused_addr, args, state):
        try:
            if not self._check_duplicated("/back_home", state):
                if state == 1.0:
                    self.servo_ctrller.initial_abs_home()
                    self._send_feedback("/back_home", "back_home")
                    logger.info("Back to home position.")
        except Exception as e:
            logger.error(f"Error in back_home_handler: {e}")
            self._send_error_feedback("back_home_handler", e)

    def _pr_mode_ctrl_handler(self, unused_addr, args, path_numb=0):
        try:
            if not self._check_duplicated("/pr_step_path", path_numb):
                logger.info("Sending Command. ")
                self.servo_ctrller.write_PF82(path_numb)
                self._send_feedback("/pr_step_path", path_numb)
                logger.info(f"Use Pr Mode to goto configed postion number: {path_numb}. ")
        except Exception as e:
            logger.error(f"Error in pr_mode_ctrl_handler: {e}")
            self._send_error_feedback("pr_mode_ctrl_handler", e)

    def _set_home_position_handler(self, unused_addr, args, state):
        try:
            if not self._check_duplicated("/set_home", state):
                if state == 1.0:
                    self.servo_ctrller.set_home_position()
                    self._send_feedback("/set_home_position", "set_home_position")
                    logger.info("Home position set.")
        except Exception as e:
            logger.error(f"Error in set_home_position_handler: {e}")
            self._send_error_feedback("set_home_position_handler", e)

    def _build_dispatcher(self) -> Dispatcher:
        dispatcher = Dispatcher()
        dispatcher.map("/servo", self._servo_handler, "data")
        dispatcher.map("/clear", self._clear_handler, "clear")
        dispatcher.map("/set_point", self._set_point_handler, "angle", "acc_time", "rpm")
        dispatcher.map("/set_point_2", self._set_point_handler_2, "angle", "acc_time", "rpm")
        dispatcher.map("/back_home", self._back_home_handler, "state")
        dispatcher.map("/pr_step_path", self._pr_mode_ctrl_handler, "path_number")
        dispatcher.map("/set_home", self._set_home_position_handler, "state")
        return dispatcher

    def start(self) -> None:
        if self._server is not None:
            raise RuntimeError("OSC server is already running.")
        if self.feedback_enabled:
            self._feedback_client = udp_client.SimpleUDPClient(self.feedback_ip, self.feedback_port)
            # ServoController outlives this server across start/stop cycles,
            # so listeners must be registered/unregistered here rather than
            # in __init__, or a restart would pile up duplicate callbacks.
            self.servo_ctrller.register_event_listener("on_motion_completed", self._on_motion_completed_feedback)
            self.servo_ctrller.register_event_listener("on_moving", self._on_moving_feedback)
            self.servo_ctrller.register_event_listener("on_alarm", self._on_alarm_feedback)
            self._feedback_wired = True
            logger.info(f"Feedback enabled -> {self.feedback_ip}:{self.feedback_port}")
        self._server = pythonosc_server.ThreadingOSCUDPServer(
            (self.listen_ip, self.listen_port), self._build_dispatcher()
        )
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        logger.info(f"OSC server listening on {self._server.server_address}")

    def stop(self) -> None:
        if self._server is None:
            return
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2)
        self._server = None
        self._thread = None
        if self._feedback_wired:
            self.servo_ctrller.unregister_event_listener("on_motion_completed", self._on_motion_completed_feedback)
            self.servo_ctrller.unregister_event_listener("on_moving", self._on_moving_feedback)
            self.servo_ctrller.unregister_event_listener("on_alarm", self._on_alarm_feedback)
            self._feedback_wired = False
        self._feedback_client = None
        logger.info("OSC server stopped.")
