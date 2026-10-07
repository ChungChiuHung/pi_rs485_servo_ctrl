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
own slot. osc.py sends no feedback and has no continuous-motion addresses
(those exist only in servo_comm_shihlin_50W).
"""
import logging
import threading

from pythonosc.dispatcher import Dispatcher
from pythonosc import osc_server as pythonosc_server

logger = logging.getLogger(__name__)


class OSCInputServer:
    def __init__(self, servo_ctrller, listen_ip="0.0.0.0", listen_port=5005):
        self.servo_ctrller = servo_ctrller
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self._previous = {}
        self._server = None
        self._thread = None

    @property
    def is_running(self) -> bool:
        return self._server is not None

    def _check_duplicated(self, address, data) -> bool:
        if data != self._previous.get(address):
            self._previous[address] = data
            return False
        return True

    def _servo_handler(self, unused_addr, args, data):
        try:
            if not self._check_duplicated("/servo", data):
                logger.info(f"Received: {data}")
                if data == 1.0:
                    self.servo_ctrller.servo_on()
                    logger.info("Servo turned on.")
                elif data == 0.0:
                    self.servo_ctrller.servo_off()
                    logger.info("Servo turned off.")
        except Exception as e:
            logger.error(f"Error in servo_handler: {e}")

    def _clear_handler(self, unused_addr, args, clear):
        try:
            if not self._check_duplicated("/clear", clear):
                if clear == 1.0:
                    self.servo_ctrller.clear_alarm_12()
                    logger.info("Cleared alarm 12.")
        except Exception as e:
            logger.error(f"Error in clear_handler: {e}")

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

    def _set_point_handler(self, unused_addr, args, angle, acc_time, rpm):
        self._move_to_angle("/set_point", angle, acc_time, rpm, "set_point_handler")

    def _set_point_handler_2(self, unused_addr, args, angle, acc_time, rpm):
        self._move_to_angle("/set_point_2", angle, acc_time, rpm, "set_point_handler_2")

    def _back_home_handler(self, unused_addr, args, state):
        try:
            if not self._check_duplicated("/back_home", state):
                if state == 1.0:
                    self.servo_ctrller.initial_abs_home()
                    logger.info("Back to home position.")
        except Exception as e:
            logger.error(f"Error in back_home_handler: {e}")

    def _pr_mode_ctrl_handler(self, unused_addr, args, path_numb=0):
        try:
            if not self._check_duplicated("/pr_step_path", path_numb):
                logger.info("Sending Command. ")
                self.servo_ctrller.write_PF82(path_numb)
                logger.info(f"Use Pr Mode to goto configed postion number: {path_numb}. ")
        except Exception as e:
            logger.error(f"Error in pr_mode_ctrl_handler: {e}")

    def _set_home_position_handler(self, unused_addr, args, state):
        try:
            if not self._check_duplicated("/set_home", state):
                if state == 1.0:
                    self.servo_ctrller.set_home_position()
                    logger.info("Home position set.")
        except Exception as e:
            logger.error(f"Error in set_home_position_handler: {e}")

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
        logger.info("OSC server stopped.")
