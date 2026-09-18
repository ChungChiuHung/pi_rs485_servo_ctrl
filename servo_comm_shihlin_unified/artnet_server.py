"""
Art-Net (DMX-over-Ethernet) input server -- the second continuous-motion
input source, alongside OSC (osc_server.py). Listens for ArtDMX packets on
UDP port 6454 (the Art-Net standard) and maps 3 DMX channels onto the same
ServoController continuous-motion methods OSC's handlers use, so this is a
peer of osc_server.OSCInputServer with the exact same lifecycle shape
(start()/stop()/is_running) for app.py's /server/start+/server/stop to
drive identically regardless of protocol.

DMX channel mapping (provisional -- see
docs/servo_comm_shihlin_merge_design.md's web-UI plan §2; adjust once the
actual upstream Art-Net source (lighting console, TouchDesigner, etc.) and
its real channel layout are known -- this is a placeholder convention, not
a standard):
  Channel 1 (enable/speed): 0 = disable continuous motion; 1-255 linearly
      maps to speed_rpm (scaled by `max_speed_rpm`, minimum 1 rpm).
  Channel 2 (direction): 0 = stop; 1-127 = CCW; 128-255 = CW.
  Channel 3 (cancel): 0 = normal; a 0->nonzero transition triggers the same
      cancel behavior as OSC's /cancel_loop.
  Channel 4 (position-mode trigger): 0 = idle; a 0->nonzero transition
      triggers a single absolute-angle move (ServoController.post_step_motion_by(),
      the same call OSC's /set_point makes) using whatever channels 5-7
      currently hold. Optional -- a sender that only ever fills channels
      1-3 (continuous mode only) still works, this is just never triggered.
  Channel 5-6 (target angle, high byte/low byte): a 16-bit big-endian value
      0-65535, linearly mapped to 0..`position_mode_max_angle` degrees.
  Channel 7 (position move speed): 1-255 linearly maps to speed_rpm (scaled
      by `max_speed_rpm`, minimum 1 rpm -- same scaling as channel 1).
  Channel 8 (servo on/off): 0 = Servo off; 1-255 = Servo on (level-based
      like channel 1, not edge-triggered -- mirrors OSC's /servo).
  Channel 9 (clear alarm 12): 0 = normal; a 0->nonzero transition clears
      Alarm 12, mirroring OSC's /clear.
  Channel 10 (back home): 0 = idle; a 0->nonzero transition triggers
      ServoController.initial_abs_home() (a real move back to the saved
      home position), mirroring OSC's /back_home.
  Channel 11 (set home): 0 = idle; a 0->nonzero transition triggers
      ServoController.set_home_position() -- persists the CURRENT position
      as the new home reference (see its own docstring), mirroring OSC's
      /set_home.
  Channel 12 (reset initial absolute position): 0 = idle; a 0->nonzero
      transition triggers ServoController.write_PA29_Initial_Abs_Pos(),
      mirroring OSC's /reset_initial_abs_position.
  Channels 8-12 are optional, like 4-7 -- a sender filling only channels
  1-3 (or 1-7) still works, these are just never triggered.

All channels are edge-triggered against the previously-seen frame, not
re-sent on every DMX refresh (a real Art-Net source typically resends the
full frame 30-44 times/second even when nothing changed) -- otherwise e.g.
enable_speed_ctrl() or post_step_motion_by() would fire on every single
frame instead of once per real state change.

No external Art-Net library dependency: ArtDMX's binary header is small
and stable, so it's parsed by hand rather than adding a new
requirements.txt entry for it.
"""
import logging
import socket
import struct
import threading

logger = logging.getLogger(__name__)

ARTNET_PORT = 6454
ARTNET_ID = b"Art-Net\x00"
OP_OUTPUT_DMX = 0x5000


class ArtNetInputServer:
    def __init__(self, servo_ctrller, listen_ip="0.0.0.0", listen_port=ARTNET_PORT,
                 universe=0, max_speed_rpm=100, acc_time=5000, position_mode_max_angle=360):
        self.servo_ctrller = servo_ctrller
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.universe = universe
        self.max_speed_rpm = max_speed_rpm
        self.acc_time = acc_time
        self.position_mode_max_angle = position_mode_max_angle

        self._sock = None
        self._thread = None
        self._stop_event = threading.Event()

        self._last_enable_channel = None
        self._last_direction_channel = None
        self._last_cancel_channel = 0
        self._last_position_trigger_channel = 0
        self._last_servo_channel = None
        self._last_clear_alarm_channel = 0
        self._last_back_home_channel = 0
        self._last_set_home_channel = 0
        self._last_reset_initial_abs_pos_channel = 0

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @staticmethod
    def parse_artdmx(packet: bytes):
        """Returns (universe, dmx_data_bytes) for a valid ArtDMX (OpOutput)
        packet, or None if `packet` isn't one (wrong ID/opcode/too short)."""
        if len(packet) < 18 or packet[:8] != ARTNET_ID:
            return None
        opcode = struct.unpack('<H', packet[8:10])[0]
        if opcode != OP_OUTPUT_DMX:
            return None
        sub_uni = packet[14]
        net = packet[15]
        universe = sub_uni | (net << 8)
        length = struct.unpack('>H', packet[16:18])[0]
        data = packet[18:18 + length]
        return universe, data

    def _handle_dmx(self, universe: int, data: bytes) -> None:
        if universe != self.universe or len(data) < 3:
            return

        enable_channel, direction_channel, cancel_channel = data[0], data[1], data[2]

        try:
            if cancel_channel > 0 and self._last_cancel_channel == 0:
                self.servo_ctrller.cancel_continuous_reading()
                logger.info("Art-Net: cancel triggered (channel 3 rising edge).")
            self._last_cancel_channel = cancel_channel

            if len(data) >= 7:
                self._handle_position_mode_channels(data)

            if len(data) >= 12:
                self._handle_extended_channels(data)

            if enable_channel == 0:
                if self._last_enable_channel not in (None, 0):
                    self.servo_ctrller.speed_ctrl_action(0)
                    logger.info("Art-Net: continuous motion disabled (channel 1 -> 0).")
            else:
                if self._last_enable_channel in (None, 0):
                    speed_rpm = max(1, round(enable_channel / 255 * self.max_speed_rpm))
                    self.servo_ctrller.enable_speed_ctrl(speed_rpm, self.acc_time, True)
                    logger.info(f"Art-Net: continuous motion enabled at {speed_rpm} rpm "
                                f"(channel 1 = {enable_channel}).")

                if direction_channel != self._last_direction_channel:
                    # Per docs/en_manual.txt:10380-10382 (JOG_OPERATION,
                    # 0x0904): 1 = forward rotation (CCW), 2 = reverse
                    # rotation (CW).
                    if direction_channel == 0:
                        action_value = 0
                    elif direction_channel < 128:
                        action_value = 1  # CCW
                    else:
                        action_value = 2  # CW
                    self.servo_ctrller.speed_ctrl_action(action_value)
                    logger.info(f"Art-Net: direction channel={direction_channel} -> "
                                f"action={action_value}")

            self._last_enable_channel = enable_channel
            self._last_direction_channel = direction_channel
        except Exception as e:
            logger.error(f"Error handling Art-Net DMX frame: {e}")

    def _handle_position_mode_channels(self, data: bytes) -> None:
        """Channels 4-7 -- absolute-angle position mode, the same call
        OSC's /set_point makes. Requires len(data) >= 7 (checked by the
        caller); a sender using only channels 1-3 never triggers this."""
        position_trigger_channel = data[3]
        angle_high_byte, angle_low_byte, speed_channel = data[4], data[5], data[6]

        if position_trigger_channel > 0 and self._last_position_trigger_channel == 0:
            angle_raw = (angle_high_byte << 8) | angle_low_byte
            angle = angle_raw / 65535 * self.position_mode_max_angle
            speed_rpm = max(1, round(speed_channel / 255 * self.max_speed_rpm))
            self.servo_ctrller.post_step_motion_by(angle, self.acc_time, speed_rpm)
            logger.info(
                f"Art-Net: position-mode move to {angle:.2f} deg at {speed_rpm} rpm "
                f"(channel 4 rising edge, channels 5-6 = {angle_raw}, channel 7 = {speed_channel})."
            )
        self._last_position_trigger_channel = position_trigger_channel

    def _handle_extended_channels(self, data: bytes) -> None:
        """Channels 8-12 -- servo on/off, clear alarm, back home, set home,
        reset initial absolute position. Requires len(data) >= 12 (checked
        by the caller); a sender using only channels 1-7 never triggers
        this. Mirrors OSC's /servo, /clear, /back_home, /set_home,
        /reset_initial_abs_position respectively."""
        servo_channel = data[7]
        clear_alarm_channel = data[8]
        back_home_channel = data[9]
        set_home_channel = data[10]
        reset_initial_abs_pos_channel = data[11]

        if servo_channel != self._last_servo_channel:
            if servo_channel == 0:
                if self._last_servo_channel not in (None, 0):
                    self.servo_ctrller.servo_off()
                    logger.info("Art-Net: servo off (channel 8 -> 0).")
            else:
                self.servo_ctrller.servo_on()
                logger.info(f"Art-Net: servo on (channel 8 = {servo_channel}).")
        self._last_servo_channel = servo_channel

        if clear_alarm_channel > 0 and self._last_clear_alarm_channel == 0:
            self.servo_ctrller.clear_alarm_12()
            logger.info("Art-Net: clear alarm 12 triggered (channel 9 rising edge).")
        self._last_clear_alarm_channel = clear_alarm_channel

        if back_home_channel > 0 and self._last_back_home_channel == 0:
            self.servo_ctrller.initial_abs_home()
            logger.info("Art-Net: back home triggered (channel 10 rising edge).")
        self._last_back_home_channel = back_home_channel

        if set_home_channel > 0 and self._last_set_home_channel == 0:
            self.servo_ctrller.set_home_position()
            logger.info("Art-Net: set home triggered (channel 11 rising edge).")
        self._last_set_home_channel = set_home_channel

        if reset_initial_abs_pos_channel > 0 and self._last_reset_initial_abs_pos_channel == 0:
            self.servo_ctrller.write_PA29_Initial_Abs_Pos()
            logger.info("Art-Net: reset initial absolute position triggered (channel 12 rising edge).")
        self._last_reset_initial_abs_pos_channel = reset_initial_abs_pos_channel

    def _serve(self) -> None:
        self._sock.settimeout(0.5)
        while not self._stop_event.is_set():
            try:
                packet, _addr = self._sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            parsed = self.parse_artdmx(packet)
            if parsed is None:
                continue
            universe, data = parsed
            self._handle_dmx(universe, data)

    def start(self) -> None:
        if self.is_running:
            raise RuntimeError("Art-Net server is already running.")
        self._stop_event.clear()
        self._last_enable_channel = None
        self._last_direction_channel = None
        self._last_cancel_channel = 0
        self._last_position_trigger_channel = 0
        self._last_servo_channel = None
        self._last_clear_alarm_channel = 0
        self._last_back_home_channel = 0
        self._last_set_home_channel = 0
        self._last_reset_initial_abs_pos_channel = 0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.listen_ip, self.listen_port))
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        logger.info(
            f"Art-Net server listening on {self.listen_ip}:{self.listen_port} "
            f"(universe {self.universe})"
        )

    def stop(self) -> None:
        if not self.is_running:
            return
        self._stop_event.set()
        self._thread.join(timeout=2)
        if self._sock is not None:
            self._sock.close()
        self._sock = None
        self._thread = None
        logger.info("Art-Net server stopped.")
