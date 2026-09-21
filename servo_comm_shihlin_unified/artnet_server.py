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
      maps to speed_rpm (scaled by `max_speed_rpm`, minimum 1 rpm). The speed
      is applied when motion starts AND whenever the value changes while it
      runs (at most one write per 100 ms, latest value wins).
  Channel 2 (direction): 0 = stop; 1-127 = CCW; 128-255 = CW. It may be set
      before or after channel 1: the direction is always sent when motion
      starts. The drive refuses a direct CW<->CCW switch; that is reported
      (log, Channel Monitor, stats) and the switch happens once channel 2 has
      gone through 0.
  Channel 3 (cancel): 0 = normal; a 0->nonzero transition triggers the same
      cancel behavior as OSC's /cancel_loop. If continuous motion was
      requested (channel 1 > 0) it stays off, and channels 1-2 are ignored,
      until channel 1 has been set to 0.
  Channel 4 (position-mode trigger): 0 = idle; a 0->nonzero transition
      triggers a single absolute-angle move (ServoController.post_step_motion_by(),
      the same call OSC's /set_point makes) using whatever channels 5-7
      currently hold. Optional -- a sender that only ever fills channels
      1-3 (continuous mode only) still works, this is just never triggered.
  Channel 5-6 (target angle, high byte/low byte): 16-bit big-endian, 0.01
      deg per step, 32768 = 0 deg: angle = (value - 32768) / 100, so 90 deg is
      32768 + 9000 = 41768 (about +-327 deg). There is no limit on how far
      the resulting move may be.
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
  Channels 13-14 (relative move angle), 15-16 (move time) and 17 (trigger):
  a second, independent way to move -- "move BY this angle in this many
  seconds" instead of "go to this absolute angle at this rpm". It has its OWN
  trigger (channel 17) and never involves channels 4-7, just as channel 4
  never looks at channels 13-16. The rpm is calculated here, so the sender
  never has to know the gear ratio.
    Channel 13-14 (high byte/low byte): 16-bit value, 32768 = no move; each
        step is 0.01 deg, so 32768+n = +n/100 deg and 32768-n = -n/100 deg
        (range about +-327 deg; no limit on the distance moved).
        Positive = the direction in which the tracked angle increases.
    Channel 15-16 (high byte/low byte): 16-bit duration in 0.01 s steps
        (0.01 .. 655.35 s). 0 = no time given: the move is refused.
    Channel 17 (move-by trigger): 0 = idle; a 0->nonzero transition moves to
        (current angle + relative angle). If channel 4 also rises in the same
        frame, only the absolute move runs. rpm = motor revolutions / seconds * 60,
  worked out from PULSES (angle * base_pulse_per_degree, which already
  includes the gear ratio) so no division by an angle is needed and the only
  divisor (the duration) is checked to be non-zero first; it is then rounded
  to a whole rpm, at least 1 and at most `max_speed_rpm` (if the requested
  time is too short for that limit, the move simply takes longer). The
  acceleration ramp (`acc_time`) is not part of the requested time.

All channels are edge-triggered against the previously-seen frame, not
re-sent on every DMX refresh (a real Art-Net source typically resends the
full frame 30-44 times/second even when nothing changed) -- otherwise e.g.
enable_speed_ctrl() or post_step_motion_by() would fire on every single
frame instead of once per real state change.

Network safety (see OSC_ARTNET_GUIDE.md, "Network setup and safety"): this
is a pure receiver -- it never sends a packet. Frames are accepted only for
one universe (default 1, not the 0 other DMX equipment usually uses) and,
optionally, only from listed sender IPs; bind listen_ip to the NIC address
the sender unicasts to; continuous rotation is stopped if the signal is lost
(signal_timeout_s) and does not restart by itself; channels 10-12 are ignored
unless enable_dangerous_channels is set.

No external Art-Net library dependency: ArtDMX's binary header is small
and stable, so it's parsed by hand rather than adding a new
requirements.txt entry for it.
"""
import logging
import socket
import struct
import threading
import time
from collections import deque

logger = logging.getLogger(__name__)

ARTNET_PORT = 6454
# Default universe for THIS server. 0 is what other DMX equipment on the same
# network usually listens on; using it here risks that equipment reading our
# motor channels (and us reading its data).
DEFAULT_UNIVERSE = 1
# No frame for our universe for this long -> stop continuous rotation.
# Comfortably above Wi-Fi latency spikes; 0 disables.
DEFAULT_SIGNAL_TIMEOUT_S = 2.0
ARTNET_ID = b"Art-Net\x00"
OP_OUTPUT_DMX = 0x5000

# Angle encoding shared by channels 5-6 (absolute target) and 13-14 (relative
# move): 16 bits, 0.01 deg per step, 32768 = 0 deg.
ANGLE_CENTER = 32768
ANGLE_STEP_DEG = 0.01
DURATION_STEP_S = 0.01          # seconds per raw step of channels 15-16

# Channel 1 speed changes while running are written at most this often.
SPEED_UPDATE_MIN_INTERVAL_S = 0.1
# A direction write the drive did not answer is retried no sooner than this.
DIRECTION_RETRY_INTERVAL_S = 0.3


def decode_angle_centideg(high_byte: int, low_byte: int) -> int:
    """Hundredths of a degree from a channel pair (5-6 or 13-14): an integer,
    so nothing depends on floating-point angles until the very end."""
    return ((high_byte << 8) | low_byte) - ANGLE_CENTER


def decode_relative_move(data: bytes):
    """(delta_centideg, duration_centisec) from channels 13-16, or None if the
    frame is shorter than 16 channels."""
    if len(data) < 16:
        return None
    return decode_angle_centideg(data[12], data[13]), (data[14] << 8) | data[15]


class ArtNetInputServer:
    def __init__(self, servo_ctrller, listen_ip="0.0.0.0", listen_port=ARTNET_PORT,
                 universe=DEFAULT_UNIVERSE, max_speed_rpm=100, acc_time=5000,
                 signal_timeout_s=DEFAULT_SIGNAL_TIMEOUT_S,
                 allowed_sources=None, enable_dangerous_channels=False):
        """listen_ip: bind to the NIC address the sender unicasts to (a
        specific address only receives unicast; Linux does not deliver
        broadcast to a socket bound to a unicast address). universe: default
        1 -- other DMX equipment on the same network commonly sits on 0, and
        if it sees these frames it would drive its outputs from our motor
        channels. signal_timeout_s: stop continuous rotation if no frame for
        our universe arrives for this long (0 disables). allowed_sources:
        only accept frames from these sender IPs (None/empty = any).
        enable_dangerous_channels: channels 10-12 (back home = real move,
        set home = overwrites the saved home, reset abs position) are
        ignored unless this is True."""
        self.servo_ctrller = servo_ctrller
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.universe = universe
        self.signal_timeout_s = float(signal_timeout_s or 0)
        self.allowed_sources = frozenset(allowed_sources or ())
        self.enable_dangerous_channels = bool(enable_dangerous_channels)
        self.max_speed_rpm = max_speed_rpm
        self.acc_time = acc_time

        self._sock = None
        self._thread = None
        self._stop_event = threading.Event()

        self._last_enable_channel = None
        # The direction channel value the drive has actually ACCEPTED (None =
        # nothing sent since motion started, so the next start must send it).
        self._last_direction_channel = None
        # A requested direction the drive refused (a direct CW<->CCW switch);
        # not asked again until channel 2 changes.
        self._refused_direction = None
        self._last_direction_failure_monotonic = -1e9
        self._last_cancel_channel = 0
        self._last_position_trigger_channel = 0
        self._last_relative_trigger_channel = 0
        self._last_servo_channel = None
        self._last_clear_alarm_channel = 0
        self._last_back_home_channel = 0
        self._last_set_home_channel = 0
        self._last_reset_initial_abs_pos_channel = 0
        # After a channel-3 cancel while motion was requested: channels 1-2 are
        # ignored until channel 1 returns to 0 (see _handle_dmx).
        self._cancel_latched = False
        # Live speed: the rpm channel 1 asks for now, the rpm last written to
        # the drive, and when (see _apply_pending_speed()).
        self._desired_speed_rpm = None
        self._applied_speed_rpm = None
        self._last_speed_write_monotonic = 0.0

        # Raw bytes of the most recently received DMX frame (for the
        # configured universe only), plus when it arrived -- for the web
        # UI's Art-Net Channel Monitor (get_channel_snapshot()), so a user
        # can see what a console/controller actually sent without an
        # external DMX tool.
        self._last_frame_data = None
        self._last_frame_time = None

        # Loss-of-signal protection and receive statistics -- see
        # _check_signal_watchdog() / get_stats().
        self._last_valid_frame_monotonic = None
        self._watchdog_tripped = False
        self._frame_times = deque(maxlen=64)
        self._last_sequence = None
        self._last_sequence_time = 0.0
        self._stats = {"frames_ok": 0, "dropped_source": 0, "dropped_universe": 0,
                       "dropped_sequence": 0, "watchdog_trips": 0, "direction_refusals": 0}

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
        if universe != self.universe:
            self._stats["dropped_universe"] += 1
            return
        if len(data) < 3:
            return

        self._last_frame_data = data
        self._last_frame_time = time.time()
        now = time.monotonic()
        self._last_valid_frame_monotonic = now
        self._frame_times.append(now)
        self._stats["frames_ok"] += 1

        enable_channel, direction_channel, cancel_channel = data[0], data[1], data[2]

        try:
            if cancel_channel > 0 and self._last_cancel_channel == 0:
                self.servo_ctrller.cancel_continuous_reading()
                logger.info("Art-Net: cancel triggered (channel 3 rising edge).")
                if enable_channel != 0 or self._last_enable_channel not in (None, 0):
                    # The drive has left JOG mode. Writing speed/direction now
                    # would go to a mode that is no longer active, so hold
                    # everything until the sender sets channel 1 to 0.
                    self._cancel_latched = True
                    self._last_enable_channel = 0
                    self._reset_motion_state()
                    logger.info("Art-Net: continuous motion is off until channel 1 is set to 0.")
            self._last_cancel_channel = cancel_channel

            absolute_fired = False
            if len(data) >= 7:
                absolute_fired = self._handle_position_mode_channels(data)

            if len(data) >= 17:
                self._handle_relative_move_channel(data, absolute_fired)

            if len(data) >= 12:
                self._handle_extended_channels(data)

            if self._watchdog_tripped or self._cancel_latched:
                # Signal lost, or the sender cancelled: resuming by itself would
                # be a surprise start, so continuous motion stays off until the
                # sender explicitly sets channel 1 to 0 (which re-arms it).
                if enable_channel != 0:
                    return
                was = "cancel" if self._cancel_latched else "signal loss"
                self._watchdog_tripped = False
                self._cancel_latched = False
                logger.info(f"Art-Net: channel 1 = 0 after {was}; continuous motion re-armed.")

            if enable_channel == 0:
                if self._last_enable_channel not in (None, 0):
                    self.servo_ctrller.speed_ctrl_action(0)
                    logger.info("Art-Net: continuous motion disabled (channel 1 -> 0).")
                self._reset_motion_state()
            else:
                speed_rpm = max(1, round(enable_channel / 255 * self.max_speed_rpm))
                if self._last_enable_channel in (None, 0):
                    self.servo_ctrller.enable_speed_ctrl(speed_rpm, self.acc_time, True)
                    # Motion is armed from here on, even if the direction write
                    # below gets no answer: a failure there must not run the
                    # whole start sequence again on the next frame.
                    self._last_enable_channel = enable_channel
                    self._desired_speed_rpm = self._applied_speed_rpm = speed_rpm
                    self._last_speed_write_monotonic = now
                    # enable_speed_ctrl() re-enters JOG mode, so the direction
                    # has to be sent again -- even if channel 2 was already
                    # set before channel 1 came up, or did not change since
                    # the last run.
                    self._last_direction_channel = None
                    self._refused_direction = None
                    logger.info(f"Art-Net: continuous motion enabled at {speed_rpm} rpm "
                                f"(channel 1 = {enable_channel}).")
                else:
                    self._desired_speed_rpm = speed_rpm
                    self._apply_pending_speed(now)

                self._apply_direction(direction_channel, now)

            self._last_enable_channel = enable_channel
        except Exception as e:
            logger.error(f"Error handling Art-Net DMX frame: {e}")

    def _reset_motion_state(self) -> None:
        """Forget what was sent for continuous motion (it stopped or JOG was
        left), so the next start sends everything again."""
        self._last_direction_channel = None
        self._refused_direction = None
        self._desired_speed_rpm = None
        self._applied_speed_rpm = None

    def _apply_direction(self, direction_channel: int, now: float) -> None:
        """Sends the direction channel 2 asks for, once per change. The drive
        refuses a direct CW<->CCW switch (speed_ctrl_action() returns False);
        that is remembered so it is neither retried on every frame nor mistaken
        for done, and it clears when channel 2 changes again. A write the drive
        did not answer (seen on the real drive right after JOG mode is entered)
        is not counted as applied and is retried after DIRECTION_RETRY_INTERVAL_S,
        never on every frame."""
        if direction_channel == self._last_direction_channel:
            self._refused_direction = None
            return
        if direction_channel == self._refused_direction:
            return
        # Per docs/en_manual.txt:10380-10382 (JOG_OPERATION, 0x0904): 1 =
        # forward rotation (CCW), 2 = reverse rotation (CW).
        if direction_channel == 0:
            action_value = 0
        elif direction_channel < 128:
            action_value = 1  # CCW
        else:
            action_value = 2  # CW
        if now - self._last_direction_failure_monotonic < DIRECTION_RETRY_INTERVAL_S:
            return
        try:
            applied = self.servo_ctrller.speed_ctrl_action(action_value)
        except Exception as e:
            self._last_direction_failure_monotonic = now
            logger.warning(f"Art-Net: direction write got no usable answer ({e}); will retry.")
            return
        if applied is False:
            self._refused_direction = direction_channel
            self._stats["direction_refusals"] += 1
            logger.warning(
                f"Art-Net: direction change refused by the drive (channel 2 = {direction_channel}): "
                "a direct CW<->CCW switch is not allowed -- set channel 2 to 0 first."
            )
            return
        self._last_direction_channel = direction_channel
        self._refused_direction = None
        logger.info(f"Art-Net: direction channel={direction_channel} -> action={action_value}")

    def _apply_pending_speed(self, now=None) -> None:
        """Writes the speed channel 1 asks for (0x0903) when it differs from
        what the drive has, at most once per SPEED_UPDATE_MIN_INTERVAL_S so a
        fader move does not flood the serial line; the latest value wins. Also
        called by the receive loop when no frames arrive, so the last value is
        not left pending. Does nothing unless continuous motion is running."""
        if self._cancel_latched or self._watchdog_tripped:
            return
        if self._last_enable_channel in (None, 0):
            return
        desired = self._desired_speed_rpm
        if desired is None or desired == self._applied_speed_rpm:
            return
        now = time.monotonic() if now is None else now
        if now - self._last_speed_write_monotonic < SPEED_UPDATE_MIN_INTERVAL_S:
            return
        # Whatever happens, do not try again before the interval has passed.
        self._last_speed_write_monotonic = now
        try:
            self.servo_ctrller.config_speed_0x0903(desired)
        except Exception as e:
            logger.error(f"Art-Net: could not change the speed to {desired} rpm: {e}")
            return
        self._applied_speed_rpm = desired
        logger.info(f"Art-Net: speed changed to {desired} rpm (channel 1).")

    def _handle_position_mode_channels(self, data: bytes) -> bool:
        """Channels 4-7 -- absolute-angle position mode, the same call OSC's
        /set_point makes. Requires len(data) >= 7 (checked by the caller); a
        sender using only channels 1-3 never triggers this. Uses channels 5-7
        only -- never channels 13-17. Returns True when this frame carried the
        trigger's rising edge (whether the move ran or was refused)."""
        position_trigger_channel = data[3]
        fired = False

        if position_trigger_channel > 0 and self._last_position_trigger_channel == 0:
            fired = True
            angle_centideg = decode_angle_centideg(data[4], data[5])
            angle = angle_centideg / 100
            speed_channel = data[6]
            speed_rpm = max(1, round(speed_channel / 255 * self.max_speed_rpm))
            # Consume the edge BEFORE moving: a refused move (position
            # unreadable) must not be retried on every 30-44 fps frame.
            self._last_position_trigger_channel = position_trigger_channel
            try:
                self.servo_ctrller.post_step_motion_by(angle, self.acc_time, speed_rpm)
            except Exception as e:
                logger.error(f"Art-Net: position-mode move to {angle:.2f} deg refused: {e}")
                return fired
            logger.info(
                f"Art-Net: position-mode move to {angle:.2f} deg at {speed_rpm} rpm "
                f"(channel 4 rising edge, channels 5-6 = {(data[4] << 8) | data[5]}, "
                f"channel 7 = {speed_channel})."
            )
        self._last_position_trigger_channel = position_trigger_channel
        return fired

    def _handle_relative_move_channel(self, data: bytes, absolute_fired: bool) -> None:
        """Channel 17 -- move BY the angle in channels 13-14 in the time in
        channels 15-16. Requires len(data) >= 17 (checked by the caller). Uses
        channels 13-16 only -- never channels 4-7."""
        trigger = data[16]
        if trigger > 0 and self._last_relative_trigger_channel == 0:
            # Consume the edge first, like the absolute trigger.
            self._last_relative_trigger_channel = trigger
            if absolute_fired:
                logger.warning(
                    "Art-Net: channels 4 and 17 both fired in one frame; only the absolute "
                    "move (channel 4) was run.")
                return
            delta_centideg, duration_centisec = decode_relative_move(data)
            if duration_centisec == 0:
                logger.warning(
                    "Art-Net: move-by trigger (channel 17) refused: the time in channels "
                    "15-16 is 0.")
                return
            self._move_by_angle_in_time(delta_centideg, duration_centisec)
        self._last_relative_trigger_channel = trigger

    def calculate_move_rpm(self, delta_centideg: int, duration_centisec: int) -> int:
        """rpm needed to turn the output shaft by delta_centideg (hundredths of
        a degree) in duration_centisec (hundredths of a second), as a whole
        number in [1, max_speed_rpm].

        Done in pulses: pulses = |angle| * base_pulse_per_degree (already
        includes the gear ratio), motor revolutions = pulses / encoder pulses
        per rev, so
            rpm = pulses / pulses_per_rev / (centisec / 100) * 60
                = pulses * 6000 / (pulses_per_rev * centisec).
        The divisor is a positive constant times the duration, and the
        duration is rejected when it is not positive -- there is no way to
        divide by zero, whatever the angle is (a zero angle simply gives the
        minimum rpm and is not moved by the caller anyway)."""
        if duration_centisec <= 0:
            raise ValueError("duration must be greater than zero")
        pulses = round(abs(delta_centideg) * self.servo_ctrller.base_pulse_per_degree / 100)
        pulses_per_rev = self.servo_ctrller.profile["encoder_pulses_per_rev"]
        rpm = round(pulses * 6000 / (pulses_per_rev * duration_centisec))
        return max(1, min(rpm, self.max_speed_rpm))

    def _move_by_angle_in_time(self, delta_centideg: int, duration_centisec: int) -> None:
        """Channel-17 trigger with channels 13-16: relative move over a time.
        The trigger edge was consumed by the caller, so a refused move is not
        retried on every frame."""
        if delta_centideg == 0:
            logger.info("Art-Net: relative move of 0 deg ignored (channels 13-14 = 32768).")
            return
        try:
            speed_rpm = self.calculate_move_rpm(delta_centideg, duration_centisec)
            delta_deg = delta_centideg / 100
            self.servo_ctrller.post_step_motion_by(
                delta_deg, self.acc_time, speed_rpm, relative=True)
        except Exception as e:
            logger.error(
                f"Art-Net: relative move of {delta_centideg / 100:+.2f} deg in "
                f"{duration_centisec / 100:g} s refused: {e}")
            return
        logger.info(
            f"Art-Net: relative move {delta_centideg / 100:+.2f} deg in "
            f"{duration_centisec / 100:g} s -> {speed_rpm} rpm (channel 17 rising edge)."
        )

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

        # Channels 10-12 can make a real move / overwrite the saved home, so
        # they are ignored (with a log line per rising edge) unless the
        # server was started with enable_dangerous_channels.
        if back_home_channel > 0 and self._last_back_home_channel == 0:
            if self._dangerous_channel_allowed(10, "back home"):
                self.servo_ctrller.initial_abs_home()
                logger.info("Art-Net: back home triggered (channel 10 rising edge).")
        self._last_back_home_channel = back_home_channel

        if set_home_channel > 0 and self._last_set_home_channel == 0:
            if self._dangerous_channel_allowed(11, "set home"):
                self.servo_ctrller.set_home_position()
                logger.info("Art-Net: set home triggered (channel 11 rising edge).")
        self._last_set_home_channel = set_home_channel

        if reset_initial_abs_pos_channel > 0 and self._last_reset_initial_abs_pos_channel == 0:
            if self._dangerous_channel_allowed(12, "reset initial absolute position"):
                self.servo_ctrller.write_PA29_Initial_Abs_Pos()
                logger.info("Art-Net: reset initial absolute position triggered (channel 12 rising edge).")
        self._last_reset_initial_abs_pos_channel = reset_initial_abs_pos_channel

    def _dangerous_channel_allowed(self, channel: int, what: str) -> bool:
        if self.enable_dangerous_channels:
            return True
        logger.warning(
            f"Art-Net: channel {channel} ({what}) ignored -- dangerous channels 10-12 are "
            "disabled (enable them when starting the Art-Net server)."
        )
        return False

    def _check_signal_watchdog(self, now=None) -> None:
        """Stops continuous rotation when frames for our universe have stopped
        arriving (sender crashed, cable pulled, Wi-Fi dropped). Without this
        the last commanded rotation would simply continue: the drive's own
        communication timeout cannot help because this app keeps polling it.
        Only acts while continuous motion is active; latches (see
        _handle_dmx) so it does not restart by itself."""
        if not self.signal_timeout_s or self._watchdog_tripped:
            return
        if self._last_enable_channel in (None, 0) or self._last_valid_frame_monotonic is None:
            return
        now = time.monotonic() if now is None else now
        if now - self._last_valid_frame_monotonic < self.signal_timeout_s:
            return
        self._watchdog_tripped = True
        self._stats["watchdog_trips"] += 1
        logger.warning(
            f"Art-Net: no frame for universe {self.universe} for {self.signal_timeout_s:g}s -- "
            "stopping continuous rotation."
        )
        try:
            self.servo_ctrller.speed_ctrl_action(0)
        except Exception as e:
            logger.error(f"Art-Net: failed to stop rotation after signal loss: {e}")
        # Motion stopped: whatever was sent for it has to be sent again on restart.
        self._reset_motion_state()

    def get_stats(self) -> dict:
        """Receive statistics for the web UI: lets a user see the sender's real
        frame rate and whether stray/foreign packets are being dropped."""
        now = time.monotonic()
        age = None if self._last_valid_frame_monotonic is None else now - self._last_valid_frame_monotonic
        fps = None
        if len(self._frame_times) >= 2 and age is not None and age < 5:
            span = self._frame_times[-1] - self._frame_times[0]
            fps = (len(self._frame_times) - 1) / span if span > 0 else None
        return {
            **self._stats,
            "frame_age_s": age,
            "frames_per_s": fps,
            "signal_timeout_s": self.signal_timeout_s,
            "watchdog_tripped": self._watchdog_tripped,
            "cancel_latched": self._cancel_latched,
            "direction_refused": self._refused_direction is not None,
            "allowed_sources": sorted(self.allowed_sources),
            "dangerous_channels_enabled": self.enable_dangerous_channels,
            "listen_ip": self.listen_ip,
            "universe": self.universe,
        }

    def get_channel_snapshot(self):
        """Interpreted state of the most recently received DMX frame, for
        the web UI's Art-Net Channel Monitor -- lets a user confirm what a
        console/controller actually sent without an external DMX tool.
        Returns None if no frame has arrived yet for the configured
        universe. Read-only; never touches the driver."""
        data = self._last_frame_data
        if data is None:
            return None

        def entry(channel, label, value, interpreted):
            return {"channel": channel, "label": label, "value": value, "interpreted": interpreted}

        channels = []
        enable = data[0]
        channels.append(entry(
            1, "Enable/Speed", enable,
            "disabled" if enable == 0 else f"{max(1, round(enable / 255 * self.max_speed_rpm))} rpm"
        ))
        direction = data[1] if len(data) > 1 else 0
        direction_label = "stop" if direction == 0 else ("CCW" if direction < 128 else "CW")
        if self._refused_direction is not None and direction == self._refused_direction:
            direction_label += " - REFUSED by the drive (set channel 2 to 0 first)"
        channels.append(entry(2, "Direction", direction, direction_label))
        cancel = data[2] if len(data) > 2 else 0
        cancel_note = "triggered" if cancel > 0 else "idle"
        if self._cancel_latched:
            cancel_note += " - motion held off: set channel 1 to 0"
        channels.append(entry(3, "Cancel", cancel, cancel_note))

        if len(data) >= 7:
            trigger, angle_high, angle_low, speed_ch = data[3], data[4], data[5], data[6]
            angle = decode_angle_centideg(angle_high, angle_low) / 100
            channels.append(entry(4, "Position trigger", trigger, "armed" if trigger > 0 else "idle"))
            channels.append(entry(5, "Angle (high byte)", angle_high, f"{angle:+.2f} deg (combined w/ ch 6)"))
            channels.append(entry(6, "Angle (low byte)", angle_low, ""))
            channels.append(entry(
                7, "Position speed", speed_ch,
                f"{max(1, round(speed_ch / 255 * self.max_speed_rpm))} rpm"
            ))

        if len(data) >= 12:
            servo, clear, back_home, set_home, reset_abs = data[7], data[8], data[9], data[10], data[11]
            channels.append(entry(8, "Servo on/off", servo, "on" if servo > 0 else "off"))
            channels.append(entry(9, "Clear Alarm 12", clear, "triggered" if clear > 0 else "idle"))
            note = "" if self.enable_dangerous_channels else " - IGNORED (channels 10-12 disabled)"
            channels.append(entry(10, "Back home", back_home, ("triggered" + note) if back_home > 0 else "idle"))
            channels.append(entry(11, "Set home", set_home, ("triggered" + note) if set_home > 0 else "idle"))
            channels.append(entry(12, "Reset initial abs pos", reset_abs, ("triggered" + note) if reset_abs > 0 else "idle"))

        relative = decode_relative_move(data)
        if relative is not None:
            delta_centideg, duration_centisec = relative
            if duration_centisec > 0:
                rpm = self.calculate_move_rpm(delta_centideg, duration_centisec)
                summary = f"{delta_centideg / 100:+.2f} deg in {duration_centisec / 100:g} s = {rpm} rpm"
                angle_note = f"{delta_centideg / 100:+.2f} deg (combined w/ ch 14)"
                time_note = f"{duration_centisec / 100:g} s (combined w/ ch 16)"
                trigger_note = summary
            else:
                angle_note = f"{delta_centideg / 100:+.2f} deg (ch 17 is refused while the time is 0)"
                time_note = "0 = no time given"
                trigger_note = None
            channels.append(entry(13, "Move by angle (high byte)", data[12], angle_note))
            channels.append(entry(14, "Move by angle (low byte)", data[13], ""))
            channels.append(entry(15, "Move time (high byte)", data[14], time_note))
            channels.append(entry(16, "Move time (low byte)", data[15], trigger_note or ""))
        if len(data) >= 17:
            channels.append(entry(17, "Move-by trigger", data[16], "armed" if data[16] > 0 else "idle"))

        return {"received_at": self._last_frame_time, "channels": channels}

    def _process_packet(self, packet: bytes, source_ip: str) -> None:
        """One received UDP datagram: source filter, ArtDMX parse, sequence
        check, then the per-universe handling. Never sends anything."""
        if self.allowed_sources and source_ip not in self.allowed_sources:
            self._stats["dropped_source"] += 1
            return
        parsed = self.parse_artdmx(packet)
        if parsed is None:
            return
        universe, data = parsed
        if universe == self.universe and not self._sequence_is_current(packet[12]):
            self._stats["dropped_sequence"] += 1
            return
        self._handle_dmx(universe, data)

    def _sequence_is_current(self, sequence: int) -> bool:
        """Art-Net sequence byte (0 = disabled). Drops a packet that is
        slightly BEHIND the last one (reordered by the network) so a stale
        frame cannot overwrite a newer one. A sender that restarted (its
        counter resets) is accepted after a quiet second."""
        if sequence == 0:
            return True
        now = time.monotonic()
        if self._last_sequence is not None and now - self._last_sequence_time <= 1.0:
            behind = (self._last_sequence - sequence) % 256
            if 0 < behind < 128:
                return False
        self._last_sequence = sequence
        self._last_sequence_time = now
        return True

    def _serve(self) -> None:
        self._sock.settimeout(0.5)
        while not self._stop_event.is_set():
            try:
                packet, addr = self._sock.recvfrom(1024)
            except socket.timeout:
                packet = None
            except OSError:
                break
            if packet is not None:
                self._process_packet(packet, addr[0])
            # Every iteration (a timeout, or a foreign-universe packet
            # arriving in a stream) -- the check must not depend on a
            # packet for OUR universe showing up.
            self._check_signal_watchdog()
            # ...and the last speed change must not stay pending when the
            # sender stops transmitting for a moment.
            try:
                self._apply_pending_speed()
            except Exception as e:
                logger.error(f"Art-Net: pending speed change failed: {e}")

    def start(self) -> None:
        if self.is_running:
            raise RuntimeError("Art-Net server is already running.")
        self._stop_event.clear()
        self._last_enable_channel = None
        self._reset_motion_state()
        self._cancel_latched = False
        self._last_cancel_channel = 0
        self._last_position_trigger_channel = 0
        self._last_relative_trigger_channel = 0
        self._last_servo_channel = None
        self._last_clear_alarm_channel = 0
        self._last_back_home_channel = 0
        self._last_set_home_channel = 0
        self._last_reset_initial_abs_pos_channel = 0
        self._last_frame_data = None
        self._last_frame_time = None
        self._last_valid_frame_monotonic = None
        self._watchdog_tripped = False
        self._frame_times.clear()
        self._last_sequence = None
        self._stats = {key: 0 for key in self._stats}
        # No SO_REUSEADDR: for UDP it lets a second process bind the same
        # port and silently take frames (e.g. an accidentally duplicated
        # service); a clear "address in use" error is what we want instead.
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self.listen_ip, self.listen_port))
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        logger.info(
            f"Art-Net server listening on {self.listen_ip}:{self.listen_port} "
            f"(universe {self.universe}, signal timeout {self.signal_timeout_s:g}s, "
            f"sources {sorted(self.allowed_sources) or 'any'}, "
            f"channels 10-12 {'ENABLED' if self.enable_dangerous_channels else 'disabled'})"
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
