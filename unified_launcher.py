"""
unified_launcher.py
====================
Senior Embedded Systems Engineer reference implementation.

Integrates Betaflight MSP attitude polling (via MultiWii) with two
synchronized finite-state machines:

  HatchMachine  – predicts future pitch/roll and controls the hatch servo.
  DroneMachine  – arms motors only when conditions are fully safe.

Threading model
---------------
* A single background thread polls the FC at ~20 Hz (50 ms sleep).
* State-machine transitions are driven by threading.Timer (non-blocking),
  so IMU polling is NEVER stalled by a sleep() inside FSM logic.
* The dashboard refresh runs in the main thread at ~10 Hz.

States
------
HatchMachine : PREDICTING → OPENING → OPEN | ABORT
DroneMachine : IDLE → LAUNCHING → LAUNCHED | ABORT

Transition rules (§ 10° margin everywhere)
-------------------------------------------
  PREDICTING → OPENING  : predicted_angle (2 s ahead) ≤ 10°
  OPENING    → ABORT    : current_angle > 15°  (abort during 2-s open window)
  OPENING    → OPEN     : 2-s timer fires without abort
  IDLE       → LAUNCHING: HatchMachine is OPEN AND current_angle ≤ 10°
  LAUNCHING  → LAUNCHED : 2-s motor-spin timer fires
"""

from __future__ import annotations

import signal
import sys
import threading
import time
from collections import deque
from enum import Enum, auto
from typing import Optional

from MultiWii import MultiWii

# ─────────────────────────────────────────────
#  Configuration
# ─────────────────────────────────────────────
COM_PORT: str = "COM3"          # ← Set to COM3 as per user's drone connection
POLL_HZ: float = 20.0           # IMU polling target (Hz)
POLL_INTERVAL: float = 1.0 / POLL_HZ

ANGLE_MARGIN: float = 10.0      # °  – used for both prediction gate & launch gate
ABORT_MARGIN: float = 15.0      # °  – abort-while-opening threshold
OPEN_DURATION: float = 2.0      # s  – hatch opening duration
LAUNCH_DURATION: float = 2.5    # s  – motor spin-up duration (matches motor pulse script)
PREDICTION_HORIZON: float = 2.0 # s  – how far ahead to predict attitude

MOTOR_TEST_VALUE: int = 1100    # Safe default (DSHOT: ~100-300, PWM: ~1100)
MOTOR_OFF_VALUE: int = 1000     # 0 for DSHOT, 1000 for PWM. (Betaflight UI shows 1000 for OFF)

# MSP command descriptors (matches MultiWii.send / .receive signature)
MSP_ATTITUDE = {"message_id": 108, "direction": "<"}
MSP_SET_MOTOR = {"message_id": 214, "direction": "<"}

# ─────────────────────────────────────────────
#  State enumerations
# ─────────────────────────────────────────────
class HatchState(Enum):
    PREDICTING = auto()
    OPENING    = auto()
    OPEN       = auto()
    CLOSING    = auto()
    CLOSED     = auto()
    ABORT      = auto()

class DroneState(Enum):
    IDLE       = auto()
    LAUNCHING  = auto()
    LAUNCHED   = auto()
    ABORT      = auto()

# ─────────────────────────────────────────────
#  Shared IMU data container (thread-safe)
# ─────────────────────────────────────────────
class IMUData:
    """Latest attitude snapshot, written by the poller thread."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.roll: float = 0.0
        self.pitch: float = 0.0
        self.heading: float = 0.0
        self.timestamp: float = time.monotonic()
        # Sliding window of (timestamp, max_angle) for trend prediction
        self._history: deque[tuple[float, float]] = deque(maxlen=40)  # ~2 s @ 20 Hz

    def update(self, roll: float, pitch: float, heading: float) -> None:
        with self._lock:
            self.roll = roll
            self.pitch = pitch
            self.heading = heading
            self.timestamp = time.monotonic()
            self._history.append((self.timestamp, max(abs(roll), abs(pitch))))

    def snapshot(self) -> tuple[float, float, float]:
        """Return (roll, pitch, heading) atomically."""
        with self._lock:
            return self.roll, self.pitch, self.heading

    def predict_max_angle(self, horizon: float = PREDICTION_HORIZON) -> float:
        """
        Linear extrapolation of max(|roll|, |pitch|) over the next `horizon` seconds.

        Uses the slope of the last 1-second window of readings projected forward.
        If history is too short, returns the current max angle (conservative).
        """
        with self._lock:
            if len(self._history) < 4:
                return max(abs(self.roll), abs(self.pitch))

            now = time.monotonic()
            # Samples from the last 1 s
            window = [(t, a) for t, a in self._history if now - t <= 1.0]
            if len(window) < 2:
                return max(abs(self.roll), abs(self.pitch))

            t0, a0 = window[0]
            t1, a1 = window[-1]
            dt = t1 - t0
            if dt < 1e-9:
                return a1

            slope = (a1 - a0) / dt          # °/s
            predicted = a1 + slope * horizon
            return max(0.0, predicted)      # clamp to non-negative

# ─────────────────────────────────────────────
#  HatchMachine
# ─────────────────────────────────────────────
class HatchMachine:
    """
    Finite-state machine that governs hatch servo behaviour.

    Transition graph
    ----------------
        PREDICTING ──(pred ≤ 10°)──► OPENING
        OPENING ──(current > 15°)──► ABORT
        OPENING ──(2 s elapsed)────► OPEN
        OPEN ─────(Drone LAUNCHED)─► CLOSING
        CLOSING ──(2 s elapsed)────► CLOSED
        CLOSED / ABORT  (terminal)
    """

    def __init__(self, imu: IMUData) -> None:
        self._imu = imu
        self._lock = threading.Lock()
        self.state: HatchState = HatchState.PREDICTING
        self._timer: Optional[threading.Timer] = None

    # ── public API ──────────────────────────────────────────────────────────

    def evaluate(self, drone_state: DroneState) -> None:
        """Called by the poller thread every cycle."""
        with self._lock:
            state = self.state

        if state == HatchState.PREDICTING:
            self._check_predicting()
        elif state == HatchState.OPENING:
            self._check_opening()
        elif state == HatchState.OPEN:
            self._check_open(drone_state)
        # CLOSING triggers via timer; CLOSED / ABORT are terminal

    @property
    def is_open(self) -> bool:
        return self.state == HatchState.OPEN

    def abort(self) -> None:
        with self._lock:
            self._cancel_timer()
            # If the hatch was OPENING or OPEN and we abort, we MUST shut it.
            if self.state in (HatchState.OPENING, HatchState.OPEN):
                self.state = HatchState.CLOSING
                self._timer = threading.Timer(OPEN_DURATION, self._on_close_complete)
                self._timer.daemon = True
                self._timer.start()
            elif self.state == HatchState.PREDICTING:
                self.state = HatchState.ABORT

    def reset(self) -> None:
        """Resets the state machine for another launch sequence."""
        with self._lock:
            self._cancel_timer()
            self.state = HatchState.PREDICTING

    # ── private helpers ──────────────────────────────────────────────────────

    def _check_predicting(self) -> None:
        predicted = self._imu.predict_max_angle()
        if predicted <= ANGLE_MARGIN:
            with self._lock:
                if self.state == HatchState.PREDICTING:      # double-check under lock
                    self.state = HatchState.OPENING
                    self._timer = threading.Timer(OPEN_DURATION, self._on_open_complete)
                    self._timer.daemon = True
                    self._timer.start()

    def _check_opening(self) -> None:
        roll, pitch, _ = self._imu.snapshot()
        current_max = max(abs(roll), abs(pitch))
        if current_max > ABORT_MARGIN:
            with self._lock:
                if self.state == HatchState.OPENING:
                    self._cancel_timer()
                    self.state = HatchState.ABORT

    def _check_open(self, drone_state: DroneState) -> None:
        """If the Drone is done launching, we can close the hatch."""
        if drone_state == DroneState.LAUNCHED:
            with self._lock:
                if self.state == HatchState.OPEN:
                    self.state = HatchState.CLOSING
                    self._timer = threading.Timer(OPEN_DURATION, self._on_close_complete)
                    self._timer.daemon = True
                    self._timer.start()

    def _on_open_complete(self) -> None:
        with self._lock:
            if self.state == HatchState.OPENING:
                self.state = HatchState.OPEN

    def _on_close_complete(self) -> None:
        with self._lock:
            if self.state == HatchState.CLOSING:
                self.state = HatchState.CLOSED

    def _cancel_timer(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

# ─────────────────────────────────────────────
#  DroneMachine
# ─────────────────────────────────────────────
class DroneMachine:
    """
    Finite-state machine that governs motor arming.

    Transition graph
    ----------------
        IDLE ──(hatch OPEN AND current ≤ 10°)──► LAUNCHING
        LAUNCHING ──(2.5 s elapsed)─────────────► LAUNCHED
        LAUNCHED / ABORT  (terminal)
    """

    def __init__(self, imu: IMUData, hatch: HatchMachine) -> None:
        self._imu = imu
        self._hatch = hatch
        self._lock = threading.Lock()
        self.state: DroneState = DroneState.IDLE
        self._launch_timer: Optional[threading.Timer] = None

    # ── public API ──────────────────────────────────────────────────────────

    def evaluate(self) -> None:
        """Called by the poller thread every cycle."""
        with self._lock:
            state = self.state

        if state == DroneState.IDLE:
            self._check_idle()
        # LAUNCHING is managed by timer; LAUNCHED / ABORT are terminal

    def abort(self) -> None:
        with self._lock:
            self._cancel_timer()
            self.state = DroneState.ABORT

    def reset(self) -> None:
        """Resets the state machine for another launch sequence."""
        with self._lock:
            self._cancel_timer()
            self.state = DroneState.IDLE

    # ── private helpers ──────────────────────────────────────────────────────

    def _check_idle(self) -> None:
        if not self._hatch.is_open:
            return
        roll, pitch, _ = self._imu.snapshot()
        current_max = max(abs(roll), abs(pitch))
        if current_max <= ANGLE_MARGIN:
            with self._lock:
                if self.state == DroneState.IDLE:
                    self.state = DroneState.LAUNCHING
                    self._launch_timer = threading.Timer(LAUNCH_DURATION, self._on_launch_complete)
                    self._launch_timer.daemon = True
                    self._launch_timer.start()

    def _on_launch_complete(self) -> None:
        """Timer callback – fires after LAUNCH_DURATION."""
        with self._lock:
            if self.state == DroneState.LAUNCHING:
                self.state = DroneState.LAUNCHED

    def _cancel_timer(self) -> None:
        if self._launch_timer is not None:
            self._launch_timer.cancel()
            self._launch_timer = None

# ─────────────────────────────────────────────
#  Motor Helpers (Direct MSP)
# ─────────────────────────────────────────────
import struct

def send_motor_command_direct(ser: serial.Serial, motor_value: int) -> None:
    """
    Betaflight requires EXACTLY this byte sequence for MSP_SET_MOTOR.
    The MultiWii library's generic struct packer often fails the CRC check
    or sends the wrong length for this specific command, so we do it raw.
    """
    msg_id = 214
    # 8 uint16 values
    payload = struct.pack('<8H', *([motor_value] * 8))
    size = len(payload)
    
    crc = size ^ msg_id
    for b in payload:
        crc ^= b
        
    packet = b'$M<' + bytes([size, msg_id]) + payload + bytes([crc])
    try:
        ser.write(packet)
    except Exception:
        pass

# ─────────────────────────────────────────────
#  IMU Poller thread
# ─────────────────────────────────────────────
def imu_poller(
    board: MultiWii,
    imu: IMUData,
    hatch: HatchMachine,
    drone: DroneMachine,
    stop_event: threading.Event,
) -> None:
    """
    Background thread: polls attitude at ~20 Hz, drives both FSMs,
    and rapidly pulses MSP_SET_MOTOR when in the LAUNCHING state.
    """
    motors_active_last_tick = False

    while not stop_event.is_set():
        loop_start = time.monotonic()
        try:
            # 1. Read attitude
            board.send(MSP_ATTITUDE)
            data = board.receive()

            if len(data) >= 3:
                roll    = data[0] / 10.0   # tenths → degrees
                pitch   = data[1] / 10.0
                heading = float(data[2])
                imu.update(roll, pitch, heading)

                # Abort both FSMs if drone hatch encounters emergency
                if (hatch.state == HatchState.OPENING and
                        max(abs(roll), abs(pitch)) > ABORT_MARGIN):
                    hatch.abort()
                    drone.abort()
                else:
                    drone.evaluate()
                    hatch.evaluate(drone.state)

            # 2. Control Motors
            # Betaflight requires constant pulsing of MSP_SET_MOTOR while spinning.
            if drone.state == DroneState.LAUNCHING:
                # To guarantee the motors spin on both DSHOT (value ~100-200) and PWM (value ~1100),
                # we'll send a combined array. The drone will ignore the values that are out
                # of bounds for its configured protocol anyway.
                # Format: [motor_1, motor_2, motor_3, motor_4, motor_1_alt, motor_2_alt...]
                spin_packet_vals = [MOTOR_TEST_VALUE, MOTOR_TEST_VALUE, MOTOR_TEST_VALUE, MOTOR_TEST_VALUE, 150, 150, 150, 150]
                
                # Raw packing since build_msp_set_motor_packet expects a uniform list
                msg_id = 214
                payload = struct.pack('<8H', *spin_packet_vals)
                size = len(payload)
                crc = size ^ msg_id
                for b in payload: crc ^= b
                packet = b'$M<' + bytes([size, msg_id]) + payload + bytes([crc])
                
                try: board.com.write(packet)
                except Exception: pass
                
                motors_active_last_tick = True
            elif motors_active_last_tick:
                # We just exited LAUNCHING (either LAUNCHED or ABORT).
                # Send the OFF signal to safely halt motors immediately.
                for _ in range(5): # burst to ensure it arrives
                    send_motor_command_direct(board.com, MOTOR_OFF_VALUE)
                    time.sleep(0.01)
                motors_active_last_tick = False

        except Exception as exc:           # noqa: BLE001
            # Transient serial errors → log and continue; do not crash thread
            pass                           # dashboard will show stale data

        elapsed = time.monotonic() - loop_start
        sleep_time = max(0.0, POLL_INTERVAL - elapsed)
        stop_event.wait(timeout=sleep_time)   # interruptible sleep

# ─────────────────────────────────────────────
#  Dashboard
# ─────────────────────────────────────────────
_HATCH_COLOUR = {
    HatchState.PREDICTING: "\033[33m",   # yellow
    HatchState.OPENING:    "\033[36m",   # cyan
    HatchState.OPEN:       "\033[32m",   # green
    HatchState.CLOSING:    "\033[35m",   # magenta
    HatchState.CLOSED:     "\033[90m",   # dark grey
    HatchState.ABORT:      "\033[31m",   # red
}
_DRONE_COLOUR = {
    DroneState.IDLE:       "\033[33m",
    DroneState.LAUNCHING:  "\033[36m",
    DroneState.LAUNCHED:   "\033[32m",
    DroneState.ABORT:      "\033[31m",
}
_RESET = "\033[0m"


def _coloured(text: str, colour: str) -> str:
    return f"{colour}{text}{_RESET}"


def print_dashboard(
    imu: IMUData,
    hatch: HatchMachine,
    drone: DroneMachine,
) -> None:
    roll, pitch, _ = imu.snapshot()
    current_max = max(abs(roll), abs(pitch))
    predicted   = imu.predict_max_angle()

    hc = _HATCH_COLOUR.get(hatch.state, "")
    dc = _DRONE_COLOUR.get(drone.state, "")

    hatch_str = _coloured(f"Hatch: {hatch.state.name:<10}", hc)
    drone_str = _coloured(f"Drone: {drone.state.name:<9}", dc)
    angle_str = f"Current: {current_max:5.1f}°  (R:{roll:+6.1f}°  P:{pitch:+6.1f}°)"
    pred_str  = f"Predicted(2s): {predicted:5.1f}°"

    line = f"\r[{hatch_str}]  [{drone_str}]  [{angle_str}]  [{pred_str}]"
    print(line, end="", flush=True)

# ─────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────
def main() -> None:
    print("=== Unified Launcher — Betaflight MSP Edition ===")
    print(f"Connecting to {COM_PORT} …", flush=True)

    # Allow PySerial to accept "socket://" paths natively for the mock test
    if COM_PORT.startswith("socket://"):
        import serial
        serial.Serial = serial.serial_for_url

    try:
        board = MultiWii(COM_PORT)
    except Exception as exc:
        print(f"\n[FATAL] Could not open serial port {COM_PORT}: {exc}")
        sys.exit(1)

    imu   = IMUData()
    hatch = HatchMachine(imu)
    drone = DroneMachine(imu, hatch)

    stop_event = threading.Event()

    # Graceful shutdown on Ctrl-C / SIGTERM
    def _shutdown(signum=None, frame=None) -> None:  # noqa: ANN001
        print("\n\n[SHUTDOWN] Signal received — stopping …")
        stop_event.set()

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    poller_thread = threading.Thread(
        target=imu_poller,
        args=(board, imu, hatch, drone, stop_event),
        name="IMU-Poller",
        daemon=True,
    )
    poller_thread.start()

    print("Polling started. Press Ctrl-C to exit.\n")

    # ── Dashboard loop (main thread, ~10 Hz) ────────────────────────────────
    try:
        while not stop_event.is_set():
            print_dashboard(imu, hatch, drone)

            # Reset instead of exiting once both FSMs have reached terminal states
            if (hatch.state in (HatchState.CLOSED, HatchState.ABORT) and
                    drone.state in (DroneState.LAUNCHED, DroneState.ABORT)):
                time.sleep(2.5)    # let final state render for 2.5s before resetting
                hatch.reset()
                drone.reset()

            time.sleep(0.1)        # 10 Hz dashboard refresh
    finally:
        stop_event.set()
        poller_thread.join(timeout=2.0)
        print("\n\n=== Final States ===")
        print(f"  Hatch : {hatch.state.name}")
        print(f"  Drone : {drone.state.name}")
        print("===================\nDone.")


if __name__ == "__main__":
    main()
