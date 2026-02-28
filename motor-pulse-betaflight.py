"""
motor-pulse-betaflight.py
─────────────────────────
Betaflight motor test via MSP (MultiWii Serial Protocol).

This script implements MSP from scratch using pyserial — no external
MSP library needed. It replicates what the Betaflight Configurator's
Motors tab does:
  1. Query FC status & configuration
  2. Check arming flags (motors won't spin if armed)
  3. Detect motor protocol (DSHOT vs PWM — affects value range)
  4. Send MSP_SET_MOTOR continuously at ~50 Hz

!! REMOVE ALL PROPELLERS BEFORE RUNNING !!
"""

from enum import auto

import serial
import struct
import time
import sys

# ─── USER CONFIG ────────────────────────────────────────────────
COM_PORT       = "COM7"
BAUD_RATE      = 115200

# For DSHOT protocol: values are 0 (off) to 2047 (full), ~100-300 = gentle spin
# For PWM/Oneshot:    values are 1000 (off) to 2000 (full)
# The script auto-detects protocol and adjusts, but you can override:
MOTOR_VALUE    = 1100  # None = auto (low safe value), or set manually (min throttle = 1070)
SPIN_DURATION  = 2.5      # seconds
MOTOR_INDEX    = None     # None = ALL motors, or 0/1/2/3 for one motor
# ────────────────────────────────────────────────────────────────

# MSP Command IDs
MSP_API_VERSION    = 1
MSP_FC_VARIANT     = 2
MSP_FC_VERSION     = 3
MSP_STATUS         = 101
MSP_RAW_IMU        = 102
MSP_MOTOR          = 104
MSP_RC             = 105
MSP_ATTITUDE       = 108
MSP_ANALOG         = 110
MSP_SET_MOTOR      = 214
MSP_ADVANCED_CONFIG = 90
MSP_BOARD_INFO      = 4

# Motor protocol types (from Betaflight)
MOTOR_PROTOCOLS = {
    0: "PWM",
    1: "ONESHOT125",
    2: "ONESHOT42",
    3: "MULTISHOT",
    4: "BRUSHED",
    5: "DSHOT150",
    6: "DSHOT300",
    7: "DSHOT600",
    8: "PROSHOT1000",
    9: "DISABLED",
}


class MSPConnection:
    """Clean MSP v1 implementation for Betaflight communication."""

    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(0.1)
        self.ser.flushInput()

    def close(self):
        self.ser.close()

    def send_command(self, cmd_id, data=b''):
        """Send an MSP command. Always use '<' — Betaflight treats '>' as reply (ignored)."""
        direction = b'<'  # MUST be '<' for ALL host→FC messages
        size = len(data)
        # CRC = XOR of size, cmd_id, and all data bytes
        crc = size ^ cmd_id
        for b in data:
            crc ^= b
        header = b'$M' + direction + bytes([size, cmd_id])
        packet = header + data + bytes([crc])
        self.ser.write(packet)

    def receive_response(self, timeout=1.0):
        """Read an MSP response. Returns (cmd_id, data_bytes) or None on timeout."""
        deadline = time.time() + timeout
        # Scan for '$M' header
        while time.time() < deadline:
            b = self.ser.read(1)
            if b == b'$':
                b2 = self.ser.read(1)
                if b2 == b'M':
                    break
        else:
            return None

        direction = self.ser.read(1)  # '>' response, '!' error
        if not direction:
            return None

        size_byte = self.ser.read(1)
        if not size_byte:
            return None
        size = size_byte[0]

        cmd_byte = self.ser.read(1)
        if not cmd_byte:
            return None
        cmd_id = cmd_byte[0]

        data = self.ser.read(size) if size > 0 else b''
        crc_byte = self.ser.read(1)

        if direction == b'!':
            return None  # FC reported error for this command

        return (cmd_id, data)

    def request(self, cmd_id, data=b'', timeout=1.0):
        """Send a command and wait for the response."""
        self.ser.flushInput()
        self.send_command(cmd_id, data)
        return self.receive_response(timeout)

    def send_only(self, cmd_id, data=b''):
        """Send a command without waiting for response (for SET commands)."""
        self.send_command(cmd_id, data)


def get_fc_info(msp):
    """Query basic flight controller information."""
    info = {}

    # FC Variant (e.g., "BTFL" for Betaflight)
    resp = msp.request(MSP_FC_VARIANT)
    if resp:
        _, data = resp
        info['variant'] = data[:4].decode('ascii', errors='replace')

    # FC Version
    resp = msp.request(MSP_FC_VERSION)
    if resp:
        _, data = resp
        if len(data) >= 3:
            info['version'] = f"{data[0]}.{data[1]}.{data[2]}"

    # API Version
    resp = msp.request(MSP_API_VERSION)
    if resp:
        _, data = resp
        if len(data) >= 3:
            info['api'] = f"{data[1]}.{data[2]}"

    # Board Info
    resp = msp.request(MSP_BOARD_INFO)
    if resp:
        _, data = resp
        if len(data) >= 4:
            info['board'] = data[:4].decode('ascii', errors='replace')

    return info


def get_status(msp):
    """Query FC status including arming flags."""
    resp = msp.request(MSP_STATUS)
    if not resp:
        return None

    _, data = resp
    if len(data) < 11:
        return None

    status = {}
    status['cycle_time'] = struct.unpack('<H', data[0:2])[0]
    status['i2c_errors'] = struct.unpack('<H', data[2:4])[0]
    status['sensors'] = struct.unpack('<H', data[4:6])[0]
    status['flight_mode'] = struct.unpack('<I', data[6:10])[0]
    status['profile'] = data[10]

    # Extended status fields (if available)
    if len(data) >= 15:
        status['system_load'] = struct.unpack('<H', data[11:13])[0]

    # Arming disable flags (varies by API version)
    if len(data) >= 20:
        idx = 13
        if len(data) > idx + 1:
            status['pid_profile_count'] = data[idx]
            idx += 1
        if len(data) > idx + 1:
            status['rate_profile'] = data[idx]
            idx += 1
        # Additional flag bytes
        if len(data) > idx:
            flag_byte_count = data[idx] & 0x0F
            idx += 1
            idx += flag_byte_count  # skip extended flight mode flags
        if len(data) > idx:
            status['arming_disable_count'] = data[idx]
            idx += 1
        if len(data) > idx + 3:
            status['arming_disable_flags'] = struct.unpack('<I', data[idx:idx+4])[0]

    return status


def get_motor_protocol(msp):
    """Query motor protocol from MSP_ADVANCED_CONFIG."""
    resp = msp.request(MSP_ADVANCED_CONFIG)
    if not resp:
        return None, None

    _, data = resp
    if len(data) < 5:
        return None, None

    # Byte layout: gyro_sync_denom, pid_process_denom, use_continuous_update,
    #              motor_protocol, motor_pwm_rate(u16), motor_idle(u16), ...
    motor_protocol = data[3]
    motor_pwm_rate = struct.unpack('<H', data[4:6])[0] if len(data) >= 6 else 0
    motor_idle = struct.unpack('<H', data[6:8])[0] if len(data) >= 8 else 0

    return motor_protocol, {
        'pwm_rate': motor_pwm_rate,
        'idle_value': motor_idle,
    }


def get_motor_values(msp):
    """Read current motor output values."""
    resp = msp.request(MSP_MOTOR)
    if not resp:
        return None
    _, data = resp
    if len(data) < 16:
        return None
    return list(struct.unpack('<8H', data[:16]))


def get_battery(msp):
    """Read basic battery info."""
    resp = msp.request(MSP_ANALOG)
    if not resp:
        return None
    _, data = resp
    if len(data) < 3:
        return None
    return {
        'vbat': data[0] / 10.0,  # 0.1V steps
        'mah': struct.unpack('<H', data[1:3])[0],
    }


def is_dshot(protocol_id):
    """Check if motor protocol is DSHOT."""
    return protocol_id in (5, 6, 7, 8)  # DSHOT150, 300, 600, PROSHOT1000


def get_safe_motor_value(protocol_id):
    """Return a safe low motor test value based on protocol."""
    if is_dshot(protocol_id):
        # DSHOT: 0=off, 48=minimum, values 1-47 are reserved for commands
        # For testing: ~100-200 is a gentle spin
        return 1100  # Betaflight maps this internally for DSHOT
    else:
        # PWM/Oneshot: 1000=off, 1050-1100=minimum spin
        return 1075


# Arming disable flag names (from Betaflight source)
ARMING_DISABLE_FLAGS = [
    "NO_GYRO",
    "FAILSAFE",
    "RX_FAILSAFE",
    "BAD_RX_RECOVERY",
    "BOXFAILSAFE",
    "RUNAWAY_TAKEOFF",
    "CRASH_DETECTED",
    "THROTTLE",
    "ANGLE",
    "BOOT_GRACE_TIME",
    "NOPREARM",
    "LOAD",
    "CALIBRATING",
    "CLI",
    "CMS_MENU",
    "BST",
    "MSP",
    "PARALYZE",
    "GPS",
    "RESC",
    "RPMFILTER",
    "REBOOT_REQUIRED",
    "DSHOT_BITBANG",
    "ACC_CALIBRATION",
    "MOTOR_PROTOCOL",
    "ARM_SWITCH",
]


def decode_arming_flags(flags_int):
    """Decode arming disable flags bitmask into human-readable list."""
    active = []
    for i, name in enumerate(ARMING_DISABLE_FLAGS):
        if flags_int & (1 << i):
            active.append(name)
    return active


def set_motors(msp, values):
    """Send MSP_SET_MOTOR with 8 motor values."""
    data = struct.pack('<8H', *values)
    msp.send_only(MSP_SET_MOTOR, data)


def build_motor_values(pwm, motor_index=None, motor_off=1000):
    """Build list of 8 motor values."""
    if motor_index is not None:
        vals = [motor_off] * 8
        vals[motor_index] = pwm
        return vals
    return [pwm] * 8


# ═══════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════
def main():
    print("=" * 55)
    print("  BETAFLIGHT MOTOR TEST — MSP Direct")
    print("=" * 55)
    print(f"  Port: {COM_PORT} @ {BAUD_RATE} baud")
    print()

    # ── Connect ──────────────────────────────────────────────
    print("[1/5] Connecting to FC...")
    try:
        msp = MSPConnection(COM_PORT, BAUD_RATE)
    except serial.SerialException as e:
        print(f"  ERROR: Cannot open {COM_PORT}: {e}")
        print("  → Is the FC plugged in via USB?")
        print("  → Is Betaflight Configurator closed? (it locks the port)")
        return

    # ── Identify FC ──────────────────────────────────────────
    print("[2/5] Querying flight controller...")
    info = get_fc_info(msp)
    if not info:
        print("  ERROR: No response from FC. Check:")
        print("  → USB cable (try a different one — some are charge-only)")
        print("  → COM port number (check Device Manager)")
        msp.close()
        return

    variant = info.get('variant', '????')
    version = info.get('version', '?.?.?')
    board = info.get('board', '????')
    api = info.get('api', '?.?')
    print(f"  Firmware:  {variant} {version}")
    print(f"  Board:     {board}")
    print(f"  API:       {api}")

    if variant != 'BTFL':
        print(f"  WARNING: Expected Betaflight (BTFL), got '{variant}'")
        print("  → Motor testing may work differently")

    # ── Motor Protocol ───────────────────────────────────────
    print("[3/5] Checking motor configuration...")
    protocol_id, motor_config = get_motor_protocol(msp)
    if protocol_id is not None:
        protocol_name = MOTOR_PROTOCOLS.get(protocol_id, f"UNKNOWN({protocol_id})")
        print(f"  Protocol:  {protocol_name}")
        if motor_config:
            print(f"  PWM Rate:  {motor_config['pwm_rate']} Hz")
            print(f"  Idle:      {motor_config['idle_value']}")

        if protocol_id == 9:  # DISABLED
            print("\n  !! MOTOR PROTOCOL IS DISABLED !!")
            print("  → In Betaflight CLI, set a motor protocol:")
            print("     set motor_pwm_protocol = DSHOT300")
            print("     save")
            msp.close()
            return
    else:
        protocol_name = "UNKNOWN"
        print("  Could not read motor config (older firmware?)")

    # Determine motor value
    dshot = is_dshot(protocol_id) if protocol_id is not None else False
    if MOTOR_VALUE is not None:
        motor_val = MOTOR_VALUE
    else:
        motor_val = get_safe_motor_value(protocol_id if protocol_id else 0)

    motor_off = 0 if dshot else 1000

    # ── Status & Arming Check ────────────────────────────────
    print("[4/5] Checking FC status...")
    status = get_status(msp)
    if status:
        print(f"  Cycle time:  {status.get('cycle_time', '?')} µs")
        load = status.get('system_load', None)
        if load is not None:
            print(f"  CPU load:    {load / 10.0:.1f}%")

        arming_flags = status.get('arming_disable_flags')
        if arming_flags is not None:
            if arming_flags == 0:
                print("  Arming:     READY (no disable flags)")
            else:
                flags = decode_arming_flags(arming_flags)
                print(f"  Arming disabled by: {', '.join(flags)}")
                # This is fine — we WANT it disarmed for motor testing
                print("  (This is OK — motor testing works when DISARMED)")
    else:
        print("  Could not read status")

    # Read current motor values
    motors = get_motor_values(msp)
    if motors:
        print(f"  Current motors: {motors[:4]}")

    # Battery check
    battery = get_battery(msp)
    if battery:
        print(f"  Battery:   {battery['vbat']:.1f}V, {battery['mah']} mAh used")
        if battery['vbat'] < 1.0:
            print("\n  !! NO BATTERY DETECTED !!")
            print("  → Motors need a LiPo battery to spin")
            print("  → USB alone only powers the FC, not the ESCs/motors")
            print("  → Plug in your battery and try again")
            msp.close()
            return

    # ── Motor Test ───────────────────────────────────────────
    print()
    print("=" * 55)
    motor_label = f"Motor {MOTOR_INDEX}" if MOTOR_INDEX is not None else "ALL motors"
    print(f"  Target:    {motor_label}")
    print(f"  Value:     {motor_val} ({'DSHOT range' if dshot else 'PWM µs'})")
    print(f"  Duration:  {SPIN_DURATION}s")
    print(f"  Protocol:  {protocol_name}")
    print("=" * 55)
    print()
    print("!! REMOVE ALL PROPELLERS !!")
    print("!! Press Ctrl+C at ANY TIME to kill motors !!")
    print()

    # Countdown
    print("Starting in 3 seconds...")
    for i in range(3, 0, -1):
        print(f"  {i}...")
        time.sleep(1)

    # ── Spin ─────────────────────────────────────────────────
    motor_vals = build_motor_values(motor_val, MOTOR_INDEX, motor_off)
    stop_vals = [motor_off] * 8

    print(f"\n>>> SPINNING: {motor_vals[:4]} <<<")
    start = time.time()
    count = 0
    try:
        while time.time() - start < SPIN_DURATION:
            set_motors(msp, motor_vals)
            count += 1
            elapsed = time.time() - start

            # Flush stale responses periodically
            if count % 50 == 0:
                if msp.ser.in_waiting > 100:
                    msp.ser.flushInput()

            # Status print every second
            if count % 50 == 0:
                print(f"  {elapsed:.1f}s / {SPIN_DURATION}s — {count} packets sent")

            time.sleep(0.02)  # 50 Hz, matches Betaflight Configurator

    except KeyboardInterrupt:
        print("\n!! ABORTED !!")

    # ── Stop ─────────────────────────────────────────────────
    print("Stopping motors...")
    try:
        for _ in range(100):
            set_motors(msp, stop_vals)
            time.sleep(0.01)
    except Exception as e:
        print(f"  (Serial error during stop: {e})")
        print("  Motors will stop automatically when no commands are sent.")

    elapsed = time.time() - start
    print(f"Motors OFF. Ran {elapsed:.1f}s, sent {count} packets.")

    # ── Post-test diagnostics ────────────────────────────────
    # Read motor values after test to confirm they returned to zero
    try:
        time.sleep(0.2)
        msp.ser.flushInput()
        motors_after = get_motor_values(msp)
        if motors_after:
            print(f"Motor values now: {motors_after[:4]}")
    except Exception:
        pass

    try:
        msp.close()
    except Exception:
        pass

    print()
    if count > 0:
        print("If motors did NOT spin, check these (in order):")
        print()
        print("1. BATTERY: Is a LiPo plugged in? USB only powers the FC,")
        print("   not the ESCs. Motors CANNOT spin without battery power.")
        print()
        print("2. BETAFLIGHT CONFIGURATOR: Open the Motors tab and try the")
        print("   sliders there first. If they don't work either, it's a")
        print("   hardware/config issue, not a script issue.")
        print()
        print("3. MOTOR PROTOCOL: In Betaflight CLI, check:")
        print("     get motor_pwm_protocol")
        print("   Common values: DSHOT300, DSHOT600, ONESHOT125, STANDARD")
        print("   If DISABLED → set motor_pwm_protocol = DSHOT300")
        print()
        print("4. ESC CALIBRATION: If using PWM protocol, ESCs may need")
        print("   calibration. Use Betaflight Configurator for this.")
        print()
        print(f"5. MOTOR VALUE: Currently {motor_val}. Try increasing:")
        if dshot:
            print("   For DSHOT: try 1150, 1200, 1300")
        else:
            print("   For PWM: try 1100, 1200, 1300, 1500")
        print()
        print("6. INDIVIDUAL MOTORS: Set MOTOR_INDEX = 0 (then 1, 2, 3)")
        print("   to test one at a time.")


if __name__ == "__main__":
    main()
