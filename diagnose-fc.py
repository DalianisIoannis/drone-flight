"""
diagnose-fc.py — Full Betaflight FC diagnostic over MSP
Reads everything: status, arming flags, motor protocol, battery, motors, RC.
Does NOT try to spin motors — just reports the FC state.
"""
import serial
import struct
import time

COM_PORT  = "COM7"
BAUD_RATE = 115200

# MSP IDs
MSP_API_VERSION     = 1
MSP_FC_VARIANT      = 2
MSP_FC_VERSION      = 3
MSP_BOARD_INFO      = 4
MSP_STATUS          = 101
MSP_STATUS_EX       = 150
MSP_RAW_IMU         = 102
MSP_MOTOR           = 104
MSP_RC              = 105
MSP_ATTITUDE        = 108
MSP_ANALOG          = 110
MSP_ADVANCED_CONFIG = 90
MSP_MOTOR_CONFIG    = 131
MSP_FEATURE_CONFIG  = 36
MSP_BATTERY_STATE   = 130
MSP_SET_MOTOR       = 214

MOTOR_PROTOCOLS = {
    0: "PWM", 1: "ONESHOT125", 2: "ONESHOT42", 3: "MULTISHOT",
    4: "BRUSHED", 5: "DSHOT150", 6: "DSHOT300", 7: "DSHOT600",
    8: "PROSHOT1000", 9: "DISABLED",
}

ARMING_DISABLE_FLAGS = [
    "NO_GYRO", "FAILSAFE", "RX_FAILSAFE", "BAD_RX_RECOVERY",
    "BOXFAILSAFE", "RUNAWAY_TAKEOFF", "CRASH_DETECTED", "THROTTLE",
    "ANGLE", "BOOT_GRACE_TIME", "NOPREARM", "LOAD",
    "CALIBRATING", "CLI", "CMS_MENU", "BST",
    "MSP", "PARALYZE", "GPS", "RESC",
    "RPMFILTER", "REBOOT_REQUIRED", "DSHOT_BITBANG", "ACC_CALIBRATION",
    "MOTOR_PROTOCOL", "ARM_SWITCH",
]

FEATURES = [
    "RX_PPM", "unused1", "INFLIGHT_ACC_CAL", "RX_SERIAL",
    "MOTOR_STOP", "SERVO_TILT", "SOFTSERIAL", "GPS",
    "unused8", "SONAR", "TELEMETRY", "unused11",
    "3D", "RX_PARALLEL_PWM", "RX_MSP", "RSSI_ADC",
    "LED_STRIP", "DISPLAY", "OSD", "unused19",
    "CHANNEL_FORWARDING", "TRANSPONDER", "AIRMODE", "unused23",
    "unused24", "RX_SPI", "SOFTSERIAL", "ESC_SENSOR",
    "ANTI_GRAVITY", "DYNAMIC_FILTER",
]


def msp_send(ser, cmd_id, data=b''):
    # MUST always be '<' — Betaflight routes '>' to reply handler (ignores commands)
    direction = b'<'
    size = len(data)
    crc = size ^ cmd_id
    for b in data:
        crc ^= b
    packet = b'$M' + direction + bytes([size, cmd_id]) + data + bytes([crc])
    ser.write(packet)


def msp_recv(ser, timeout=1.5):
    deadline = time.time() + timeout
    while time.time() < deadline:
        b = ser.read(1)
        if b == b'$':
            b2 = ser.read(1)
            if b2 == b'M':
                break
    else:
        return None

    direction = ser.read(1)
    if not direction:
        return None

    size_b = ser.read(1)
    if not size_b:
        return None
    size = size_b[0]

    cmd_b = ser.read(1)
    if not cmd_b:
        return None
    cmd_id = cmd_b[0]

    data = ser.read(size) if size > 0 else b''
    ser.read(1)  # crc

    if direction == b'!':
        return ('error', cmd_id, data)
    return ('ok', cmd_id, data)


def msp_request(ser, cmd_id, data=b''):
    ser.flushInput()
    msp_send(ser, cmd_id, data)
    return msp_recv(ser)


def hex_dump(data, max_bytes=64):
    return ' '.join(f'{b:02X}' for b in data[:max_bytes])


def main():
    print("=" * 60)
    print("  BETAFLIGHT FC DIAGNOSTIC")
    print("=" * 60)
    print(f"  Port: {COM_PORT} @ {BAUD_RATE}")
    print()

    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        return

    time.sleep(0.2)
    ser.flushInput()

    # ── FC Identity ──────────────────────────────────────────
    print("── FC IDENTITY ──")
    resp = msp_request(ser, MSP_FC_VARIANT)
    if resp and resp[0] == 'ok':
        variant = resp[2][:4].decode('ascii', errors='replace')
        print(f"  Variant:    {variant}")
    else:
        print("  Variant:    NO RESPONSE")
        print("  → FC not responding. Check USB cable & COM port.")
        ser.close()
        return

    resp = msp_request(ser, MSP_FC_VERSION)
    if resp and resp[0] == 'ok':
        d = resp[2]
        if len(d) >= 3:
            print(f"  Version:    {d[0]}.{d[1]}.{d[2]}")
        # Check for version string (newer API)
        if len(d) > 3:
            vstr = d[3:].decode('ascii', errors='replace').strip('\x00')
            if vstr:
                print(f"  Version str: {vstr}")

    resp = msp_request(ser, MSP_API_VERSION)
    if resp and resp[0] == 'ok':
        d = resp[2]
        if len(d) >= 3:
            print(f"  API:        {d[0]}.{d[1]}.{d[2]}")

    resp = msp_request(ser, MSP_BOARD_INFO)
    if resp and resp[0] == 'ok':
        d = resp[2]
        board = d[:4].decode('ascii', errors='replace')
        print(f"  Board:      {board}")
        if len(d) > 4:
            print(f"  Board data: [{hex_dump(d)}]")

    # ── Status ───────────────────────────────────────────────
    print("\n── FC STATUS ──")
    resp = msp_request(ser, MSP_STATUS)
    if resp and resp[0] == 'ok':
        d = resp[2]
        print(f"  Raw data ({len(d)} bytes): [{hex_dump(d)}]")
        if len(d) >= 11:
            cycle = struct.unpack('<H', d[0:2])[0]
            i2c_err = struct.unpack('<H', d[2:4])[0]
            sensors = struct.unpack('<H', d[4:6])[0]
            flags = struct.unpack('<I', d[6:10])[0]
            profile = d[10]
            print(f"  Cycle time:    {cycle} µs")
            print(f"  I2C errors:    {i2c_err}")
            print(f"  Sensor mask:   0x{sensors:04X} (ACC={'Y' if sensors&1 else 'N'} BARO={'Y' if sensors&2 else 'N'} MAG={'Y' if sensors&4 else 'N'} GPS={'Y' if sensors&8 else 'N'} GYRO={'Y' if sensors&32 else 'N'})")
            print(f"  Flight mode:   0x{flags:08X}")
            print(f"  PID profile:   {profile}")

        if len(d) >= 13:
            load = struct.unpack('<H', d[11:13])[0]
            print(f"  CPU load:      {load / 10.0:.1f}%")

        # Parse arming disable flags (MSP_STATUS_EX format embedded in MSP_STATUS for newer BF)
        if len(d) >= 17:
            idx = 13
            pid_count = d[idx]; idx += 1
            rate_profile = d[idx]; idx += 1
            print(f"  PID profiles:  {pid_count}")
            print(f"  Rate profile:  {rate_profile}")

            if idx < len(d):
                flag_extra_bytes = d[idx] & 0x0F
                idx += 1
                # Skip extended flight mode flags
                idx += flag_extra_bytes

            if idx < len(d):
                arming_count = d[idx]; idx += 1
                print(f"  Arming flag count: {arming_count}")

            if idx + 4 <= len(d):
                arming_flags = struct.unpack('<I', d[idx:idx+4])[0]
                idx += 4
                print(f"  Arming flags:  0x{arming_flags:08X}")
                if arming_flags == 0:
                    print("  → ARMING READY (no disable flags set)")
                else:
                    active = []
                    for i, name in enumerate(ARMING_DISABLE_FLAGS):
                        if arming_flags & (1 << i):
                            active.append(f"    bit {i}: {name}")
                    print("  → ARMING DISABLED by:")
                    for f in active:
                        print(f)

            if idx < len(d):
                config_state = d[idx]; idx += 1
                print(f"  Config state:  0x{config_state:02X} {'(reboot required!)' if config_state & 1 else '(OK)'}")

    # ── Advanced Config (Motor Protocol) ─────────────────────
    print("\n── MOTOR PROTOCOL ──")
    resp = msp_request(ser, MSP_ADVANCED_CONFIG)
    if resp and resp[0] == 'ok':
        d = resp[2]
        print(f"  Raw data ({len(d)} bytes): [{hex_dump(d)}]")
        if len(d) >= 4:
            gyro_denom = d[0]
            pid_denom = d[1]
            continuous = d[2]
            protocol = d[3]
            proto_name = MOTOR_PROTOCOLS.get(protocol, f"UNKNOWN({protocol})")
            print(f"  Gyro denom:    {gyro_denom}")
            print(f"  PID denom:     {pid_denom}")
            print(f"  Continuous:    {continuous}")
            print(f"  Protocol:      {protocol} = {proto_name}")

            if protocol == 9:
                print("  !! MOTOR PROTOCOL IS DISABLED — motors will NOT work !!")
                print("  → Fix: Betaflight CLI → set motor_pwm_protocol = DSHOT300 → save")

        if len(d) >= 6:
            pwm_rate = struct.unpack('<H', d[4:6])[0]
            print(f"  PWM rate:      {pwm_rate} Hz")
        if len(d) >= 8:
            idle = struct.unpack('<H', d[6:8])[0]
            print(f"  Motor idle:    {idle}")
        if len(d) >= 9:
            use_32k = d[8]
            print(f"  Gyro 32kHz:    {use_32k}")
        if len(d) >= 10:
            inversion = d[9]
            print(f"  Motor invert:  {inversion}")

    # ── Motor Config ─────────────────────────────────────────
    print("\n── MOTOR CONFIG ──")
    resp = msp_request(ser, MSP_MOTOR_CONFIG)
    if resp and resp[0] == 'ok':
        d = resp[2]
        print(f"  Raw data ({len(d)} bytes): [{hex_dump(d)}]")
        if len(d) >= 6:
            min_thr = struct.unpack('<H', d[0:2])[0]
            max_thr = struct.unpack('<H', d[2:4])[0]
            min_cmd = struct.unpack('<H', d[4:6])[0]
            print(f"  Min throttle:  {min_thr}")
            print(f"  Max throttle:  {max_thr}")
            print(f"  Min command:   {min_cmd}")
        if len(d) >= 7:
            pole_count = d[6]
            print(f"  Motor poles:   {pole_count}")
    elif resp and resp[0] == 'error':
        print("  MSP_MOTOR_CONFIG not supported (older firmware)")
    else:
        print("  No response")

    # ── Feature Flags ────────────────────────────────────────
    print("\n── FEATURES ──")
    resp = msp_request(ser, MSP_FEATURE_CONFIG)
    if resp and resp[0] == 'ok':
        d = resp[2]
        if len(d) >= 4:
            feats = struct.unpack('<I', d[:4])[0]
            print(f"  Feature mask:  0x{feats:08X}")
            active = []
            for i, name in enumerate(FEATURES):
                if i < 32 and feats & (1 << i):
                    active.append(name)
            print(f"  Active:  {', '.join(active) if active else 'NONE'}")
            if 'MOTOR_STOP' in active:
                print("  → MOTOR_STOP is enabled (motors stop at zero throttle)")
            if 'AIRMODE' in active:
                print("  → AIRMODE is enabled")
            if '3D' in active:
                print("  → 3D mode is enabled (midpoint throttle)")

    # ── Battery ──────────────────────────────────────────────
    print("\n── BATTERY ──")
    resp = msp_request(ser, MSP_ANALOG)
    if resp and resp[0] == 'ok':
        d = resp[2]
        print(f"  Raw data ({len(d)} bytes): [{hex_dump(d)}]")
        if len(d) >= 7:
            vbat = d[0] / 10.0
            mah = struct.unpack('<H', d[1:3])[0]
            rssi = struct.unpack('<H', d[3:5])[0]
            amps = struct.unpack('<h', d[5:7])[0] / 100.0
            print(f"  Voltage:  {vbat:.1f}V")
            print(f"  mAh used: {mah}")
            print(f"  RSSI:     {rssi}")
            print(f"  Current:  {amps:.2f}A")
            if vbat < 1.0:
                print("  !! NO BATTERY — motors cannot spin without battery power !!")
                print("  !! USB only powers the FC board, not ESCs/motors !!")

    # Also try MSP_BATTERY_STATE for more detail
    resp = msp_request(ser, MSP_BATTERY_STATE)
    if resp and resp[0] == 'ok':
        d = resp[2]
        print(f"  Battery state ({len(d)} bytes): [{hex_dump(d)}]")
        if len(d) >= 8:
            cells = d[0]
            capacity = struct.unpack('<H', d[1:3])[0]
            vbat_legacy = d[3] / 10.0
            mah = struct.unpack('<H', d[4:6])[0]
            amps = struct.unpack('<h', d[6:8])[0] / 100.0
            print(f"  Cells:    {cells}S")
            print(f"  Capacity: {capacity} mAh")
            print(f"  Vbat:     {vbat_legacy:.1f}V (legacy)")
            if len(d) >= 10:
                state = d[8]
                vbat_fine = struct.unpack('<H', d[9:11])[0] / 100.0 if len(d) >= 11 else 0
                states = {0: "OK", 1: "WARNING", 2: "CRITICAL", 3: "NOT_PRESENT"}
                print(f"  State:    {states.get(state, f'UNKNOWN({state})')}")
                if vbat_fine > 0:
                    print(f"  Vbat fine: {vbat_fine:.2f}V")

    # ── Current Motor Values ─────────────────────────────────
    print("\n── MOTOR VALUES (current output) ──")
    resp = msp_request(ser, MSP_MOTOR)
    if resp and resp[0] == 'ok':
        d = resp[2]
        if len(d) >= 16:
            motors = struct.unpack('<8H', d[:16])
            for i, v in enumerate(motors[:4]):
                label = "OFF" if v <= 1000 else f"RUNNING ({v})"
                print(f"  Motor {i}: {v}  [{label}]")
            # Check if motors beyond 4 have values
            for i in range(4, 8):
                if motors[i] > 0:
                    print(f"  Motor {i}: {motors[i]}")

    # ── RC Channels ──────────────────────────────────────────
    print("\n── RC CHANNELS ──")
    resp = msp_request(ser, MSP_RC)
    if resp and resp[0] == 'ok':
        d = resp[2]
        n_channels = len(d) // 2
        if n_channels > 0:
            channels = struct.unpack(f'<{n_channels}H', d)
            names = ['Roll', 'Pitch', 'Yaw', 'Throttle'] + [f'AUX{i}' for i in range(1, n_channels - 3)]
            for i, val in enumerate(channels[:min(8, n_channels)]):
                name = names[i] if i < len(names) else f'CH{i}'
                print(f"  {name:10s}: {val}")
            if n_channels > 8:
                print(f"  ... {n_channels} channels total")
    else:
        print("  No RC data (receiver not connected?)")

    # ── Attitude ─────────────────────────────────────────────
    print("\n── ATTITUDE ──")
    resp = msp_request(ser, MSP_ATTITUDE)
    if resp and resp[0] == 'ok':
        d = resp[2]
        if len(d) >= 6:
            roll = struct.unpack('<h', d[0:2])[0] / 10.0
            pitch = struct.unpack('<h', d[2:4])[0] / 10.0
            heading = struct.unpack('<h', d[4:6])[0]
            print(f"  Roll:    {roll:.1f}°")
            print(f"  Pitch:   {pitch:.1f}°")
            print(f"  Heading: {heading}°")

    # ── Quick Motor Test (single packet) ─────────────────────
    print("\n── QUICK MOTOR TEST (single MSP_SET_MOTOR packet) ──")
    print("  Sending motors = [1050, 1050, 1050, 1050, 0, 0, 0, 0]...")
    test_data = struct.pack('<8H', 1050, 1050, 1050, 1050, 0, 0, 0, 0)
    msp_send(ser, MSP_SET_MOTOR, test_data)
    time.sleep(0.1)
    # Read back motor values to see if FC accepted it
    resp = msp_request(ser, MSP_MOTOR)
    if resp and resp[0] == 'ok':
        d = resp[2]
        if len(d) >= 16:
            motors = struct.unpack('<8H', d[:16])
            print(f"  Motors after SET: {list(motors[:4])}")
            if motors[0] == 1050:
                print("  → FC ACCEPTED the motor command!")
                print("  → If motors still don't physically spin, the issue is:")
                print("     - No battery (ESCs have no power)")
                print("     - ESC issue (not calibrated, not connected)")
                print("     - Motor wiring issue")
            elif motors[0] == 0 or motors[0] == 1000:
                print("  → FC REJECTED the motor command (value didn't change)")
                print("  → Possible reasons:")
                print("     - FC is ARMED (must be disarmed for motor test)")
                print("     - Motor protocol is DISABLED")
                print("     - Firmware doesn't allow MSP motor override")
    else:
        print("  Could not read back motor values")

    # Stop motors
    stop_data = struct.pack('<8H', 1000, 1000, 1000, 1000, 0, 0, 0, 0)
    msp_send(ser, MSP_SET_MOTOR, stop_data)

    ser.close()

    print()
    print("=" * 60)
    print("  DIAGNOSTIC COMPLETE")
    print("=" * 60)


if __name__ == "__main__":
    main()