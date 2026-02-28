from MultiWii import MultiWii
import time
import struct
import threading
import msvcrt  # Windows keyboard input (non-blocking)

# MSP command definitions
MSP_ATTITUDE  = {'message_id': 108, 'direction': '<'}  # Request attitude
MSP_RAW_IMU   = {'message_id': 102, 'direction': '<'}  # Request raw IMU
MSP_ALTITUDE  = {'message_id': 109, 'direction': '<'}  # Request baro altitude
MSP_SET_MOTOR = {'message_id': 214, 'direction': '>'}  # Set motor values

# Motor values (PWM microseconds): 1000 = off, ~1050 = minimum spin
MOTOR_OFF = 1000
MOTOR_LOW = 1050    # Very gentle spin — adjust if needed
PULSE_DURATION = 1  # seconds — how long the motors spin

# Replace with your actual COM port
board = MultiWii("COM3")
lock = threading.Lock()  # Protect serial access between threads

def send_motor_command(value):
    """Send MSP_SET_MOTOR manually to work around library bug."""
    motor_values = [value] * 8
    datatype = '8H'
    msg_id = 214  # MSP_SET_MOTOR
    packed_data = struct.pack('<' + datatype, *motor_values)
    size = len(packed_data)
    # Calculate CRC
    crc = size ^ msg_id
    for b in packed_data:
        crc ^= b
    # Build and send MSP packet: $M< <size> <id> <data> <crc>
    header = struct.pack('<3c2B', b'$', b'M', b'<', size, msg_id)
    board.com.write(header + packed_data + struct.pack('<B', crc))

def motor_pulse():
    """Briefly spin motors for PULSE_DURATION seconds, then stop."""
    with lock:
        print(f"\n>>> MOTOR PULSE: {MOTOR_LOW} for {PULSE_DURATION}s")
        send_motor_command(MOTOR_LOW)
    time.sleep(PULSE_DURATION)
    with lock:
        send_motor_command(MOTOR_OFF)
        print(">>> Motors OFF\n")

def attitude_loop(stop_event):
    """Background thread: continuously read and print attitude + altitude data."""
    while not stop_event.is_set():
        try:
            # ── Attitude ────────────────────────────────
            with lock:
                board.send(MSP_ATTITUDE)
                att_data = board.receive()

            roll    = att_data[0] / 10.0 if len(att_data) >= 3 else 0.0
            pitch   = att_data[1] / 10.0 if len(att_data) >= 3 else 0.0
            heading = att_data[2]         if len(att_data) >= 3 else 0

            # ── Altitude (barometer) ────────────────────
            alt_m   = 0.0
            vario   = 0.0
            try:
                with lock:
                    board.send(MSP_ALTITUDE)
                    alt_raw = board.receive()
                # MultiWii.receive() unpacks all payload bytes as int16 pairs.
                # MSP_ALTITUDE layout: int32 alt_cm, int16 vario_cm_s
                # → returned as (low_word, high_word, vario)
                if len(alt_raw) >= 2:
                    alt_cm = struct.unpack('<i', struct.pack('<hh', alt_raw[0], alt_raw[1]))[0]
                    alt_m  = alt_cm / 100.0
                if len(alt_raw) >= 3:
                    vario  = alt_raw[2] / 100.0   # already int16 cm/s
            except Exception:
                pass   # No baro fitted — altitude stays 0.0

            print(
                f"\rRoll: {roll:7.1f}°  Pitch: {pitch:7.1f}°  "
                f"Hdg: {heading:4}°  "
                f"Alt: {alt_m:6.2f}m  Vario: {vario:+.2f}m/s   ",
                end="", flush=True
            )
            time.sleep(0.1)
        except Exception as e:
            print(f"\nRead error: {e}")
            time.sleep(1)

def main():
    print("=== Betaflight Live Data ===")
    print("Press 'R' to pulse motors | Ctrl+C to quit")
    print("!! REMOVE PROPELLERS BEFORE TESTING !!\n")

    stop_event = threading.Event()
    reader = threading.Thread(target=attitude_loop, args=(stop_event,), daemon=True)
    reader.start()

    try:
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8', errors='ignore').upper()
                if key == 'R':
                    motor_pulse()
            time.sleep(0.05)
    except KeyboardInterrupt:
        stop_event.set()
        with lock:
            send_motor_command(MOTOR_OFF)
        print("\nStopped. Motors OFF.")

if __name__ == "__main__":
    main()