from MultiWii import MultiWii
import struct
import time
import sys

COM_PORT = "COM7"

# ─── ADJUST THESE ───────────────────────────────────────────────
MOTOR_VALUE    = 1600   # Start low. Increase by 50 if no spin: 1100→1150→1200→1300
SPIN_DURATION  = 5.0    # How many seconds to keep motors spinning
MOTOR_INDEX    = 1-3   # None = ALL motors, or 0-3 for a single motor
# ────────────────────────────────────────────────────────────────

MOTOR_OFF = 1000

board = MultiWii(COM_PORT)

def send_raw(msg_id, direction, packed_data=b''):
    """Send any MSP command with raw packed data."""
    size = len(packed_data)
    crc = size ^ msg_id
    for b in packed_data:
        crc ^= b
    # MUST always be '<' for host→FC. Betaflight ignores '>' (treats as reply).
    d = b'<'
    header = struct.pack('<3c2B', b'$', b'M', d, size, msg_id)
    board.com.write(header + packed_data + struct.pack('<B', crc))

def send_motor(values):
    """Set motor PWM values. `values` is a list of 8 PWM ints."""
    packed = struct.pack('<8H', *values)
    send_raw(214, '>', packed)  # MSP_SET_MOTOR

def read_motor():
    """Request current motor values from FC (MSP_MOTOR = 104)."""
    send_raw(104, '<')
    time.sleep(0.05)
    if board.com.in_waiting:
        raw = board.com.read(board.com.in_waiting)
        return raw
    return None

def build_motor_values(pwm):
    """Build list of 8 motor values based on MOTOR_INDEX setting."""
    if MOTOR_INDEX is not None:
        vals = [MOTOR_OFF] * 8
        vals[MOTOR_INDEX] = pwm
        return vals
    return [pwm] * 8

def stop_motors():
    """Send stop command many times to ensure motors are off."""
    print("Stopping motors...")
    for _ in range(50):
        send_motor([MOTOR_OFF] * 8)
        time.sleep(0.01)
    print("Motors OFF.")

# ─── MAIN ───────────────────────────────────────────────────────
print("=" * 50)
print("  MOTOR SPIN TEST")
print("=" * 50)
print(f"  COM Port:    {COM_PORT}")
print(f"  PWM Value:   {MOTOR_VALUE}")
print(f"  Duration:    {SPIN_DURATION}s")
motor_label = f"Motor {MOTOR_INDEX}" if MOTOR_INDEX is not None else "ALL motors"
print(f"  Target:      {motor_label}")
print("=" * 50)
print()
print("!! REMOVE ALL PROPELLERS !!")
print("!! Press Ctrl+C at ANY time to kill motors !!")
print()

# Step 1: Warm up MSP — request motor/status data to wake up the FC
print("[1/3] Waking up FC via MSP...")
for _ in range(10):
    send_raw(101, '<')   # MSP_STATUS
    time.sleep(0.02)
    send_raw(104, '<')   # MSP_MOTOR
    time.sleep(0.02)
board.com.flushInput()
print("      FC connection active.")

# Step 2: Pre-spin ramp — some ESCs need a brief low signal before responding
print("[2/3] Sending initial motor signals (ESC wake-up)...")
for _ in range(100):  # 1 second of MOTOR_OFF to let ESCs initialize
    send_motor([MOTOR_OFF] * 8)
    time.sleep(0.01)
board.com.flushInput()
print("      ESCs initialized.")

# Step 3: Countdown
print(f"[3/3] Starting in 3 seconds... (Ctrl+C to abort)")
for i in range(3, 0, -1):
    print(f"      {i}...")
    time.sleep(1)

# ─── SPIN ───────────────────────────────────────────────────────
motor_vals = build_motor_values(MOTOR_VALUE)
print(f"\n>>> SPINNING at PWM {MOTOR_VALUE} for {SPIN_DURATION}s <<<")
start = time.time()
count = 0
try:
    while time.time() - start < SPIN_DURATION:
        send_motor(motor_vals)
        count += 1
        elapsed = time.time() - start
        # Print status every second
        if count % 100 == 0:
            print(f"    Running... {elapsed:.1f}s / {SPIN_DURATION}s  ({count} packets sent)")
        time.sleep(0.01)  # 100 Hz — matches Betaflight Configurator rate
except KeyboardInterrupt:
    print("\n!! ABORTED by user !!")

# ─── STOP ───────────────────────────────────────────────────────
stop_motors()
elapsed = time.time() - start
print(f"\nDone. Ran for {elapsed:.1f}s, sent {count} MSP_SET_MOTOR packets.")

if count > 0 and MOTOR_VALUE >= 1050:
    print("\nIf motors did NOT spin, try:")
    print(f"  1. Increase MOTOR_VALUE (currently {MOTOR_VALUE}) — try {MOTOR_VALUE + 100}")
    print("  2. In Betaflight CLI, run:  set motor_pwm_protocol = STANDARD")
    print("  3. In Betaflight CLI, run:  set motor_pwm_rate = 400")
    print("  4. Check that ESCs are calibrated and powered (battery plugged in)")
    print("  5. Try individual motors: set MOTOR_INDEX = 0 (then 1, 2, 3)")
