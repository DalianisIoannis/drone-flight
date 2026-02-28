from machine import Pin, PWM
import network
import socket
import time

# =========================
# Wi-Fi settings
# =========================
WIFI_SSID = "iPhone 13 pro"
WIFI_PASSWORD = "tasos123"

# =========================
# Servo settings
# =========================
SERVO_PIN = 15
SERVO_FREQ = 50

# Continuous-rotation behavior:
# NEUTRAL_US = stop
# lower pulse = one direction
# higher pulse = opposite direction
NEUTRAL_US = 1500

# Tune direction strength
OPEN_DELTA_US = 250
CLOSE_DELTA_US = 250

# Tune how long it runs to reach your "open" / "close"
OPEN_TIME_MS = 560
CLOSE_TIME_MS = 570

servo = PWM(Pin(SERVO_PIN))
servo.freq(SERVO_FREQ)

def set_pulse_us(us):
    servo.duty_ns(int(us * 1000))

def stop_servo():
    set_pulse_us(NEUTRAL_US)

def spin_open():
    # If direction is wrong, swap this with spin_close()
    set_pulse_us(NEUTRAL_US + OPEN_DELTA_US)

def spin_close():
    set_pulse_us(NEUTRAL_US - CLOSE_DELTA_US)

# =========================
# Wi-Fi connect
# =========================
def connect_wifi(ssid, password, timeout_s=15):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if not wlan.isconnected():
        print("Connecting to Wi-Fi...")
        wlan.connect(ssid, password)

        start = time.ticks_ms()
        while not wlan.isconnected():
            if time.ticks_diff(time.ticks_ms(), start) > timeout_s * 1000:
                raise RuntimeError("Wi-Fi connection timed out")
            time.sleep_ms(200)

    ip = wlan.ifconfig()[0]
    print("Wi-Fi connected")
    print("Pico IP:", ip)
    return wlan, ip

# =========================
# Web page
# =========================
def web_page(status_text, logical_state):
    return """<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>Pico Open / Close Control</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            text-align: center;
            padding: 24px;
        }
        h2 {
            margin-bottom: 10px;
        }
        p {
            font-size: 18px;
            margin: 8px 0;
        }
        .row {
            margin-top: 16px;
        }
        a {
            display: inline-block;
            margin: 8px;
            padding: 14px 20px;
            text-decoration: none;
            border: 1px solid #333;
            border-radius: 10px;
            color: #111;
            font-size: 17px;
            min-width: 120px;
        }
    </style>
</head>
<body>
    <h2>Pico Open / Close Control</h2>
    <p>Status: %s</p>
    <p>Logical state: %s</p>

    <div class="row">
        <a href="/open">Open</a>
        <a href="/stop">Stop / Center</a>
        <a href="/close">Close</a>
    </div>
</body>
</html>
""" % (status_text, logical_state)

# =========================
# Motion state
# =========================
logical_state = "unknown"   # "open", "closed", "stopped", "moving open", "moving close", "unknown"
status_text = "Ready"

motion_active = False
motion_kind = ""            # "open" or "close"
motion_end_ms = 0

def start_open_motion():
    global motion_active, motion_kind, motion_end_ms, logical_state, status_text
    spin_open()
    motion_active = True
    motion_kind = "open"
    motion_end_ms = time.ticks_add(time.ticks_ms(), OPEN_TIME_MS)
    logical_state = "moving open"
    status_text = "Opening..."

def start_close_motion():
    global motion_active, motion_kind, motion_end_ms, logical_state, status_text
    spin_close()
    motion_active = True
    motion_kind = "close"
    motion_end_ms = time.ticks_add(time.ticks_ms(), CLOSE_TIME_MS)
    logical_state = "moving close"
    status_text = "Closing..."

def stop_now():
    global motion_active, motion_kind, logical_state, status_text
    stop_servo()
    motion_active = False
    motion_kind = ""
    logical_state = "stopped"
    status_text = "Stopped"

# Start in stopped state
stop_now()

# =========================
# Start network + server
# =========================
_, ip = connect_wifi(WIFI_SSID, WIFI_PASSWORD)

addr = socket.getaddrinfo("0.0.0.0", 80)[0][-1]
server = socket.socket()
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(addr)
server.listen(2)
server.settimeout(0.05)

print("Open in browser: http://%s" % ip)

# =========================
# Main loop
# =========================
while True:
    # Non-blocking timed stop after open/close action
    if motion_active and time.ticks_diff(time.ticks_ms(), motion_end_ms) >= 0:
        stop_servo()
        motion_active = False

        if motion_kind == "open":
            logical_state = "open"
            status_text = "Opened"
            print("OPEN complete")
        elif motion_kind == "close":
            logical_state = "closed"
            status_text = "Closed"
            print("CLOSE complete")

        motion_kind = ""

    # Handle web requests
    try:
        client, client_addr = server.accept()
    except OSError:
        continue

    try:
        client.settimeout(1.0)

        request = client.recv(1024)
        if not request:
            client.close()
            continue

        first_line = request.split(b"\r\n", 1)[0]
        parts = first_line.split()

        path = "/"
        if len(parts) >= 2:
            try:
                path = parts[1].decode()
            except:
                path = "/"

        if path == "/open":
            start_open_motion()
            print("OPEN from", client_addr)

        elif path == "/close":
            start_close_motion()
            print("CLOSE from", client_addr)

        elif path == "/stop":
            stop_now()
            print("STOP from", client_addr)

        html = web_page(status_text, logical_state)

        response = (
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html\r\n"
            "Connection: close\r\n"
            "\r\n"
            + html
        )

        client.write(response)

    except OSError:
        pass

    except Exception as e:
        print("Request error:", e)

    finally:
        try:
            client.close()
        except:
            pass