"""
mock_drone.py
=============
A software-in-the-loop bridge for testing `unified_launcher.py`
without a real Betaflight flight controller AND without any COM drivers!

This uses PySerial's built-in TCP socket support (RFC 2217 / raw).
It acts as a local TCP server on port 5555. 

How to use:
1. Run this script:   python mock_drone.py
2. Open unified_launcher.py and set COM_PORT = "socket://127.0.0.1:5555"
3. Run launcher:      python unified_launcher.py
"""

import sys
import time
import struct
import socket
import threading

# MSP_ATTITUDE ID
MSP_ATTITUDE = 108

# Simulation sequence:
# (time_offset, roll, pitch, heading)
SEQUENCE = [
    (0.0,  20.0, 20.0, 0.0),
    (2.0,  20.0, 20.0, 0.0),  # Hold at 20.0
    (8.0,  14.0, 14.0, 0.0),  # Slow gentle drop to 14 over 6s
    (12.0, 14.0, 14.0, 0.0),  # Hold at 14 for 4s
    (14.0, 5.0,  5.0,  0.0),  # Drop to 5 over 2s
    (15.0, 5.0,  5.0,  0.0),  # Hold at 5 (Hatch opening)
    (18.0, 5.0,  5.0,  0.0),  # Hatch open -> Drone launching
    (22.0, 5.0,  5.0,  0.0),  # Drone Launched
]

def encode_msp_attitude(roll_deg: float, pitch_deg: float, heading_deg: float) -> bytes:
    payload = struct.pack("<hhh", int(roll_deg * 10), int(pitch_deg * 10), int(heading_deg))
    size = len(payload)
    checksum = 0
    checksum ^= size
    checksum ^= MSP_ATTITUDE
    for b in payload:
        checksum ^= b
    return b"$M>" + struct.pack("<BB", size, MSP_ATTITUDE) + payload + struct.pack("<B", checksum)

def get_current_state(elapsed_time: float) -> tuple[float, float, float]:
    if elapsed_time <= SEQUENCE[0][0]:
        return SEQUENCE[0][1:]
    if elapsed_time >= SEQUENCE[-1][0]:
        return SEQUENCE[-1][1:]
        
    for i in range(len(SEQUENCE) - 1):
        t1, r1, p1, h1 = SEQUENCE[i]
        t2, r2, p2, h2 = SEQUENCE[i+1]
        
        if t1 <= elapsed_time < t2:
            progress = (elapsed_time - t1) / (t2 - t1)
            r = r1 + (r2 - r1) * progress
            p = p1 + (p2 - p1) * progress
            h = h1 + (h2 - h1) * progress
            return (r, p, h)
            
    return SEQUENCE[-1][1:]

def run_tcp_server(host="127.0.0.1", port=5555):
    print(f"\n[TCP MOCK SERVER]")
    print(f"Update unified_launcher.py -> COM_PORT = \"socket://{host}:{port}\"")
    print(f"Waiting for unified_launcher.py to connect...")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        # Allow immediate reuse of the port
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((host, port))
        server.listen(1)
        
        try:
            conn, addr = server.accept()
            with conn:
                print(f"Connected by {addr} -> Starting Simulation Sequence!")
                start_time = time.monotonic()
                
                # Make socket non-blocking so we can read and write asynchronously
                conn.setblocking(False)
                
                while True:
                    try:
                        data = conn.recv(1024)
                        if not data:
                            print("Client disconnected cleanly.")
                            break
                            
                        # We received a request, calculate current state and send response
                        elapsed = time.monotonic() - start_time
                        roll, pitch, heading = get_current_state(elapsed)
                        
                        # Only print every once in a while to not flood the server console,
                        # or just rely on the launcher dashboard.
                        
                        response = encode_msp_attitude(roll, pitch, heading)
                        conn.sendall(response)
                        
                    except BlockingIOError:
                        time.sleep(0.01)
                    except ConnectionError:
                        print("Client disconnected.")
                        break
                    
        except KeyboardInterrupt:
            print("\nShutting down.")

if __name__ == "__main__":
    run_tcp_server()
