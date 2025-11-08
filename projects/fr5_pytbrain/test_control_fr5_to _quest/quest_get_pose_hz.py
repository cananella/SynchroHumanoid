#!/usr/bin/env python3
"""
Quest Pose Receiver with Hz Measurement
"""

import socket
import time

def start_server(host='0.0.0.0', port=5454):
    """
    Quest pose data receiver with Hz measurement

    Args:
        host: Listen address (0.0.0.0 = all interfaces)
        port: Listen port
    """
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    server.bind((host, port))
    server.listen(1)
    print(f"[SERVER] Listening on {host}:{port}")
    print("[SERVER] Waiting for a connection...")

    conn, addr = server.accept()
    print(f"[CONNECTED] {addr}\n")

    # Hz measurement variables
    packet_count = 0
    start_time = time.time()
    last_report_time = start_time

    with conn:
        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    print(f"\n[DISCONNECTED] {addr}")
                    break

                decoded_lines = data.decode().strip().split('\n')
                for decoded in decoded_lines:
                    if decoded:
                        packet_count += 1

                        # Parse data
                        try:
                            values = [float(x) for x in decoded.split(',')]
                            if len(values) == 6:
                                x, y, z, rx, ry, rz = values
                                print(f"[{packet_count:4d}] X:{x:7.4f} Y:{y:7.4f} Z:{z:7.4f} | RX:{rx:7.2f} RY:{ry:6.2f} RZ:{rz:7.2f}")
                            else:
                                print(f"[RECEIVED] {decoded}")
                        except:
                            print(f"[RECEIVED] {decoded}")

                        # Hz measurement - report every 2 seconds
                        current_time = time.time()
                        elapsed = current_time - last_report_time

                        if elapsed >= 2.0:
                            hz = packet_count / elapsed
                            total_hz = packet_count / (current_time - start_time)

                            print(f"\n{'='*60}")
                            print(f"📊 Hz: {hz:.2f} Hz (last 2s: {packet_count} packets)")
                            print(f"📈 Average: {total_hz:.2f} Hz")
                            print(f"{'='*60}\n")

                            # Reset for next period
                            packet_count = 0
                            last_report_time = current_time

        except ConnectionResetError:
            print(f"\n[CONNECTION LOST] {addr}")
        except KeyboardInterrupt:
            print(f"\n[INTERRUPTED]")
        finally:
            # Final report
            total_time = time.time() - start_time
            total_packets = packet_count

            print(f"\n{'='*60}")
            print(f"📋 Final Report")
            print(f"{'='*60}")
            print(f"Total time: {total_time:.2f}s")
            print(f"Total packets: {total_packets}")
            if total_time > 0:
                print(f"Average Hz: {total_packets / total_time:.2f}")
            print(f"{'='*60}")

            print("[SERVER] Closing connection.")

if __name__ == "__main__":
    start_server()
