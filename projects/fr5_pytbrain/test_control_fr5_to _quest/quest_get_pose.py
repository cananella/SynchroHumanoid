import socket

def start_server(host='0.0.0.0', port=5454):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    server.bind((host, port))
    server.listen(1)
    print(f"[SERVER] Listening on {host}:{port}")
    print("[SERVER] Waiting for a connection...")

    conn, addr = server.accept()
    print(f"[CONNECTED] {addr}")

    with conn:
        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    print(f"[DISCONNECTED] {addr}")
                    break
                
                decoded_lines = data.decode().strip().split('\n')
                for decoded in decoded_lines:
                    if decoded:
                        print(f"[RECEIVED] {decoded}")
        except ConnectionResetError:
            print(f"[CONNECTION LOST] {addr}")
        finally:
            print("[SERVER] Closing connection.")

if __name__ == "__main__":
    start_server()