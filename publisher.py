import socket

HOST = '127.0.0.1'  # Aynı makine
PORT = 5556         # Unity'de tanımladığın port

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    print("Bağlanıyor...")
    s.connect((HOST, PORT))
    print("Bağlandı!")

    buffer = ""
    while True:
        data = s.recv(1024).decode('utf-8')
        if not data:
            break

        buffer += data
        while '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            try:
                hits = [float(x) for x in line.strip().split(',')]
                print("Sonar verisi:", hits)
            except ValueError as e:
                print("Veri hatası:", e)