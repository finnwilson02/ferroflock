import socket

def receive_data():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('127.0.0.1', 8080))  # connect to the c++ server

    while True:
        data = client_socket.recv(1024).decode('utf-8')  # receive data from c++ server
        if data:
            print(data)

if __name__ == "__main__":
    receive_data()
