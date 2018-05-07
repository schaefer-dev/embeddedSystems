import socket
 
def Main():
    host = "127.0.0.1"
    port = 50000
     
    mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    mySocket.bind((host,port))
    mySocket.listen()    
    BUFFER_SIZE = 8
    
    conn, addr = mySocket.accept()
    print ("Connection from: " + str(addr))
    while True:
            data = conn.recv(BUFFER_SIZE)
            if not data:
                    break
            decoded = int.from_bytes(data, byteorder='big', signed=True)
            print ("data: ", data)
            print ("dec : ", decoded)             
    conn.close()
     
if __name__ == '__main__':
    Main()
