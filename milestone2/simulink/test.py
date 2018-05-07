import socket
 
def Main():
    host = "127.0.0.1"
    port = 50000
     
    mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    mySocket.bind((host,port))
    mySocket.listen(1)
    BUFFER_SIZE = 8
    
    conn, addr = mySocket.accept()
    print ("Connection from: " + str(addr))
    while True:
            data = conn.recv(BUFFER_SIZE)
            if not data:
                    break
            print ("from connected  user: ", data)
             
            #data = str(data).upper()
            #print ("sending: " + str(data))
            #conn.send(data.encode())
             
    conn.close()
     
if __name__ == '__main__':
    Main()
