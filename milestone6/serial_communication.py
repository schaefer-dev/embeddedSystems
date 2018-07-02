import serial
import time

ser = serial.Serial('COM3', 9600, timeout=0)

def sendPing():
    ping = b'\x50\x50\x51'
    ser.write(ping)
    time.sleep(1)

def sendPosUpdate():
    update = b'\x60\x00\x00\x00\x00\x00\x00'
    ser.write(update)
    time.sleep(1)

def sendOutOfBounds():
    update = b'\x61\x00\x00\x00\x00\x00\x00'
    ser.write(update)
    time.sleep(1)    

def readSerial():
    try:
        message = ser.readline()

        if (message[0] == 0x50):
            print ("He sent a ping")
        elif (message[0] == 0x51):
            print ("He sent a pong")
        else:
            messageDec = message.decode("utf-8")
            print ("He said: " + messageDec[0:len(messageDec)-1])
        
    except ser.SerialTimeoutException:
        print('Data could not be read')
        

while 1:
    sendPosUpdate()
    sendPing()

    
    


        



