import serial
import time
from random import randint

ser = serial.Serial('COM3', 9600, timeout=0)
timeSystemStart = int(round(time.time() * 1000))

pongReceived = False
moving = True
missedPings = 0

def sendPing():
    print ("Sending ping")
    ping = b'\x50\x00\x01'
    ser.write(ping)
    pongReceived = False;
    time.sleep(2 + randint(0, 2))

def sendPosUpdate():
    print ("Sending update")
    update = b'\x60\x00\x00\x00\x00\x00\x00'
    ser.write(update)
    time.sleep(2 + randint(0, 2))

def sendOutOfBounds():
    print ("Sending out of bounds")
    update = b'\x61\x00\x00\x00\x00\x00\x00'
    ser.write(update)
    time.sleep(2 + randint(0, 2))

def receivePong(payload):
    pongReceived = True;
    if (payload[1] == 0x00 and payload[2] == 0x02):
        print ("Received a pong")
    else:    
        print ("Received a pong with wrong nonce")

def receiveMovement(payload):
    print ("Received movement")
    if(payload[1] == 0x00 and payload[2] == 0x00):
        moving = False
    else:
        moving = True


def readSerial():
    try:
        message = ser.readline()
        if (message[0] == 0x51):
            receivePong(message)
        elif (message[0] == 0x80):
            receiveMovement(message)
        else:
            messageDec = message.decode("utf-8")
            print ("He said: " + messageDec[0:len(messageDec)-1])
        
    except ser.SerialTimeoutException:
        print('Data could not be read')
        
def pingRun():
    return True
    sendPing()
    readSerial()
    if (not pongReceived):
        print('Ping missed')
        missedPings += 1
    sendPing()
    readSerial()
    if (not pongReceived):
        print('Ping missed')
        missedPings += 1
    sendPing()
    readSerial()
    if (not pongReceived):
        print('Ping missed')
        missedPings += 1
    if (missedPings >= 3):
        print('Missed 3 pings, robot disqualified!')
        return False
    return True 

def outOfBoundsRun():
    return True
    sendOutOfBounds()
    readSerial()
    if (moving):
        print('Did not stop after oob, robot disqualified!')
        return False
    for i in range (0, 30):    
        time.sleep(1)
        readSerial()
        if (moving):
            print('Did not stop after oob, robot disqualified!')
            return False
    return True    
    
    
print('Referee started')
print('Ping Run')
print('Success' if pingRun() else 'Fail')

print('Out of Bounds Run')
print('Success' if outOfBoundsRun() else 'Fail')
    


    

    


    
    


        



