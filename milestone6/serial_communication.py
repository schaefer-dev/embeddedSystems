import serial
import time
from random import randint

ser = serial.Serial('/dev/cu.wchusbserial1420', 9600, timeout=0)
# serial port slower than RF, this might need changes to work 100%
ser.timeout = 0.1

pongReceived = False
oobDetected = True
missedPings = 0

messageTemplate = b'\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30'

def sendMessagePrexif():
    global ser

    prefix = "z"
    ser.write(bytes(prefix, 'utf-8'))
    time.sleep(0.3)
    ser.reset_input_buffer()

def sendPing():
    global pongReceived
    global ser

    print ("Sending ping")
    sendMessagePrexif()
    ping = bytearray(len(messageTemplate))
    ping[0] = 80 #\x50
    ping[1] = 80  #\x00
    ping[2] = 85  #\x01
    for i in range(3, 50):
        ping[i] = 46;
    ser.write(ping)
    pongReceived = False;

def sendPosUpdate():
    global ser

    print ("Sending update")
    sendMessagePrexif()
    update = bytearray(len(messageTemplate))
    update[0] = '/x60'
    for i in range(1, 50):
        update[i] = 46;
    ser.write(update)


def sendOutOfBounds():
    global ser

    print ("Sending out of bounds")
    sendMessagePrexif()
    update = bytearray(len(messageTemplate))
    update[0] = 0x61 #/x61
    for i in range(1, 50):
        update[i] = 46;
    ser.write(update)

def receivePong(payload):
    global pongReceived
    global ser

    if (payload[1] == 80 and payload[2] == 86):
        print ("Received a pong")
        pongReceived = True;
    else:
        print ("Received a pong with wrong nonce " + str(payload[1]) + str(payload[2]))

def checkOOBDetection(payload):
    global oobDetected
    global ser

    if(payload[0] == 79 and payload[1] == 79 and payload[2] == 66):
        oobDetected = True
    else:
        oobDetected = False

def readSerial():
    global ser

    try:
        message = ser.read(15)
        if (len(message) == 0):
            print("No Message received!")
            return
        if (message[0] == 0x51):
            receivePong(message)
        elif (message[0] == 0x4f):
            checkOOBDetection(message)
        else:
            messageDec = message.decode("utf-8")
            # messageDec.strip()
            print ("He said: " + messageDec[0:15])

    except ser.SerialTimeoutException:
        print('Data could not be read')

def pingRun():
    global missedPings
    global pongReceived
    global ser

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
    global ser
    global oobDetected

    sendOutOfBounds()
    readSerial()
    if (not oobDetected):
        print('Did not stop after oob, robot disqualified!')
        return False
    return True

print('Referee started')

print('Out of Bounds Run')
print('Success' if outOfBoundsRun() else 'Fail')

print('Ping Run')
print('Success' if pingRun() else 'Fail')

print('Out of Bounds Run')
print('Success' if outOfBoundsRun() else 'Fail')

print('Out of Bounds Run')
print('Success' if outOfBoundsRun() else 'Fail')
















