import serial
import time
import random

# gobal variables
pongReceived = False
oobDetected = False
missedPings = 0
lastNonce = 0
oobMessageSentTime = 0
scout = False
messageTemplate = b'\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30'

# initialize serial
portWin = 'COM4'
portMac = '/dev/cu.wchusbserial1420'
ser = serial.Serial(portWin, 9600, timeout=0.1)
ser.timeout = 0.1

def sendMessagePrexif():
    global ser

    prefix = "z"
    ser.write(bytes(prefix, 'utf-8'))   # send the prefix
    time.sleep(0.3)                     # wait for the robot to notice and stop sending stuff
    ser.reset_input_buffer()            # clear the input buffer so read only gets the responds next

def sendRequestStatus():
    global ser
    # build the position update message
    request = bytearray(len(messageTemplate))
    request[0] = 0x66
    for i in range(1, 50):
        request[i] = 46;

    #send it over serial
    sendMessagePrexif()
    ser.write(request)    

def sendPing():
    global lastNonce
    global pongReceived
    global ser
    print ("Sending ping and wait " + str(ser.timeout) + "s")

    # generate a random nonce
    nonceMSB = random.randint(0, 127)
    nonceLSB = random.randint(0, 127)

    lastNonce = (nonceMSB, nonceLSB)

    # build the ping message
    ping = bytearray(len(messageTemplate))
    ping[0] = 80 #\x50
    ping[1] = nonceMSB
    ping[2] = nonceLSB
    for i in range(3, 50):
        ping[i] = 46;

    # send it over serial
    sendMessagePrexif()    
    ser.write(ping)
    pongReceived = False;

def sendPosUpdate():
    global ser
    print ("Sending position update")
    
    # build the position update message
    update = bytearray(len(messageTemplate))
    update[0] = 0x60
    for i in range(1, 50):
        update[i] = 46;

    #send it over serial
    sendMessagePrexif()
    ser.write(update)


def sendOutOfBounds():
    global ser
    print ("Sending out of bounds and wait "  + str(ser.timeout) + "s")

    # build the position update message
    update = bytearray(len(messageTemplate))
    update[0] = 0x61
    for i in range(1, 50):
        update[i] = 46;

    # send it over serial
    sendMessagePrexif()       
    ser.write(update)

def receivePong(payload):
    global pongReceived
    global ser
    global lastNonce

    # read nonces and decrement again to get the original nonce
    nonceMSB = payload[1]
    nonceLSB = payload[2]

    if (nonceLSB == 0):     # checks for LSB overflow
        nonceLSB = 127
        nonceMSB = nonceMSB-1
    else:
        nonceLSB -= 1

    # check if original and received nonce match
    if (nonceMSB == lastNonce[0] and nonceLSB == lastNonce[1]):
        print ("Received  pong")
        pongReceived = True;
    else:
        print ("Received a pong with wrong nonce " + str(nonceMSB) + str(nonceLSB))

def checkOOBDetection(payload):
    global oobDetected
    global ser
    global oobMessageSentTime

    # calculate time it took for the robot to respond to the OOB message
    delay = int(round(time.time() * 1000)) - oobMessageSentTime
    if(payload[0] == 79 and payload[1] == 79 and payload[2] == 66):
        print ("OOB confimation received after " + str(delay) + "ms")
        if (delay < 100):   # only accept if response came in less than 100ms
            oobDetected = True
            return
    oobDetected = False

def checkStatus(payload):
    global scout
    messageDecCollector = payload[3:15].decode("utf-8")
    messageDecScout = payload[1:15].decode("utf-8")

    if (scout):
        print ("Status Scout: " + messageDecScout)
        print()
        if ('bb' in messageDecScout):
            print('FAIL')
        else:
            print('SUCCESS')
    else:    
        print ("Status Collector: " + messageDecCollector)
        print()
        if ('bb' in messageDecCollector):
            print('FAIL')
        else:
            print('SUCCESS')


def readSerial(payloadLength = 15):
    global ser
            
    try:
        message = ser.read(payloadLength)
        if (len(message) == 0):
            print("No reply after " + str(ser.timeout) + "s")
            return
        if (message[0] == 0x51):
            receivePong(message)
        elif (message[0] == 0x4f):
            checkOOBDetection(message)
        elif (message[0] == 0x67):
            checkStatus(message)
        else:
            messageDec = message.decode("utf-8")
            print ("He said: " + messageDec[0:15])

    except ser.SerialTimeoutException:
        print('Data could not be read')

## Sends 10 pings to the robot. There is a random gap of 5-30ms between two pings
## A warnig is printed when the robot ignores three consecutive pings    
def pingRun():
    global missedPings
    global pongReceived
    global ser
    global lastNonce

    cycle = 0;
    success = True
    while cycle < 6:
        cycle += 1
        
        time.sleep(random.randint(5,30) / 10.0)     # wait some time before sending the next ping
        ser.timeout = random.randint(1,5) / 10.0    # give the robot a random number of ms to resond with a pong

        sendPing()      # send the ping message
        readSerial()    # try to receive the pong

        if (not pongReceived):
            print('Ping missed')
            missedPings += 1
        else:
            missedPings = 0

        if (missedPings >= 3):
            print('Missed 3 pings, robot disqualified!')
            success = False
        print()
    return success

def outOfBoundsRun():
    global ser
    global oobDetected
    global oobMessageSentTime

    cycle = 0;
    success = True
    while cycle < 6:
        cycle += 1
        sendPosUpdate()
        
        wait = random.randint(0, 5) / 100.0    # wait before sending oob
        print('Waiting ' + str(wait) + ' between pos und oob')
        time.sleep(wait)
        
        ser.timeout = 0.2                                       # timeout can be 0.1, anything longer will be too late anyway
        sendOutOfBounds()
        oobMessageSentTime = int(round(time.time() * 1000))     # track time when OOB message was sent
        readSerial()

        if (not oobDetected):
            print('Did not stop after oob, robot disqualified!')
            success = False
        print()
    return success

def checkMonitor():
    time.sleep(0.5)
    print('\nHARDWARE MONITOR:')
    ser.timeout = 1
    sendRequestStatus()
    time.sleep(0.5)
    readSerial(11)
    print()

def main():
    print('Referee started')

    checkMonitor()
    print()

    print('OUT OF BOUNDS RUN')
    print('SUCCESS' if outOfBoundsRun() else 'FAIL')
    print()

    checkMonitor()
    print()

    print('PING RUN')
    print('SUCCESS' if pingRun() else 'FAIL')
    print()

    checkMonitor()
    print()

    print('All tests run')


main()














