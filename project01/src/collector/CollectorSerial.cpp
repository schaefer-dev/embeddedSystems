
#include "CollectorSerial.h"
#include <Arduino.h>
#include "main.h"


#ifdef COLLECTOR_ROBOT_SIMULATOR
/* checks if the synchronizing prefix 0x7A (z) has been send over serial, if yes return true */
bool CollectorSerial::simulatorMessageIncoming(){
    delay(5);
    if (Serial1.available() == 0)
        return false;


    char buffer[2];
    Serial1.readBytes(buffer,1);
    if ('z' == (char)buffer[0]) {
        Serial1.begin(9600);
        return true;
    }

    Serial1.begin(9600);
    return false;
}
#endif



#ifdef COLLECTOR_ROBOT_SIMULATOR
/* reads exactly 32 bytes blocking into array */
// TODO pass terminator at 50th character
void CollectorSerial::receiveSerialBlocking(char *returnArray){
    Serial1.readBytes(returnArray, 32);
    return;
}
#endif




/* returns number of bytes read and fills into returnArray, writes at most 50 bytes */
unsigned int CollectorSerial::readMessageFromSerial(char *returnArray){
    delay(5);
#ifdef COLLECTOR_DEBUG
    Serial1.readBytes(returnArray, 50);
#endif
}