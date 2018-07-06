//
// Created by rafael on 06.07.18.
//

#ifndef EMBEDDEDSYSTEMS18_SCOUTLINESENSORS_H
#define EMBEDDEDSYSTEMS18_SCOUTLINESENSORS_H

class ScoutLineSensors
{

public:
    static void init();
    static void calibrate();
    static bool detectLine();
    static void checkForLines();
    static void readNewLines();

private:
    ScoutLineSensors();
};

#endif //EMBEDDEDSYSTEMS18_SCOUTLINESENSORS_H
