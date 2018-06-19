//
// Created by Daniel Schäfer on 19.06.18.
//

#include "Utility.h"



int Utility::int_pow(int base, int exp)
{
    int result = 1;
    while (exp)
    {
        if (exp & 1)
            result *= base;
        exp /= 2;
        base *= base;
    }
    return result;
}