#include "main.h"

double cap(double value, double cap){
    if (value > cap) return cap;
    else if (value < -cap) return -cap;
    else return value;
}

double absadd(double value, double add){
    if (value > 0) value += add;
    else if (value < 0) value -= add;
    return value;
}

double boundRad(double radians){
    radians = fmod(radians, tau);
    if (radians < 0) radians += tau;
    return radians;
}

double boundDeg(double degrees){
    degrees = fmod(degrees, 360);
    if (degrees < 0) degrees += 360;
    return degrees;
}