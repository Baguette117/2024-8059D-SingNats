#ifndef _DEBUG_
#define _DEBUG_

//debug task
void debugTerminal(void* ignore);

//print values
void printControl();
void printOdom();
void printSensors();

/*
*1st bit: Control
*2nd bit: Odom
*3rd bit: Sensors
*/
extern int debugMode;
extern int interval;

#endif