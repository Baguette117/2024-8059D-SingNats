#ifndef _SENSORS_
#define _SENSORS_

//sensors task
void sensorsTracker(void* ignore);
void sensorsSetHeading(double bearing);
void sensorsTare();

extern double sensorsPosLeft, sensorsPosRight, sensorsVelocityLeft, sensorsVelocityRight, sensorsVelocity, sensorsBearing, sensorsAngle, sensorsBearingOffset;

#endif