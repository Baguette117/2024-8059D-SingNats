#ifndef _SENSORS_
#define _SENSORS_

//sensors task
void sensorsTracker(void* ignore);
void sensorsSetHeading(double heading);

extern double sensorsPosLeft, sensorsPosRight, sensorsVelocityLeft, sensorsVelocityRight, sensorsVelocity, sensorsBearing, sensorsAngle;

#endif