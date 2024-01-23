#ifndef _ODOM_
#define _ODOM_

#define degPerInch 64.819467732
#define degPerDeg 6.901010102

//odom task
void odomTracker(void* ignore);

void odomSetCoords(double x, double y, double bearing);

extern double odomGlobalX, odomGlobalY, globalDeltaY, globalDeltaX, localX, localY, odomRadius, odomPrevPosLeft, odomPrevPosRight, odomDeltaPosLeft, odomDeltaPosRight, odomPrevAngle, odomDeltaAngle;
#endif