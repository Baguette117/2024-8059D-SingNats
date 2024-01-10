#ifndef _ODOM_
#define _ODOM_

#define degPerInch 70.81946774
#define degPerDeg 6.903682494

//odom task
void odomTracker(void* ignore);

void odomSetCoords(double x, double y, double bearing);

extern double odomGlobalX, odomGlobalY, globalDeltaY, globalDeltaX, localX, localY, odomRadius, odomPrevPosLeft, odomPrevPosRight, odomDeltaPosLeft, odomDeltaPosRight, odomPrevBearing, odomDeltaBearing, odomDeltaAngle;
#endif