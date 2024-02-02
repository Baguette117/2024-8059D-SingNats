#include "main.h"
#include "sensors.hpp"

//externs
double odomGlobalX = 0, odomGlobalY = 0, globalDeltaY, globalDeltaX, localX, localY, odomRadius, odomPrevPosLeft = 0, odomPrevPosRight = 0, odomDeltaPosLeft, odomDeltaPosRight, odomDeltaInchesLeft, odomDeltaInchesRight, odomPrevAngle, odomDeltaAngle;
bool odomEnable = true;

void odomTracker(void *ignore){
    printf("Odom tracker started\n");
    IMU inertial(inertialPort);
    Controller master(CONTROLLER_MASTER);

    //Rotating vector
    double odomRotationAngle;

    //Holding variables
    double odomPosLeft, odomPosRight, odomAngle;
    int counter = 0;

    while(true){
        if (inertial.is_calibrating()){
            odomSetCoords(0, 0, 0);
        } else if (!odomEnable){
        } else {
            odomPosLeft = sensorsPosLeft;
            odomPosRight = sensorsPosRight;
            odomAngle = sensorsAngle;
            odomDeltaPosLeft = odomPosLeft - odomPrevPosLeft;
            odomDeltaPosRight = odomPosRight - odomPrevPosRight;
            odomDeltaInchesLeft = odomDeltaPosLeft/degPerInch;
            odomDeltaInchesRight = odomDeltaPosRight/degPerInch;
            odomDeltaAngle = odomAngle - odomPrevAngle;
            odomRotationAngle = -odomPrevAngle;

            if (odomDeltaAngle > pi){
                odomDeltaAngle -= tau;
            } else if (odomDeltaAngle < -tau){
                odomDeltaAngle += pi;
            }

            if (odomDeltaAngle != 0){
                odomRadius = odomDeltaInchesRight/odomDeltaAngle + rightDist;

                localX = odomRadius - odomRadius*cos(odomDeltaAngle);
                localY = sin(odomDeltaAngle)*odomRadius;

                globalDeltaX = localX*cos(odomRotationAngle) - localY*sin(odomRotationAngle);
                globalDeltaY = localX*sin(odomRotationAngle) + localY*cos(odomRotationAngle);
            } else {
                localX = 0;
                localY = (odomDeltaPosLeft + odomDeltaPosRight)/2;
                
                globalDeltaX = localX*cos(odomRotationAngle) - localY*sin(odomRotationAngle);
                globalDeltaY = localX*sin(odomRotationAngle) + localY*cos(odomRotationAngle);
            }

            odomGlobalX += globalDeltaX;
            odomGlobalY += globalDeltaY;

            odomPrevPosLeft = odomPosLeft;
            odomPrevPosRight = odomPosRight;
            odomPrevAngle = odomAngle;
            if (!((++counter)%1)){
            master.print(2, 0, "%f", sensorsBearing);
            master.print(0, 0, "%f %f", odomGlobalX, odomGlobalY);

            }
        }
    }

    delay(5);
}


void odomSetCoords(double x, double y, double bearing){
    odomPrevPosLeft = 0;
    odomPrevPosRight = 0;
    odomPrevAngle = boundRad(bearing*toRadian);
    odomGlobalX = x;
    odomGlobalY = y;
    sensorsSetHeading(boundDeg(bearing));
}