#include "main.h"

double odomGlobalX, odomGlobalY, globalDeltaY, globalDeltaX, localX, localY, odomRadius, odomPrevPosLeft = 0, odomPrevPosRight = 0, odomDeltaPosLeft, odomDeltaPosRight, odomDeltaInchesLeft, odomDeltaInchesRight, odomPrevBearing, odomDeltaBearing, odomDeltaAngle;

void odomTracker(void *ignore){
    printf("Odom tracker started\n");
    IMU inertial(inertialPort);

    //Rotating vector
    double ethan;

    //Holding variables
    double odomPosLeft, odomPosRight, odomBearing;

    while(true){
        if(inertial.is_calibrating()){
            odomSetCoords(0, 0, 0);
        }else{
            odomPosLeft = sensorsPosLeft;
            odomPosRight = sensorsPosRight;
            odomBearing = sensorsBearing;
            odomDeltaPosLeft = odomPosLeft - odomPrevPosLeft;
            odomDeltaPosRight = odomPosRight - odomPrevPosRight;
            odomDeltaInchesLeft = odomDeltaPosLeft/degPerInch;
            odomDeltaInchesRight = odomDeltaPosRight/degPerInch;
            odomDeltaBearing = odomPrevBearing - odomBearing;
            odomDeltaAngle = -odomDeltaBearing*toRadian;
            

            if (odomDeltaAngle != 0){
                odomRadius = odomDeltaInchesRight/odomDeltaAngle + rightDist;
                
                // printf("radius: %f\n", radius);
                // printf("x: %f\n", radius - radius*cos(odomDeltaAngle));
                // printf("y: %f\n", sin(odomDeltaAngle)*radius);
                localX = odomRadius - odomRadius*cos(odomDeltaAngle);
                localY = sin(odomDeltaAngle)*odomRadius;

                ethan = -odomBearing*toRadian;

                globalDeltaX = localX*cos(ethan) - localY*sin(ethan);
                globalDeltaY = localX*sin(ethan) + localY*cos(ethan);
            } else {
                localX = 0;
                localY = (odomDeltaPosLeft + odomDeltaPosRight)/2;

                ethan = -odomBearing*toRadian;

                globalDeltaX = localX*cos(ethan) - localY*sin(ethan);
                globalDeltaY = localX*sin(ethan) + localY*cos(ethan);
            }

            odomGlobalX += globalDeltaX;
            odomGlobalY += globalDeltaY;

            odomPrevPosLeft = odomPosLeft;
            odomPrevPosRight = odomPosRight;
            odomPrevBearing = odomBearing;
        }
    }

    delay(5);
}


void odomSetCoords(double x, double y, double bearing){
    odomPrevPosLeft = 0;
    odomPrevPosRight = 0;
    odomPrevBearing = 0;
    odomGlobalX = x;
    odomGlobalY = y;
    sensorsSetHeading(bearing);
}