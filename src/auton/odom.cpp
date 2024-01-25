#include "main.h"

double odomGlobalX = 0, odomGlobalY = 0, globalDeltaY, globalDeltaX, localX, localY, odomRadius, odomPrevPosLeft = 0, odomPrevPosRight = 0, odomDeltaPosLeft, odomDeltaPosRight, odomDeltaInchesLeft, odomDeltaInchesRight, odomPrevAngle, odomDeltaAngle;
bool pause;
void odomTracker(void *ignore){
    printf("Odom tracker started\n");
    IMU inertial(inertialPort);
    Controller master(CONTROLLER_MASTER);

    //Rotating vector
    double ethan;

    //Holding variables
    double odomPosLeft, odomPosRight, odomAngle;

    while(true){
        if(inertial.is_calibrating()){
            odomSetCoords(0, 0, 0);
        }else{
            odomPosLeft = sensorsPosLeft;
            odomPosRight = sensorsPosRight;
            odomAngle = sensorsAngle;
            odomDeltaPosLeft = odomPosLeft - odomPrevPosLeft;
            odomDeltaPosRight = odomPosRight - odomPrevPosRight;
            odomDeltaInchesLeft = odomDeltaPosLeft/degPerInch;
            odomDeltaInchesRight = odomDeltaPosRight/degPerInch;
            odomDeltaAngle = odomAngle - odomPrevAngle;
            ethan = -odomPrevAngle;

            if (odomDeltaAngle > pi){
                odomDeltaAngle -= tau;
            } else if (odomDeltaAngle < -tau){
                odomDeltaAngle += pi;
            }

            if (odomDeltaAngle != 0){
                odomRadius = odomDeltaInchesRight/odomDeltaAngle + rightDist;
                
                // printf("radius: %f\n", radius);
                // printf("x: %f\n", radius - radius*cos(odomDeltaAngle));
                // printf("y: %f\n", sin(odomDeltaAngle)*radius);
                localX = odomRadius - odomRadius*cos(odomDeltaAngle);
                localY = sin(odomDeltaAngle)*odomRadius;

                globalDeltaX = localX*cos(ethan) - localY*sin(ethan);
                globalDeltaY = localX*sin(ethan) + localY*cos(ethan);
            } else {
                localX = 0;
                localY = (odomDeltaPosLeft + odomDeltaPosRight)/2;
                
                globalDeltaX = localX*cos(ethan) - localY*sin(ethan);
                globalDeltaY = localX*sin(ethan) + localY*cos(ethan);
            }

            odomGlobalX += globalDeltaX;
            odomGlobalY += globalDeltaY;

            odomPrevPosLeft = odomPosLeft;
            odomPrevPosRight = odomPosRight;
            odomPrevAngle = odomAngle;
            // master.print(0, 1, "%f", odomAngle);
            master.print(0, 0, "%f %f", odomGlobalX, odomGlobalY);
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