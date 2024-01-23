#include "main.h"
#include "math.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"

double odomGlobalX = 0, odomGlobalY = 0, globalDeltaY, globalDeltaX, localX, localY, odomRadius, odomPrevPosLeft = 0, odomPrevPosRight = 0, odomDeltaPosLeft, odomDeltaPosRight, odomDeltaInchesLeft, odomDeltaInchesRight, odomPrevAngle, odomDeltaAngle;
bool pause;
void odomTracker(void *ignore){
    printf("Odom tracker started\n");
    IMU inertial(inertialPort);
    Controller master(CONTROLLER_MASTER);
    master.clear();

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
            master.print(2, 0, "%f %f", odomGlobalX, odomGlobalY);
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

/*
// Online C++ compiler to run C++ program online
#include <iostream>
#include <cmath>
using namespace std;

#define degPerInch 64.819467732
#define degPerDeg 6.901010102
#define rightDist 6.102

int main() {
    double odomDeltaPosLeft = 5;
    double odomDeltaPosRight = 5;
    double odomDeltaInchesLeft = 5;
    double odomDeltaInchesRight = 5;
    double odomDeltaAngle = 0*15*0.0174533;
    double ethan = -45*0.0174533;

    if (odomDeltaAngle != 0){
        double odomRadius = odomDeltaInchesRight/odomDeltaAngle + rightDist;
    
        printf("radius: %f\n", odomRadius);
        printf("x: %f\n", odomRadius - odomRadius*cos(odomDeltaAngle));
        printf("y: %f\n", sin(odomDeltaAngle)*odomRadius);
        double localX = odomRadius - odomRadius*cos(odomDeltaAngle);
        double localY = sin(odomDeltaAngle)*odomRadius;

        double globalDeltaX = localX*cos(ethan) - localY*sin(ethan);
        double globalDeltaY = localX*sin(ethan) + localY*cos(ethan);
        
        printf("%f %f", globalDeltaX, globalDeltaY);
    } else {
        double localX = 0;
        double localY = (odomDeltaPosLeft + odomDeltaPosRight)/2;
        
        double globalDeltaX = localX*cos(ethan) - localY*sin(ethan);
        double globalDeltaY = localX*sin(ethan) + localY*cos(ethan);
        
        printf("%f %f", globalDeltaX, globalDeltaY);
    }

    
    return 0;
}
*/