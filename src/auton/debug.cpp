#include "control.hpp"
#include "main.h"

int debugMode = 0b00000000;
int interval = 200;

void debugTerminal(void* ignore){
    Imu inertial(inertialPort);
    Controller master(CONTROLLER_MASTER);

    printf("Debug task started\n");
    while(true){
        if(inertial.is_calibrating()){
            printf("Inertial is calibrating\n");
            // master.print(2, 0, "Inertial is calibrating");
        }else{
            if(debugMode & 1) printControl();
            if(debugMode & 2) printOdom();
            if(debugMode % 4) printSensors();
        }
        delay(interval);
    }
}

void printControl(){
    printf(
        "PowLeft: %f, PowRight: %f | TargPowLeft: %f, TargPowRight: %f\n"
        "TargLeft: %F, TargRIght: %f | ErrorLeft: %f, ErrorRight: %f | DerivLeft: %f, DerivRight: %f\n"
        "TargBearing: %f\n",
        controlPowLeft, controlPowRight, controlTargPowLeft, controlTargPowRight, controlTargLeft, controlTargRight, controlErrorLeft, controlErrorRight, controlDerivLeft, controlDerivRight, controlTargBearing);
}

void printOdom(){
    printf(
        "GlobalX: %f, GlobalY: %f\n"
        "LocalX: %f, LocalY: %f\n"
        "GlobalDeltaX: %f, GlobalDeltaY: %f\n"
        "PrevPosLeft: %f, PrevPosRight: %f\n"
        "DeltaPosLeft: %f, DeltaPosRight: %f\n"
        "PrevBearing: %f, DeltaBearing: %f, DeltaAngle: %f\n"
        "radius: %f\n",
        odomGlobalX, odomGlobalY, odomPrevPosLeft, odomPrevPosRight, odomDeltaPosLeft, odomDeltaPosRight, odomPrevBearing, odomDeltaBearing, odomDeltaAngle, localX, localY, globalDeltaX, globalDeltaY, odomRadius);
}

void printSensors(){
    printf(
        "LeftPos: %f, RightPos: %f\n"
        "LeftVelocity: %f, RightVelocity: %f, Velocity: %f\n"
        "Bearing: %f, Angle: %f\n",
        sensorsPosLeft, sensorsPosRight, sensorsVelocityLeft, sensorsVelocityRight, sensorsVelocity, sensorsBearing, sensorsAngle);
}