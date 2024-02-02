#include "main.h"
 //externs
double sensorsPosLeft, sensorsPosRight, sensorsVelocityLeft, sensorsVelocityRight, sensorsVelocity, sensorsBearing, sensorsAngle, sensorsBearingOffset = 0;
bool sensorsEnable = true;

void sensorsTracker(void *ignore){
    Motor leftFront(leftFrontPort, leftFrontGearset, leftFrontReversed, leftFrontEncoder);
    Motor leftMid(leftMidPort, leftMidGearset, leftMidReversed, leftMidEncoder);
    Motor leftBack(leftBackPort, leftBackGearset, leftBackReversed, leftBackEncoder);
    Motor rightFront(rightFrontPort, rightFrontGearset, rightFrontReversed, rightFrontEncoder);
    Motor rightMid(rightMidPort, rightMidGearset, rightMidReversed, rightMidEncoder);
    Motor rightBack(rightBackPort, rightBackGearset, rightBackReversed, rightBackEncoder);
    Imu inertial(inertialPort);

    leftFront.tare_position();
    leftMid.tare_position();
    leftBack.tare_position();
    rightFront.tare_position();
    rightMid.tare_position();
    rightBack.tare_position();

    while(true){
        if(!inertial.is_calibrating()){
            sensorsPosLeft = (leftFront.get_position() + leftMid.get_position() + leftBack.get_position())/3;
            sensorsPosRight = (rightFront.get_position() + rightMid.get_position() + rightBack.get_position())/3;

            sensorsVelocityLeft = (leftFront.get_actual_velocity() + leftMid.get_actual_velocity() + leftBack.get_actual_velocity())/3;
            sensorsVelocityRight = (rightFront.get_actual_velocity() + rightMid.get_actual_velocity() + rightBack.get_actual_velocity())/3;
            sensorsVelocity = (sensorsVelocityLeft + sensorsVelocityRight)/2;
            
            sensorsBearing = boundDeg(inertial.get_heading() + sensorsBearingOffset);
            sensorsAngle = boundRad(sensorsBearing*toRadian);
        }

        delay(5);
    }
}

void sensorsSetHeading(double bearing){
    sensorsBearingOffset += bearing - sensorsBearing;
}

void sensorsTare(){
    Motor leftFront(leftFrontPort, leftFrontGearset, leftFrontReversed, leftFrontEncoder);
    Motor leftMid(leftMidPort, leftMidGearset, leftMidReversed, leftMidEncoder);
    Motor leftBack(leftBackPort, leftBackGearset, leftBackReversed, leftBackEncoder);
    Motor rightFront(rightFrontPort, rightFrontGearset, rightFrontReversed, rightFrontEncoder);
    Motor rightMid(rightMidPort, rightMidGearset, rightMidReversed, rightMidEncoder);
    Motor rightBack(rightBackPort, rightBackGearset, rightBackReversed, rightBackEncoder);
    
    leftFront.tare_position();
    leftMid.tare_position();
    leftBack.tare_position();
    rightFront.tare_position();
    rightMid.tare_position();
    rightBack.tare_position();
}