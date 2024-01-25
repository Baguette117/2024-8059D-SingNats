#include "main.h"

//externs
double controlSpeedCap = defaultSpeedCap, controlKP = defaultKP, controlKD = defaultKD, controlKI = defaultKI, controlTurnKP = defaultTurnKP, controlTurnKD = defaultTurnKD, controlTurnKI = defaultTurnKI, controlRampingMax = defaultRampingMax;
//wheels
double controlPowLeft, controlPowRight, controlTargPowLeft = 0, controlTargPowRight = 0, controlTargLeft = 0, controlTargRight = 0, controlErrorLeft, controlErrorRight, controlPrevErrorLeft = 0, controlPrevErrorRight = 0, controlDerivLeft, controlDerivRight;
//orientation
double controlTargBearing = 0/*, controlErrorBearing, controlPrevErrorBearing = 0, controlDerivBearing*/;
//other
bool controlPIDEnable = false, controlTurnMode = false, controlRunning = false/*, controlTurnLeft*/;

void controlPID(void *ignore){
    Motor leftFront(leftFrontPort, leftFrontGearset, leftFrontReversed, leftFrontEncoder);
    Motor leftMid(leftMidPort, leftMidGearset, leftMidReversed, leftMidEncoder);
    Motor leftBack(leftBackPort, leftBackGearset, leftBackReversed, leftBackEncoder);
    Motor rightFront(rightFrontPort, rightFrontGearset, rightFrontReversed, rightFrontEncoder);
    Motor rightMid(rightMidPort, rightMidGearset, rightMidReversed, rightMidEncoder);
    Motor rightBack(rightBackPort, rightBackGearset, rightBackReversed, rightBackEncoder);
    Imu inertial(inertialPort);
    Controller master (CONTROLLER_MASTER);

    double deltaLeft, deltaRight;

    leftFront.tare_position();
    leftMid.tare_position();
    leftBack.tare_position();
    rightFront.tare_position();
    rightMid.tare_position();
    rightBack.tare_position();

    while(true){
        if (controlPIDEnable && !inertial.is_calibrating()){
            controlErrorLeft = controlTargLeft - sensorsPosLeft;
            controlErrorRight = controlTargRight - sensorsPosRight;
            
            controlDerivLeft = controlErrorLeft - controlPrevErrorLeft;
            controlDerivRight = controlErrorRight - controlPrevErrorRight;
            
            controlPrevErrorLeft = controlErrorLeft;
            controlPrevErrorRight = controlErrorRight;

            if(controlTurnMode){
                controlTargPowLeft = controlErrorLeft*controlTurnKP + controlDerivLeft*controlTurnKD;
                controlTargPowRight = controlErrorRight*controlTurnKP + controlDerivRight*controlTurnKD;

                if(fabs(controlErrorLeft) < defaultBearingTolerance && fabs(controlErrorRight) < defaultBearingTolerance){
                    controlTargPowLeft  = absadd(controlTargLeft, controlTurnKI);
                    controlTargPowRight = absadd(controlTargRight, controlTurnKI); 
                }

                // controlErrorBearing = controlTargBearing - sensorsBearing;
            }else{
                controlTargPowLeft = controlErrorLeft*controlKP + controlDerivLeft*controlKD;
                controlTargPowRight = controlErrorRight*controlKP + controlDerivRight*controlKD;
            
                if(fabs(controlErrorLeft) < defaultDistanceTolerance && fabs(controlErrorRight) < defaultDistanceTolerance){
                    controlTargPowLeft = absadd(controlTargPowLeft, controlKI);
                    controlTargPowRight = absadd(controlTargPowRight, controlKI);
                }
            }

            deltaLeft = controlTargPowLeft - controlPowLeft;
            deltaRight = controlTargPowRight - controlPowRight;

            controlPowLeft += cap(deltaLeft, controlRampingMax);
            controlPowRight += cap(deltaRight, controlRampingMax);
            controlPowLeft = cap(controlPowLeft, controlSpeedCap);
            controlPowRight = cap(controlPowRight, controlSpeedCap);
            controlDrive(controlPowLeft, controlPowRight);
            controlRunning = true;
        }

        delay(5);
    }
}


//basic movement
void controlDrive(double left, double right){
    Motor leftFront(leftFrontPort,  leftFrontReversed);
    Motor leftMid(leftMidPort, leftMidReversed);
    Motor leftBack(leftBackPort, leftBackReversed);
    Motor rightFront(rightFrontPort, rightFrontReversed);
    Motor rightMid(rightMidPort, rightMidReversed);
    Motor rightBack(rightBackPort, rightBackReversed);
    
    leftFront.move(left);
    leftMid.move(left);
    leftBack.move(left);
    rightFront.move(right);
    rightMid.move(right);
    rightBack.move(right);
}


//relative movements
bool controlMove(double inches, double timeout, double kp, double kd, double ki){
    double start = millis();
    controlTurnMode = false;
    printf("controlMove | inches: %f\ttimeout: %f\n", inches, timeout);

    controlKP = kp;
    controlKD = kd;
    controlKI = ki;

    controlTargLeft += inches*degPerInch;
    controlTargRight += inches*degPerInch;

    delay(50);
    while(fabs(controlErrorLeft) > defaultDistanceTolerance || fabs(controlErrorRight) > defaultDistanceTolerance || fabs(sensorsVelocity) > defaultVelocityTolerance){
        if (millis() - start > timeout){
            printf("Timeout\n");
            return false;
        }
        printf("Moving\n");
        delay(50);
    }

    return true;
}

bool controlTurn(double degrees, double timeout, double kp, double kd, double ki){
    double start = millis();
    controlTurnMode = true;
    printf("controlTurn | degrees: %f\ttimeout: %f\n", degrees, timeout);

    controlTurnKP = kp;
    controlTurnKD = kd;
    controlTurnKI = ki;
    controlTargLeft += degrees*degPerDeg;
    controlTargRight -= degrees*degPerDeg;
    controlTargBearing = boundDeg(controlTargBearing + degrees);

    delay(50);
    while(fabs(controlErrorLeft) > defaultBearingTolerance || fabs(controlErrorRight) > defaultBearingTolerance || fabs(sensorsVelocityLeft) > defaultVelocityTolerance || fabs(sensorsVelocityRight) > defaultVelocityTolerance){
        if (millis() - start > timeout){
            printf("Timeout\n");
            return false;
        }
        printf("Turning\n");
        delay(50);
    }
    return true;
}


//absolute movements
bool controlMoveTo(bool backwards, double x, double y, double turnTimeout, double moveTimeout, double moveKP, double moveKD, double moveKI, double turnKP, double turnKD, double turnKI){
    printf("controlMoveTo | x: %f\ty: %f\ttimeout: %f, %f\n", x, y, turnTimeout, moveTimeout);
    Controller master(CONTROLLER_MASTER);

    bool success = false;
    double diffX = x - odomGlobalX, diffY = y - odomGlobalY;
    double distance = sqrt(diffX*diffX + diffY*diffY);
    double angle = atan2(diffY, diffX);
    double bearing = boundDeg(90 - angle*toDegree);

    master.print(0, 0, "%f %f", diffX, diffY);
    master.print(1, 0, "%f %f", bearing, distance);

    if(backwards){
        success = controlTurnTo(boundDeg(bearing + 180), turnTimeout, turnKP, turnKD, turnKI);
        success &= controlMove(-distance, moveTimeout, moveKP, moveKD, moveKI);
    } else {
        success = controlTurnTo(bearing, turnTimeout, turnKP, turnKD, turnKI);
        success &= controlMove(distance, moveTimeout, moveKP, moveKD, moveKI);
    }
    return success;
}

bool controlTurnTo(double bearing, double timeout, double kp, double kd, double ki){
    double start = millis();
    controlTurnMode = true;
    printf("controlTurnTo | bearing: %f\ttimeout: %f\n", bearing, timeout);

    double errorBearing = boundDeg(bearing - sensorsBearing);
    if(errorBearing > 180) errorBearing -= 360;

    controlTurnKP = kp;
    controlTurnKD = kd;
    controlTurnKI = ki;
    controlTargLeft += errorBearing*degPerDeg;
    controlTargRight -= errorBearing*degPerDeg;
    controlTargBearing = bearing;

    delay(50);
    while(fabs(controlErrorLeft) > defaultBearingTolerance || fabs(controlErrorRight) > defaultBearingTolerance || fabs(sensorsVelocityLeft) > defaultVelocityTolerance || fabs(sensorsVelocityRight) > defaultVelocityTolerance){
        if (millis() - start > timeout){
            printf("Timeout\n");
            return false;
        }
        printf("Turning\n");
        delay(50);
    }
    return true;
}

bool controlTurnLeftTo(double bearing, double timeout, double kp, double kd, double ki){
    double start = millis();
    controlTurnMode = true;
    printf("controlTurnLeftTo | bearing: %f\ttimeout: %f\n", bearing, timeout);

    double errorBearing = bearing - sensorsBearing;
    if (errorBearing > 0) errorBearing -= 360;

    controlTurnKP = kp;
    controlTurnKD = kd;
    controlTurnKI = ki;
    controlTargLeft -= errorBearing*degPerDeg;
    controlTargRight += errorBearing*degPerDeg;
    controlTargBearing = bearing;

    delay(50);
    while(fabs(controlErrorLeft) > defaultBearingTolerance || fabs(controlErrorRight) > defaultBearingTolerance || fabs(sensorsVelocityLeft) > defaultVelocityTolerance || fabs(sensorsVelocityRight) > defaultVelocityTolerance){
        if (millis() - start > timeout){
            printf("Timeout\n");
            return false;
        }
        printf("Turning\n");
        delay(50);
    }
    return true;
}

bool controlTurnRightTo(double bearing, double timeout, double kp, double kd, double ki){
    double start = millis();
    controlTurnMode = true;
    printf("controlTurnRightTo | bearing: %f\ttimeout: %f\n", bearing, timeout);

    double errorBearing = bearing - sensorsBearing;
    if (errorBearing < 0) errorBearing += 360;

    controlTurnKP = kp;
    controlTurnKD = kd;
    controlTurnKI = ki;
    controlTargLeft -= errorBearing*degPerDeg;
    controlTargRight += errorBearing*degPerDeg;
    controlTargBearing = bearing;

    delay(50);
    while(fabs(controlErrorLeft) > defaultBearingTolerance || fabs(controlErrorRight) > defaultBearingTolerance || fabs(sensorsVelocityLeft) > defaultVelocityTolerance || fabs(sensorsVelocityRight) > defaultVelocityTolerance){
        if (millis() - start > timeout){
            printf("Timeout\n");
            return false;
        }
        printf("Turning\n");
        delay(50);
    }
    return true;
}

void controlSetCoords(double x, double y, double bearing){
    printf("controlSetCoords | x: %f\ty: %f\tbearing: %f\n", x, y, bearing);

    Motor leftFront(leftFrontPort, leftFrontGearset, leftFrontReversed, leftFrontEncoder);
    Motor leftMid(leftMidPort, leftMidGearset, leftMidReversed, leftMidEncoder);
    Motor leftBack(leftBackPort, leftBackGearset, leftBackReversed, leftBackEncoder);
    Motor rightFront(rightFrontPort, rightFrontGearset, rightFrontReversed, rightFrontEncoder);
    Motor rightMid(rightMidPort, rightMidGearset, rightMidReversed, rightMidEncoder);
    Motor rightBack(rightBackPort, rightBackGearset, rightBackReversed, rightBackEncoder);

    sensorsTare();
    odomSetCoords(x, y, bearing);
    controlTargLeft = 0;
    controlTargRight = 0;
    controlTargBearing = boundDeg(bearing);
    odomPrevPosLeft = 0;
    odomPrevPosRight = 0;
    delay(50);
}