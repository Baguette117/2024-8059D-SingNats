#include "control.hpp"
#include "main.h"
#include "mechlib.hpp"
#include "odom.hpp"
#include "paths.hpp"
#include "pros/rtos.hpp"
#include "sensors.hpp"

bool autonwl = false, autonwr = false;

bool calibration(int path){
    Task autonSensorsTask(sensorsTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sensors Task");
    Task autonOdomTask(odomTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
    Task autonPIDTask(controlPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PID Task");
    Task autonDebugTask(debugTerminal, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Debug Task");
    bool success = false;
    controlPIDEnable = true;
    debugMode = 0b110;

    switch (path){
        case 0:
            success = controlMove(24, 3000);
            break;

        case 1:
            success = controlTurn(180, 3000);
            break;

        case 2:
            success = controlMove(12, 3500);
            success &= controlMoveTo(false, -12, 12, 1000, 1000);
            success &= controlMoveTo(false, 0, 0, 1000, 3000);
            break;
    }

    controlPIDEnable = false;
    autonDebugTask.remove();
    autonPIDTask.remove();
    autonOdomTask.remove();
    autonSensorsTask.remove();

    return success;
}

void matchload(){
    Task autonSensorsTask(sensorsTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sensors Task");
    Task autonOdomTask(odomTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
    Task autonPIDTask(controlPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PID Task");
    Task autonDebugTask(debugTerminal, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Debug Task");
	Motor intake(intakePort, false);
	ADIDigitalOut wingLeft(wingLeftPort, false);
	ADIDigitalOut wingRight(wingRightPort, false);
    controlPIDEnable = true;

    shoot();

    controlSpeedCap = 80;
    controlMove(26, 1000);
    controlTurnTo(45, 1000);
    intake.move(80);
    controlMove(5, 100);
    intake.move(0);
    controlMove(25, 650);
    controlMove(-8, 1000);
    controlSetCoords(0, 28, 45);
    controlSpeedCap = 120;
    controlTurnTo(0, 1250);
    controlMove(-11, 500);
    wingLeft.set_value(true);
    controlMove(-15, 1000, .25);
    // controlTurnTo(-30, 500);
    wingLeft.set_value(false);
    autonwr = true;
    wingRight.set_value(true);
    // controlMoveTo(true, 15, -17, 500, 1000);
    // delay(4000);
    // controlMoveTo(true, 2, 2, 500, 1000);
    controlTurnTo(-90 - atan2(-24 - odomGlobalY, 36 - odomGlobalX)*toDegree, 1000);
    controlTurnTo(-60 - atan2(-24 - odomGlobalY, 36 - odomGlobalX)*toDegree, 1000);
    controlMove(-30, 1000);
    controlPIDEnable = false;
    controlDrive(-5, -15);

    autonDebugTask.remove();
    autonPIDTask.remove();
    autonOdomTask.remove();
    autonSensorsTask.remove();
}

void balls(){
    Task autonSensorsTask(sensorsTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sensors Task");
    Task autonOdomTask(odomTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
    Task autonPIDTask(controlPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PID Task");
    Task autonDebugTask(debugTerminal, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Debug Task");
	Motor intake(intakePort, false);
	ADIDigitalOut wingLeft(wingLeftPort, false);
	ADIDigitalOut wingRight(wingRightPort, false);
    controlPIDEnable = true;

    shoot();
    intake.move(-50);
    controlSetCoords(odomGlobalX, odomGlobalY, 180);
    controlMove(-22.5, 1000);
    controlMoveTo(true, -19.5, 46, 500, 1000, .15);
    controlTurnTo(80, 1000);
    intake.move(127);
    delay(400);
    controlTurnTo(100 - atan2(60 - odomGlobalY, -36 - odomGlobalX)*toDegree, 1000);
    controlPIDEnable = false;
    controlDrive(80, 80);
    intake.move(-127);
    delay(600);
    controlPIDEnable = true;
    controlSetCoords(odomGlobalX, odomGlobalY, sensorsBearing);
    controlTurnTo(-90, 1000);
    wingLeft.set_value(true);
    wingRight.set_value(true);
    intake.move(-40);
    controlMove(-30, 1000);
    controlPIDEnable = false;
    controlDrive(127, 127);
    delay(250);
    controlPIDEnable = true;
    controlTurnLeftTo(90, 1000);
    intake.move(127);
    delay(400);
    // controlMoveTo(false, -0.5, odomGlobalY, 500, 1000);
    // controlSetCoords(-0.5, 48, 90);
    wingLeft.set_value(false);
    wingRight.set_value(false);
    // controlMove(-10, 1000);
    controlSpeedCap = 75;
    controlMoveTo(false, -36, 38, 1200, 800, .20);
    controlSpeedCap = 120;
    intake.move(-127);
    controlMove(-10, 750);
    controlTurnTo(65 - atan2(24  - odomGlobalY, 10 - odomGlobalX)*toDegree, 1000);
    controlDrive(80, 80);
    intake.move(80);

    controlPIDEnable = false;
    autonDebugTask.remove();
    autonPIDTask.remove();
    autonOdomTask.remove();
    autonSensorsTask.remove();
}

void failsafe(){
    Task autonSensorsTask(sensorsTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sensors Task");
    Task autonOdomTask(odomTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
    Task autonPIDTask(controlPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PID Task");
    Task autonDebugTask(debugTerminal, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Debug Task");
	Motor intake(intakePort, false);
	ADIDigitalOut wingLeft(wingLeftPort, false);
	ADIDigitalOut wingRight(wingRightPort, false);
    controlPIDEnable = true;
    
    shoot();

    controlMoveTo(false, 0, 54, 200, 1500);
    controlTurnTo(90, 1500);
    controlPIDEnable = false;
    intake.move(80);
    controlDrive(80, 80);
    delay(500);
    intake.move(0);
    controlPIDEnable = true;
    controlMove(-7, 1000);
    controlMoveTo(false, 2, 5, 1000, 1500, .10, 8);
    controlMoveTo(true, -24, 3, 1000, 1000);
    autonwl = true;
    wingLeft.set_value(true);
    controlDrive(-15, -5);

    controlPIDEnable = false;
    autonDebugTask.remove();
    autonPIDTask.remove();
    autonOdomTask.remove();
    autonSensorsTask.remove();
}

void skills(){
    Task autonSensorsTask(sensorsTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sensors Task");
    Task autonOdomTask(odomTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
    Task autonPIDTask(controlPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PID Task");
    Task autonDebugTask(debugTerminal, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Debug Task");
	Motor intake(intakePort, false);
	ADIDigitalOut wingLeft(wingLeftPort, false);
	ADIDigitalOut wingRight(wingRightPort, false);
    controlPIDEnable = true;
    
    shoot();

    odomSetCoords(odomGlobalX, odomGlobalY, -45);
    controlSpeedCap = 80;
    controlMove(26, 1000);
    controlTurnTo(0, 1000);
    controlSpeedCap = 60;
    intake.move(120);
    controlMove(5, 100);
    intake.move(0);
    controlMove(25, 650);
    controlMove(-7, 1000);
    controlMoveTo(false, -5, 20, 1000, 1000);
    controlTurnTo(-105, 1000);
    controlPIDEnable = false;
    delay(500);
    controlSetCoords(odomGlobalX, odomGlobalY, sensorsBearing);
    controlPIDEnable = true;
    shoot(44);
    do {
        delay(20);
    } while (!cataPIDEnable);
    // controlMove(24, 1000);

    controlPIDEnable = false;
    autonDebugTask.remove();
    autonPIDTask.remove();
    autonOdomTask.remove();
    autonSensorsTask.remove();
    controlDrive(20, 20);
}