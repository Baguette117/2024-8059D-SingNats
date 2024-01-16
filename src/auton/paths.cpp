#include "control.hpp"
#include "main.h"

bool calibration(int path){
    Task autonSensorsTask(sensorsTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sensors Task");
    Task autonOdomTask(odomTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
    Task autonPIDTask(controlPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PID Task");
    Task autonDebugTask(debugTerminal, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Debug Task");
    bool success = false;
    controlPIDEnable = true;

    switch (path){
        case 0:
            success = controlMove(24, 3000);
            break;

        case 1:
            success = controlTurn(45, 3000);
            break;

        case 2:
            success = controlTurn(-90, 3000);
            success &= controlMove(24, 3000);
            success &= controlMoveTo(0, 0, 3000, 3000);
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
	Motor cata(cataPort, false);
	ADIDigitalOut wingLeft(wingLeftPort, false);
	ADIDigitalOut wingRight(wingRightPort, false);
    controlPIDEnable = true;

    controlMove(28, 3000);
    controlTurnTo(45, 1500);
    controlSpeedCap = 80;
    intake.move(80);
    controlMove(5, 100);
    intake.move(0);
    controlSpeedCap = 120;
    controlMove(25, 650);
    controlMove(-25, 1000);
    controlTurnTo(0, 1250, 3);
    wingLeft.set_value(true);
    controlMove(-25, 1000);
    wingLeft.set_value(false);
    controlMoveTo(true, 15, -15, 2000, 1000);
    // wingRight.set_value(true);
    // controlMoveTo(true, 23, -23, 1000, 1000);

    delay(250);
    controlPIDEnable = false;
    autonDebugTask.remove();
    autonPIDTask.remove();
    autonOdomTask.remove();
    autonSensorsTask.remove();
}