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
            success = controlMove(24, 10000);
            break;

        case 1:
            success = controlTurn(180, 3000);
            break;

        case 2:
            success = controlTurn(-90, 3000);
            success &= controlMove(24, 3000);
            success &= controlMoveTo(0, 0, 3000, 3000);
            break;
    }

    delay(5000);
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

    controlMove(32, 2000);
    controlTurn(45, 1000);
    intake.move(80);
    controlMove(5, 100);
    intake.move(0);
    controlMove(25, 650);
    controlMove(-30, 2000);
    controlTurn(-45, 1250);
    wingLeft.set_value(true);
    controlMove(-20, 1000);
    wingLeft.set_value(false);
    controlMoveTo(true, 15, -15, 1000, 1000);
    wingRight.set_value(true);
    controlMoveTo(true, 23, -23, 1000, 1000);

    delay(250);
    controlPIDEnable = false;
    autonDebugTask.remove();
    autonPIDTask.remove();
    autonOdomTask.remove();
    autonSensorsTask.remove();
}