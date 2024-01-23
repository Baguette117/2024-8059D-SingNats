#include "control.hpp"
#include "main.h"
#include "odom.hpp"
#include "pros/misc.hpp"

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
            success = controlMove(12, 3000);
            controlSetCoords(12, 0, 180);
            success &= controlMoveTo(false, 0, 0, 2000, 2000);
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
    Controller master(CONTROLLER_MASTER);
    Task autonSensorsTask(sensorsTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sensors Task");
    Task autonOdomTask(odomTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
    Task autonPIDTask(controlPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PID Task");
    Task autonDebugTask(debugTerminal, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Debug Task");
	Motor intake(intakePort, false);
	Motor cata(cataPort, false);
	ADIDigitalOut wingLeft(wingLeftPort, false);
	ADIDigitalOut wingRight(wingRightPort, false);
    controlPIDEnable = true;

    controlMove(26, 3000);
    controlTurnTo(45, 1000);
    controlSpeedCap = 60;
    intake.move(80);
    controlMove(5, 100);
    intake.move(0);
    controlMove(25, 650);
    controlMove(-30, 1000);
    // controlSetCoords(0, 28, 45);
    delay(50);
    master.print(0, 0, "%f %f", odomGlobalX, odomGlobalY);
    controlSpeedCap = 120;
    controlTurnTo(0, 1250);
    controlMove(-8, 500);
    wingLeft.set_value(true);
    controlMoveTo(true, 0, 10, 0, 1000);
    controlTurnTo(-30, 500);
    wingLeft.set_value(false);
    controlMoveTo(true, 15, -16, 500, 1000);
    wingRight.set_value(true);
    controlTurnTo(-45, 500);
    controlPIDEnable = false;
    controlDrive(-20, -40);

    delay(250);
    autonDebugTask.remove();
    autonPIDTask.remove();
    autonOdomTask.remove();
    autonSensorsTask.remove();
}