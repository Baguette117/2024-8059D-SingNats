#include "main.h"

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

    shoot = true;

    controlMove(26, 1000);
    controlTurnTo(45, 1000);
    controlSpeedCap = 60;
    intake.move(80);
    controlMove(5, 100);
    intake.move(0);
    controlMove(25, 650);
    controlMove(-30, 1000);
    controlSetCoords(0, 28, 45);
    delay(50);
    master.print(0, 0, "%f %f", odomGlobalX, odomGlobalY);
    controlSpeedCap = 120;
    controlTurnTo(0, 1250);
    controlMove(-8, 500);
    wingLeft.set_value(true);
    controlMoveTo(true, 0, 10, 100, 1000);
    // controlTurnTo(-30, 500);
    wingLeft.set_value(false);
    wingRight.set_value(true);
    // controlMoveTo(true, 15, -17, 500, 1000);
    controlMoveTo(true, 16, -18, 500, 1000);
    controlPIDEnable = false;
    controlDrive(0, -15);

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
	Motor cata(cataPort, false);
	ADIDigitalOut wingLeft(wingLeftPort, false);
	ADIDigitalOut wingRight(wingRightPort, false);
    controlPIDEnable = true;

    shoot = true;

    controlMove(24, 3000);
    controlMoveTo(false, -24, 48, 1000, 1000);
    controlTurnTo(90, 1000);
    wingRight.set_value(true);
    intake.move(80);
    controlMove(5, 100);
    intake.move(0);
    controlMove(30, 900);
    controlMove(-5, 500);;

    controlPIDEnable = false;
    autonDebugTask.remove();
    autonPIDTask.remove();
    autonOdomTask.remove();
    autonSensorsTask.remove();
}