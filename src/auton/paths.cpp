#include "control.hpp"
#include "main.h"

bool calibration(int path){
    Task autonSensorsTask(sensorsTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sensors Task");
    Task autonOdomTask(odomTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
    Task autonPIDTask(controlPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PID Task");
    Task autonDebugTask(debugTerminal, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Debug Task");
    Imu inertial(inertialPort);
    bool success = false;
    controlPIDEnable = true;

    switch (path){
        case 0:
            success = controlMove(24, 3000);
            printf("success: %d\n\n\n\n\n\n\n", success);
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