#ifndef _CONTROL_
#define _CONTROL_

#define defaultKP .165
#define defaultKD .8
#define defaultTurnKP .3
#define defaultTurnKD .3
#define defaultRampingMax 2
#define defaultDistanceTolerance 7.5
#define defaultBearingTolerance 22.5
#define defaultVelocityTolerance 2.4
#define defaultSpeedCap 80

//control task
void controlPID(void* ignore);

//basic movement
void controlDrive(double left, double right);

//relative movements
bool controlMove(double inches, double timeout = 0, double kp = defaultKP, double kd = defaultKD);
bool controlTurn(double degrees, double timeout = 0, double kp = defaultTurnKP, double kd = defaultTurnKD);

//absolute movements
bool controlMoveTo(double x, double y, double turnTimeout = 0, double moveTimeout = 0, double moveKP = defaultKP, double moveKD = defaultKD, double turnKP = defaultTurnKP, double turnKD = defaultTurnKD);
bool controlTurnTo(double bearing, double timeout = 0, double kp = defaultTurnKP, double kd = defaultTurnKD);
bool controlTurnLeftTo(double bearing, double timeout = 0, double kp = defaultTurnKP, double kd = defaultTurnKD);
bool controlTurnRightTo(double bearing, double timeout = 0, double kp = defaultTurnKP, double kd = defaultTurnKD);

//other
void controlSetCoords(double x, double y, double bearing);

extern double controlSpeedCap, controlKP, controlKD, controlTurnKP, controlTurnKD, controlRampingMax, controlPowLeft, controlPowRight, controlTargPowLeft, controlTargPowRight, controlTargLeft, controlTargRight, controlErrorLeft, controlErrorRight, controlPrevErrorLeft, controlPrevErrorRight, controlDerivLeft, controlDerivRight, controlTargBearing, controlErrorBearing, controlPrevErrorBearing, controlDerivBearing;
extern bool controlPIDEnable, controlTurnMode, controlTurnLeft, controlRunning;

#endif