#ifndef _CONTROL_
#define _CONTROL_

#define defaultKP .18
#define defaultKD 5
#define defaultKI 8
#define defaultTurnKP .22
#define defaultTurnKD .45
#define defaultTurnKI 13
#define defaultRampingMax 2
#define defaultDistanceTolerance 25
#define defaultBearingTolerance 2
#define defaultVelocityTolerance .4
#define defaultSpeedCap 120

//control task
void controlPID(void* ignore);

//basic movement
void controlDrive(double left, double right);

//relative movements
bool controlMove(double inches, double timeout, double kp = defaultKP, double kd = defaultKD, double ki = defaultKI);
bool controlTurn(double degrees, double timeout, double kp = defaultTurnKP, double kd = defaultTurnKD, double ki = defaultTurnKI);

//absolute movements
bool controlMoveTo(bool backwards, double x, double y, double turnTimeout, double moveTimeout, double moveKP = defaultKP, double moveKD = defaultKD, double moveKI = defaultKI, double turnKP = defaultTurnKP, double turnKD = defaultTurnKD, double turnKI = defaultTurnKI);
bool controlTurnTo(double bearing, double timeout, double kp = defaultTurnKP, double kd = defaultTurnKD, double ki = defaultTurnKI);
bool controlTurnLeftTo(double bearing, double timeout, double kp = defaultTurnKP, double kd = defaultTurnKD, double ki = defaultTurnKI);
bool controlTurnRightTo(double bearing, double timeout, double kp = defaultTurnKP, double kd = defaultTurnKD, double ki = defaultTurnKI);

//other
void controlSetCoords(double x, double y, double bearing);

extern double controlSpeedCap, controlKP, controlKD, controlKI, controlTurnKP, controlTurnKD, controlTurnKI, controlRampingMax, controlPowLeft, controlPowRight, controlTargPowLeft, controlTargPowRight, controlTargLeft, controlTargRight, controlErrorLeft, controlErrorRight, controlPrevErrorLeft, controlPrevErrorRight, controlDerivLeft, controlDerivRight, controlTargBearing, controlErrorBearing, controlPrevErrorBearing, controlDerivBearing;
extern bool controlPIDEnable, controlTurnMode, controlTurnLeft, controlRunning;

#endif