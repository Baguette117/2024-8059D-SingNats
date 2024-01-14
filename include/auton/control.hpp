#ifndef _CONTROL_
#define _CONTROL_

#define defaultKP .21
#define defaultKD 3.3
#define defaultTurnKP .35
#define defaultTurnKD .5
#define defaultRampingMax 2
#define defaultDistanceTolerance 25
#define defaultBearingTolerance 30
#define defaultVelocityTolerance -2.4
#define defaultSpeedCap 80

//control task
void controlPID(void* ignore);

//basic movement
void controlDrive(double left, double right);

//relative movements
bool controlMove(double inches, double timeout = 0, double kp = defaultKP, double kd = defaultKD);
bool controlTurn(double degrees, double timeout = 0, double kp = defaultTurnKP, double kd = defaultTurnKD);

//absolute movements
bool controlMoveTo(bool backwards, double x, double y, double turnTimeout = 0, double moveTimeout = 0, double moveKP = defaultKP, double moveKD = defaultKD, double turnKP = defaultTurnKP, double turnKD = defaultTurnKD);
bool controlTurnTo(double bearing, double timeout = 0, double kp = defaultTurnKP, double kd = defaultTurnKD);
bool controlTurnLeftTo(double bearing, double timeout = 0, double kp = defaultTurnKP, double kd = defaultTurnKD);
bool controlTurnRightTo(double bearing, double timeout = 0, double kp = defaultTurnKP, double kd = defaultTurnKD);

//other
void controlSetCoords(double x, double y, double bearing);

extern double controlSpeedCap, controlKP, controlKD, controlTurnKP, controlTurnKD, controlRampingMax, controlPowLeft, controlPowRight, controlTargPowLeft, controlTargPowRight, controlTargLeft, controlTargRight, controlErrorLeft, controlErrorRight, controlPrevErrorLeft, controlPrevErrorRight, controlDerivLeft, controlDerivRight, controlTargBearing, controlErrorBearing, controlPrevErrorBearing, controlDerivBearing;
extern bool controlPIDEnable, controlTurnMode, controlTurnLeft, controlRunning;

#endif