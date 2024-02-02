#ifndef _PATHS_
#define _PATHS_


/**
 *  \param path:
 *  1: Move 24 inches forward
 *  2: Turn 90 degrees clockwise
 *  3: Move 24 inches forward and turn 90 degrees clockwise
 *  4: Odom test
 *  returns true if the path was completed successfully
*/
bool calibration(int path);
void matchload();
void balls();
void failsafe();
void skills();

extern bool autonwl, autonwr;

#endif