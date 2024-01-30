#ifndef _MECHLIB_
#define _MECHLIB_

#define mechlibKP 5

extern double targ;
extern bool manual, cataPIDEnable;
void cataPID(void* ignore);
void shoot(int num = 1);

#endif