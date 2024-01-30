#include "main.h"

//externs
double targ;
bool manual = false, cataPIDEnable = true;

double power, error, deriv;

void cataPID(void* ignore){
    printf("Cata PID started\n");
	Motor cata(cataPort, MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);
    Controller master(CONTROLLER_MASTER);

    while(true){
        if (cataPIDEnable == true){
            error = targ - cata.get_position();
            power = error*mechlibKP;
            if (power < 20 || error < 9) cataPIDEnable = false;
            cata.move(power);
        } else if (manual || master.get_digital(DIGITAL_Y)){
            cata.move(100);
            targ = cata.get_position();
            manual = false;
        } else {
            cata.move(0);
        }
        delay(15);
    }
}

void shoot(int num){
    targ += 540*num;
    cataPIDEnable = true;
}