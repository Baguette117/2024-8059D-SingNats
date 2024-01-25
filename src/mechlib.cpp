#include "main.h"

//externs
double targ;
bool shoot = false, manual = false;

double power, error, deriv;
bool cataPIDEnable;

void cataPID(void* ignore){
    printf("Cata PID started\n");
	Motor cata(cataPort, MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);
    Controller master(CONTROLLER_MASTER);

    while(true){
        if (shoot || master.get_digital_new_press(DIGITAL_X)){
            targ += 540;
            shoot = false;
            cataPIDEnable = true;
        } else if (master.get_digital(DIGITAL_Y)){
            cata.move(100);
            targ = cata.get_position();
            manual = false;
        } else {
            if (cataPIDEnable == true){
                error = targ - cata.get_position();
                power = error*mechlibKP;
                if (power < 20 || error < 9) cataPIDEnable = false;
                cata.move(power);
            }
        }
        // master.print(0, 0, "Cata: %f", error);
        delay(25);
    }
}