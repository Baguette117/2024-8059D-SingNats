#include "main.h"
#include "pros/misc.h"

//externs
double targ;
bool shoot = false, manual = false;

double power, error, prevError, deriv;

void cataPID(void* ignore){
    printf("Cata PID started\n");
	Motor cata(cataPort, MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);
    Controller master(CONTROLLER_MASTER);

    while(true){
        if (shoot || master.get_digital_new_press(DIGITAL_X)){
            targ += 540;
            shoot = false;
        } else if (master.get_digital_new_press(DIGITAL_Y)){
            cata.move(100);
            targ = cata.get_position();
            manual = false;
        } else {
            error = targ - cata.get_position();
            deriv = error - prevError;
            power = error*mechlibKP + deriv*mechlibKD;
            if (power < 15) power = 0;
            cata.move(power);
            prevError = error;
        }
        // master.print(0, 0, "Cata: %f", error);
        delay(25);
    }
}