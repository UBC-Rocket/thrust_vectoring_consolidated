#include "startup.h"
#include "tim.h"
#include "motor.h"

static MotorState motor_state = STATE_OPEN_LOOP;
// static StartupConfig startup_config =  {
//                                         STARTUP_RAMP_TIME_MS, 
//                                         STARTUP_MIN_FREQ_HZ,
//                                         STARTUP_MAX_FREQ_HZ 
//                                         };

// StartupConfig* motor_get_startup_config( void ) {
//     return &startup_config; 
// }

MotorState* motor_get_state( void ) {
    return &motor_state; 
}

void motor_set_state(MotorState new_state) {
    motor_state = new_state; 
}

void CommutationISR() {
    if(motor_state == STATE_OPEN_LOOP){
        startup_update(motor_state); 
    } else {
        startup_fixed_step();
    }

    // here we go into closed loop mode and let the adc do its work 
    // with the call back functions 

    // printf("ARR: %lu\r\n", arr);
    // printf("Step: %u\r\n", step);
}
