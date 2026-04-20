#include "startup.h"
#include "main.h"
#include "tim.h"
#include "app.h" 
#include "motor.h"
#include "commutation.h"

#include <stdio.h>
#include <stdint.h>

#define NUM_STEPS_PER_INCREMENTS    500U  ///making this number bigger makes the arr swich in smaller steps, aka smoother startup 
#define AVG_FREQ_HZ                 ((STARTUP_MIN_FREQ_HZ + STARTUP_MAX_FREQ_HZ) / 2)
#define TOTAL_RAMP_STEPS            (AVG_FREQ_HZ * STARTUP_RAMP_TIME_MS / 1000)
#define STEPS_PER_INCREMENT         ((TOTAL_RAMP_STEPS  + NUM_STEPS_PER_INCREMENTS - 1) / NUM_STEPS_PER_INCREMENTS)
#define FREQ_INCREMENT              ((STARTUP_MAX_FREQ_HZ - STARTUP_MIN_FREQ_HZ) / NUM_STEPS_PER_INCREMENTS)


static uint32_t current_freq = STARTUP_MIN_FREQ_HZ; 
static uint8_t commutation_step = 0;
static uint16_t steps_until_update = STEPS_PER_INCREMENT;

/** 
 * @brief start up the open loop startup sequence 
 * 
 * starts the tim1 and pwm channels through commutation_init
 * sets the current step of the motor to 0 and moves to that position 
 * configures the ARR for TIM15 directly
 * resets the CNT of TIM15 to 0 
 * starts TIM15 in inturrupt mode for callbacks 
 */
void startup_begin( void ) {
    commutation_init();                 //start TIM1 and the PWM channels 
    commutate_step(commutation_step);
    steps_until_update--;                       //adds a step here to 1, since we comm to step 0 

    //configure the TIM15 for startup minimum freq 
    uint32_t ARR = (TIM15_CLCK_HZ / STARTUP_MIN_FREQ_HZ) - 1;
    TIM15->ARR = ARR;
    TIM15->CNT = 0;

    HAL_TIM_Base_Start_IT(&htim15);
}

/** 
 * @brief updating the startup freq ramping up from min to max freqs that we set in motor.h
 * 
 */
void startup_update(MotorState motor_state) {;

    if(motor_state == STATE_CLOSED_LOOP){
        return; //do nothing if we are not in startup anymore 
    }

    commutation_step = (commutation_step >= 5) ? 0U : commutation_step + 1;
    commutate_step(commutation_step);

    if(--steps_until_update != 0){
        return; //also just get out of update if we arnt on the NUM_STEPS_PER_INCREMENTS' step. refernced in a define above 
    }

    steps_until_update = STEPS_PER_INCREMENT; 

    current_freq += FREQ_INCREMENT; 

    // update tim15 freq 
    uint32_t new_ARR = (TIM15_CLCK_HZ / current_freq) - 1; 
    TIM15->ARR = new_ARR;
    TIM15->CNT = 0; 

    if(current_freq >= STARTUP_MAX_FREQ_HZ){
        motor_set_state(STATE_CLOSED_LOOP);
        return; 
    }
    //printf("ARR: %lu\r\n", new_ARR);
} 


void startup_fixed_step( void ){
    commutation_step = (commutation_step >= 5) ? 0U : commutation_step + 1; 
    commutate_step(commutation_step);
}

