#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h> 

#define MOTOR_KV                2700U
#define MOTOR_POLE_PAIRS        7U
#define SUPPLY_VOLTAGE_V        20U
#define COMMUTATION_STEP_COUNT  6
#define STARTUP_DUTY_CYCLE      0.5 //duty cycle for the startup sequence, determines the max speed 
#define TARGET_RPM              (MOTOR_KV * SUPPLY_VOLTAGE_V * STARTUP_DUTY_CYCLE)

#define STARTUP_RAMP_TIME_MS    2000           // how long initalization speed takes (milliseconds) reccomended max, may end shorter
#define STARTUP_MIN_FREQ_HZ     100             // min freq we run at 
#define STARTUP_MAX_FREQ_HZ     3000U
                                //(TARGET_RPM * MOTOR_POLE_PAIRS * COMMUTATION_STEP_COUNT) / 60  /* Electrical commutation cycles/sec — adjust  */

#define SYSCLOCK_HZ             170000000U
#define TIM15_PRSCLR            170 - 1
#define TIM15_CLCK_HZ           1000000 //(170000000 / (TIM15_PRSCLR + 1))

typedef enum { 
    STATE_OPEN_LOOP,           // Using timer to force commutation 
    STATE_CLOSED_LOOP,        // Using zero-crossing to trigger commutation 
} MotorState; 

// typedef struct {
//     uint32_t ramp_time_ms;
//     uint32_t min_freq_hz;
//     uint32_t max_freq_hz;
// } StartupConfig;
//
// StartupConfig* motor_get_startup_config(void); 


MotorState* motor_get_state( void ); 
void motor_set_state(MotorState new_state);
void CommutationISR( void );

#endif 
