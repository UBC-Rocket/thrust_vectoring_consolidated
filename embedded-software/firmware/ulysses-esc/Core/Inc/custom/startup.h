#ifndef STARTUP_H
#define STARTUP_H

#include "motor.h"

void startup_begin( void ); 
void startup_update(MotorState motor_state); 
void startup_fixed_step( void ); 

#endif 
