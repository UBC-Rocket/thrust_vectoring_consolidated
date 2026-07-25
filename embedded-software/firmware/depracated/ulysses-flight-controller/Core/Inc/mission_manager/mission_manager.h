#ifndef MISSION_MANAGER_H
#define MISSION_MANAGER_H

// rocket state machine state of flight

typedef enum {
    IDLE,
    E_STOP,
    RISE,
    HOVER,
    LOWER
} flight_state_t;

#endif
