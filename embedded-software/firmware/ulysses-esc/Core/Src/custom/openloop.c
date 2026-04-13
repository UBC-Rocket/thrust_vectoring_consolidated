#include "openloop.h"
#include "main.h"
#include "tim.h"

#include <stdio.h>
#include <stdint.h>

#define MIN_PERIOD          1000
#define MAX_PERIOD          10000

#define OPEN_LOOP_DUTY      0.5f

#define TIM_A_CCR   (TIM1->CCR2)
#define TIM_B_CCR   (TIM1->CCR1)
#define TIM_C_CCR   (TIM1->CCR3)

#define PHASE_HIGH(ccr)  ((ccr) = (uint32_t)(OPEN_LOOP_DUTY * (float)TIM1->ARR))
#define PHASE_LOW(ccr)   ((ccr) = 0U)

#define FLOAT_A()   (TIM1->CCER &= ~(TIM_CCER_CC2E  | TIM_CCER_CC2NE))
#define FLOAT_B()   (TIM1->CCER &= ~(TIM_CCER_CC1E  | TIM_CCER_CC1NE))
#define FLOAT_C()   (TIM1->CCER &= ~(TIM_CCER_CC3E  | TIM_CCER_CC3NE))

#define ENABLE_A()  (TIM1->CCER |=  (TIM_CCER_CC2E  | TIM_CCER_CC2NE))
#define ENABLE_B()  (TIM1->CCER |=  (TIM_CCER_CC1E  | TIM_CCER_CC1NE))
#define ENABLE_C()  (TIM1->CCER |=  (TIM_CCER_CC3E  | TIM_CCER_CC3NE))

static void commutation_init(void);
static void commutate_step(uint8_t step);

void OpenLoop(void) {
    commutation_init();
    while (1);
}

static void commutation_init(void) {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIM_Base_Start_IT(&htim15);
}

/*
 * Trapezoidal commutation table
 *   Step    A       B       C
 *      0    HIGH    LOW     FLOAT
 *      1    HIGH    FLOAT   LOW
 *      2    FLOAT   HIGH    LOW
 *      3    LOW     HIGH    FLOAT
 *      4    LOW     FLOAT   HIGH
 *      5    FLOAT   LOW     HIGH
 */
static void commutate_step(uint8_t step) {
    switch (step) {
        case 0: /* A=HIGH, B=LOW, C=FLOAT */
            FLOAT_C();
            PHASE_HIGH(TIM_A_CCR); ENABLE_A();
            PHASE_LOW(TIM_B_CCR);  ENABLE_B();
            break;

        case 1: /* A=HIGH, B=FLOAT, C=LOW */
            FLOAT_B();
            PHASE_HIGH(TIM_A_CCR); ENABLE_A();
            PHASE_LOW(TIM_C_CCR);  ENABLE_C();
            break;

        case 2: /* A=FLOAT, B=HIGH, C=LOW */
            FLOAT_A();
            PHASE_HIGH(TIM_B_CCR); ENABLE_B();
            PHASE_LOW(TIM_C_CCR);  ENABLE_C();
            break;

        case 3: /* A=LOW, B=HIGH, C=FLOAT */
            FLOAT_C();
            PHASE_HIGH(TIM_B_CCR); ENABLE_B();
            PHASE_LOW(TIM_A_CCR);  ENABLE_A();
            break;

        case 4: /* A=LOW, B=FLOAT, C=HIGH */
            FLOAT_B();
            PHASE_HIGH(TIM_C_CCR); ENABLE_C();
            PHASE_LOW(TIM_A_CCR);  ENABLE_A();
            break;

        case 5: /* A=FLOAT, B=LOW, C=HIGH */
            FLOAT_A();
            PHASE_HIGH(TIM_C_CCR); ENABLE_C();
            PHASE_LOW(TIM_B_CCR);  ENABLE_B();
            break;

        default: /* Fault — float everything immediately */
            FLOAT_A();
            FLOAT_B();
            FLOAT_C();
            break;
    }
}

static uint8_t step = 0;
static uint8_t dir_state = 0;   

void CommutationISR(void) {
    commutate_step(step);
    step = (step >= 5) ? 0U : step + 1;
    // uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim15);

    // if (arr <= MIN_PERIOD) {
    //     dir_state = 1;
    // } else if (arr >= MAX_PERIOD) {
    //     dir_state = 0;
    // }

    // if (dir_state == 0) {
    //     arr = (uint32_t)((float)arr * 0.99f);   /* period ↓  →  speed ↑ */
    // } else {
    //     arr = (uint32_t)((float)arr * 1.01f);   /* period ↑  →  speed ↓ */
    // }

    // __HAL_TIM_SET_AUTORELOAD(&htim15, arr);

    // printf("ARR: %lu\r\n", arr);
    // printf("Step: %u\r\n", step);
}
