#include "commutation.h"
#include "main.h"
#include "tim.h"
#include "motor.h" 

#include <stdio.h>      //printf 
#include <stdint.h>

#define TIM_A_CCR   (TIM1->CCR2)
#define TIM_B_CCR   (TIM1->CCR1)
#define TIM_C_CCR   (TIM1->CCR3)

#define TIM1_ARR 999U   //never changing, for 170KHz PWM signal

#define PHASE_HIGH(ccr)  ((ccr) = (uint32_t)(STARTUP_DUTY_CYCLE * TIM1_ARR))
#define PHASE_LOW(ccr)   ((ccr) = 0U)

#define FLOAT_A()   (TIM1->CCER &= ~(TIM_CCER_CC2E  | TIM_CCER_CC2NE))
#define FLOAT_B()   (TIM1->CCER &= ~(TIM_CCER_CC1E  | TIM_CCER_CC1NE))
#define FLOAT_C()   (TIM1->CCER &= ~(TIM_CCER_CC3E  | TIM_CCER_CC3NE))

#define ENABLE_A()  (TIM1->CCER |=  (TIM_CCER_CC2E  | TIM_CCER_CC2NE))
#define ENABLE_B()  (TIM1->CCER |=  (TIM_CCER_CC1E  | TIM_CCER_CC1NE))
#define ENABLE_C()  (TIM1->CCER |=  (TIM_CCER_CC3E  | TIM_CCER_CC3NE))

void commutation_init(void) {
    HAL_TIM_Base_Start(&htim1); 

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
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
void commutate_step(uint8_t step) {
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
