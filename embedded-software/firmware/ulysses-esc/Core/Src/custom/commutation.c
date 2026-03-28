#include "commutation.h"
#include "main.h"

#include "tim.h"

#define PWM_DUTY_CYCLE  60U   /* 71/94 counts = 75.5% at 170.2 kHz          */

/*
 * Channel mapping (from MSP):
 *   CH1 / CH1N  =  PA8  / PA11  =  HinB / LinB
 *   CH2 / CH2N  =  PA9  / PB0   =  HinA / LinA
 *   CH3 / CH3N  =  PA10 / PB15  =  HinC / LinC
 *
 * High side (Hin = PWM): CHx started,  Pulse = PWM_DUTY_CYCLE
 * Low  side (Lin = ON) : CHxN started, Pulse = 0
 *                        (OC ref permanently LOW -> N output permanently HIGH)
 * Floating             : both stopped
 */
void Set_Commutation_Step(uint8_t step) {
    /* Disable every output first — no shoot-through during transition */
    HAL_TIM_PWM_Stop   (&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop   (&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop   (&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    switch (step) {
        case 0: /* Step 1 — HinA = PWM,  LinB = ON */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_DUTY_CYCLE);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
            HAL_TIM_PWM_Start   (&htim1, TIM_CHANNEL_2);  /* HinA */
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);  /* LinB */
            break;

        case 1: /* Step 2 — HinA = PWM,  LinC = ON */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_DUTY_CYCLE);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
            HAL_TIM_PWM_Start   (&htim1, TIM_CHANNEL_2);  /* HinA */
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);  /* LinC */
            break;

        case 2: /* Step 3 — HinB = PWM,  LinC = ON */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_DUTY_CYCLE); /* was CHANNEL_1 */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);             /* was CHANNEL_3 */
            HAL_TIM_PWM_Start   (&htim1, TIM_CHANNEL_3);                  /* HinB */
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);                  /* LinC */
            break;

        case 3: /* Step 4 — HinB = PWM,  LinA = ON */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_DUTY_CYCLE); /* was CHANNEL_1 */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
            HAL_TIM_PWM_Start   (&htim1, TIM_CHANNEL_3);                  /* HinB */
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);                  /* LinA */
            break;

        case 4: /* Step 5 — HinC = PWM,  LinA = ON */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_DUTY_CYCLE); /* was CHANNEL_3 */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
            HAL_TIM_PWM_Start   (&htim1, TIM_CHANNEL_1);                  /* HinC */
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);                  /* LinA */
            break;

        case 5: /* Step 6 — HinC = PWM,  LinB = ON */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_DUTY_CYCLE); /* was CHANNEL_3 */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);             /* was CHANNEL_1 */
            HAL_TIM_PWM_Start   (&htim1, TIM_CHANNEL_1);                  /* HinC */
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);                  /* LinB */
            break;

        default: break;
    }

    /* TIM1 is an advanced-control timer: re-assert MOE after any Stop call  */
    __HAL_TIM_MOE_ENABLE(&htim1);
}
