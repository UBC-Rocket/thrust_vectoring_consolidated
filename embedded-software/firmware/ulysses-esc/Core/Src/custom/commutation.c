#include "commutation.h"
#include "main.h"

#include "tim.h"

//our arr is 999. ARR = (Timer freq / target freq) - 1 = (170MHz/170KHz) - 1
#define PWM_DUTY_CYCLE  900U   /* example: 71/94 counts = 75.5% at 170.2 kHz          */

/*
 * Channel mapping (from MSP):
 *   CH1 / CH1N  =  PA8  / PA11  =  HinB / LinB Phase B
 *   CH2 / CH2N  =  PA9  / PB0   =  HinA / LinA Phase A
 *   CH3 / CH3N  =  PA10 / PB15  =  HinC / LinC Phase C
 *
 *   PHASE A    CH2 / CH2N  =  PA9  / PB0   =  HinA / LinA
 *   PHASE B    CH1 / CH1N  =  PA8  / PA11  =  HinB / LinB
 *   PHASE C    CH3 / CH3N  =  PA10 / PB15  =  HinC / LinC
 *
 * High side (Hin = PWM): CHx started,  Pulse = PWM_DUTY_CYCLE
 * Low  side (Lin = ON) : CHxN started, Pulse = 0
 *                        (OC ref permanently LOW -> N output permanently HIGH)
 * Floating             : both stopped
 */
void Set_Commutation_Step(uint8_t step) {
    /* Disable every output first — no shoot-through during transition */
    htim1.Instance->CCER = 0;

    switch (step) {
        case 0: /* Step 1 — HinA = PWM,  LinB = ON */
            htim1.Instance->CCR2 = PWM_DUTY_CYCLE;                  // HinA
            htim1.Instance->CCR1 = 0;                               // LinB
            htim1.Instance->CCR3 = 0;

            htim1.Instance->CCER = (TIM_CCER_CC2E | TIM_CCER_CC1NE);// Set ccr for HinA and LinB, ignore C
            break;

        case 1: /* Step 2 — HinA = PWM,  LinC = ON */
            htim1.Instance->CCR2 = PWM_DUTY_CYCLE;                  /* HinA */
            htim1.Instance->CCR1 = 0;
            htim1.Instance->CCR3 = 0;                               /* LinC */

            htim1.Instance->CCER = (TIM_CCER_CC2E | TIM_CCER_CC3NE);
            break;

        case 2: /* Step 3 — HinB = PWM,  LinC = ON */
            htim1.Instance->CCR2 = 0;
            htim1.Instance->CCR1 = PWM_DUTY_CYCLE;                  /* HinB */
            htim1.Instance->CCR3 = 0;                               /* LinC */

            htim1.Instance->CCER = (TIM_CCER_CC1E | TIM_CCER_CC3NE);
            break;

        case 3: /* Step 4 — HinB = PWM,  LinA = ON */
            // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_DUTY_CYCLE);
            // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
            // HAL_TIM_PWM_Start   (&htim1, TIM_CHANNEL_3);        /* HinB */

            // HAL_TIM_PWM_Start   (&htim1, TIM_CHANNEL_2);
            // HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);        /* LinA */
            htim1.Instance->CCR2 = 0;
            htim1.Instance->CCR1 = PWM_DUTY_CYCLE;
            htim1.Instance->CCR3 = 0;

            htim1.Instance->CCER = (TIM_CCER_CC1E | TIM_CCER_CC2NE);
            break;

        case 4: /* Step 5 — HinC = PWM,  LinA = ON */
            // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_DUTY_CYCLE);
            // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
            // HAL_TIM_PWM_Start   (&htim1, TIM_CHANNEL_1);        /* HinC */

            // HAL_TIM_PWM_Start   (&htim1, TIM_CHANNEL_2);
            // HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);        /* LinA */
            htim1.Instance->CCR2 = 0;                               /* HinB */
            htim1.Instance->CCR1 = 0;
            htim1.Instance->CCR3 = PWM_DUTY_CYCLE;                  /* HinC */

            htim1.Instance->CCER = (TIM_CCER_CC3E | TIM_CCER_CC2NE);
            break;

        case 5: /* Step 6 — HinC = PWM,  LinB = ON */
            // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_DUTY_CYCLE);
            // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
            // HAL_TIM_PWM_Start   (&htim1, TIM_CHANNEL_1);        /* HinC */

            // HAL_TIM_PWM_Start   (&htim1, TIM_CHANNEL_3);
            // HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);        /* LinB */
            htim1.Instance->CCR2 = 0;
            htim1.Instance->CCR1 = 0;
            htim1.Instance->CCR3 = PWM_DUTY_CYCLE;

            htim1.Instance->CCER = (TIM_CCER_CC3E | TIM_CCER_CC1NE);
            break;

        default: break;
    }

    /* TIM1 is an advanced-control timer: re-assert MOE after any Stop call  */
    __HAL_TIM_MOE_ENABLE(&htim1);
}