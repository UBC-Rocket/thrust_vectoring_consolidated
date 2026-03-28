#include "app.h"

#include "tim.h"
#include "commutation.h"

#define COMM_FREQ_HZ    170000U  /* Electrical commutation cycles/sec — adjust  */
#define COMM_STEP_ARR   ((16000000U / (COMM_FREQ_HZ * 6U)) - 1U) /* TIM15 ARR */

static uint8_t comm_step = 0;

void app(void) {
    __HAL_TIM_SET_AUTORELOAD(&htim15, COMM_STEP_ARR);
    HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
    HAL_TIM_Base_Start_IT(&htim15);
    
    while (1) {

    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM15) {
        comm_step = (comm_step + 1U) % 6U;
        Set_Commutation_Step(comm_step);
    }
}