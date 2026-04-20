#include "app.h"
#include "main.h"
#include "motor.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_tim.h"
#include "tim.h"
#include "commutation.h"
#include "adc.h"
#include "usart.h"
#include "startup.h" 
#include "blanking.h"
#include "adcread.h"

#include <stdio.h>

// /**
//  * @brief Detecting zero crossing in back emf signal
//  * 
//  * the floating phase oscillates around 2048 mid rail for our 12bit
//  * adc with 3.3V reference(at least accroidng to google it will)
//  * 
//  * Zero crossing is just when the voltage crosses the midpoint. 
//  * this indicates that the back emf has started to reverse directions, 
//  * and we need the next commutation step to occur. 
//  * 
//  * @param step: current commutation step we are on 
//  * @return 1 if zero crossing is detected, and 0 otherwise 
//  */
// static uint8_t detect_zero_crossing(uint8_t step){
//     // if(blanking_is_active()){
//     //     return 0; 
//     // }

//     static uint16_t prev_bemf = 0; 
//     static uint8_t prev_motor_state = STATE_STARTUP;
//     uint16_t threshold = 2048;
//     uint16_t bemf = get_bemf_for_step(step);

//     if(motor_state != prev_motor_state) {
//         prev_bemf = bemf;
//         prev_motor_state = motor_state;
//         return 0;
//     }

//     if((prev_bemf < threshold && bemf >= threshold) || //rising edge
//        (prev_bemf >= threshold && bemf < threshold)) //fallingedge
//        {
//         prev_bemf = bemf;
//         return 1;
//        }

//     prev_bemf = bemf;
//     return 0; 
// }

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

void app(void) {
    startup_begin();

    while (1) {
        printf("COM: %4u    A: %4u      B: %4u      C: %4u\n", BEMF_COM, BEMF_A, BEMF_B, BEMF_C); 
        HAL_Delay(100); 
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM15) {
            CommutationISR();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){

    //check for 
    if(hadc-> Instance == ADC1){
        
    }
}
