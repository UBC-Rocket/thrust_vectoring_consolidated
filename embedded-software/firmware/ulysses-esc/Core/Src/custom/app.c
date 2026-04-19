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

//ALL OF THIS BELOW COMMENTED TO BE MOVED TO THEIR OWN FILE
// //comm step freq defines 
// #define COMM_STEP_ARR   ((16000000U / (COMM_FREQ_HZ)) - 1U) /* TIM15 ARR */
// #define BEMF_BUFFER_SIZE 128 /*Size of the circular buffer for back emf averaging*/

// typedef struct {
//     uint16_t samples[BEMF_BUFFER_SIZE];
//     uint8_t write_index;
//     uint32_t sum; 
// } PhaseBuffer; 

// /* ring buffers for filtering 
// whne new samples arrive, subtract the oldest at write index
// write a new sample, add new to sum, advance write_index
// sum is always getting updated, just divide by buffer size for the average */
// static PhaseBuffer phase_A = {0};
// static PhaseBuffer phase_B = {0};
// static PhaseBuffer phase_C = {0};

// /* communication states 
// current commutation step (0-5) 
// each step defineds which two motor phases are active and whcih is floating */
// static uint8_t comm_step = 0; 

// //flag set by adc callback when zero crossing is detected. 
// //main loop checks this flag and advances commutation 
// // volatile because its set in an interrupting context
// volatile uint8_t zero_crossing_detected = 0;

// /**
//  * @brief Updating the ring buffer with new ADC samples
//  * 
//  * subtracts oldest sample from the running sample 
//  * writes new ssample to current index
//  * adds new sample to sum 
//  * increment write pointer (wrap around at buffer size) 
//  * 
//  * @param buf: pointer to PhaseBuffer struct 
//  * @param new_sample: new 12-bit ADC value (0-4095) tehoretically but no tests yet 
//  */
// static void update_ring_buffer(PhaseBuffer* buf, uint16_t new_sample){
//     buf->sum -= buf->samples[buf->write_index];
//     buf->samples[buf->write_index] = new_sample;
//     buf->sum += new_sample;
//     buf->write_index = (buf->write_index + 1) % BEMF_BUFFER_SIZE; 
// }

// /**
//  * @brief Get averaged back-emfvalue fro mthe ring buffer 
//  * 
//  * Returns the mean value of the last BEMF_BUFFER_SIZE samples. 
//  * Just divides precalculated sum by buffer size
//  * 
//  * @param buf: pointer to PhaseBuffer struct
//  * @return averaged adc value (0- 4095) theoretically but havent tested
//  */
// static uint16_t get_averaged_bemf(PhaseBuffer* buf){
//     return buf->sum / BEMF_BUFFER_SIZE; 
// }

// //back emf reading time! 
// /**
//  * @brief getting the back emf voltage from the floating phase.
//  * 
//  * In each step we have 3 pins, hi low and floating. in each step,
//  * we get a different phase. from commutation.c i ge tthat 
//  * 
//  *   Step 0: HinA/LinC active Phase B floats
//  *   Step 1: HinB/LinC active  Phase A floats
//  *   Step 2: HinB/LinA active  Phase C floats
//  *   Step 3: HinC/LinA active Phase B floats
//  *   Step 4: HinC/LinB active  Phase A floats
//  *   Step 5: HinA/LinB active Phase C floats
//  * 
//  * @param step: current commutation step (0-5) 
//  * @return averaged backemf votlage of floating phases. 
//  */
// static uint16_t get_bemf_for_step(uint8_t step){
//     switch (step){
//         case 0: return get_averaged_bemf(&phase_B);  /* Phase B floating */
//         case 1: return get_averaged_bemf(&phase_A);  /* Phase A floating */
//         case 2: return get_averaged_bemf(&phase_C);  /* Phase C floating */
//         case 3: return get_averaged_bemf(&phase_B);  /* Phase B floating */
//         case 4: return get_averaged_bemf(&phase_A);  /* Phase A floating */
//         case 5: return get_averaged_bemf(&phase_C);  /* Phase C floating */
//         default: return 0;
//     };
// }

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
        printf("COM: %4u    A: %4u      B: %4u      C: %4u", BEMF_COM, BEMF_A, BEMF_B, BEMF_C); 
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
