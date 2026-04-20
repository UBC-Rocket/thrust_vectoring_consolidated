#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"
#include "tim.h"
#include "blanking.h"
#include <stdint.h>


typedef struct { 
    uint32_t commutation_ticks;         // timer value when the commutation happened
    uint8_t active;                     //are we blanking> 
}BlankingWindow;

//create a blankingwindow object, all to 0 
static BlankingWindow blanking = { 0 };


//blanking defines 
#define BLANKING_TIME_PERCENTAGE 20

/** 
 * @brief start blanking window using TIM15 counter 
 */
[[maybe_unused]] void blanking_start( void ){
    blanking.commutation_ticks = __HAL_TIM_GET_COUNTER(&htim15); 
    blanking.active = 1; 
} //this shi was static before

/** 
 * @brief check if stil in blanking window 
 */
[[maybe_unused]] uint8_t blanking_is_active( void ){
    if(!blanking.active) {
        return 0; 
    }

    uint32_t current_ticks = __HAL_TIM_GET_COUNTER(&htim15);
    //uint32_t elapsed = current_ticks - blanking.commutation_ticks; 
    int32_t elapsed = (int32_t)(current_ticks - blanking.commutation_ticks);
    if (elapsed < 0) elapsed += __HAL_TIM_GET_AUTORELOAD(&htim15) + 1;


    uint32_t current_arr = __HAL_TIM_GET_AUTORELOAD(&htim15); 
    uint32_t blanking_time = (current_arr * BLANKING_TIME_PERCENTAGE) / 100; 


    if(elapsed > blanking_time){
        blanking.active = 0; 
        return 0; 
    }

    return 1; 
} // this one was also static before, too lazy to check 
// i just moved the files over for both of these, the maths prolly wrong too 
