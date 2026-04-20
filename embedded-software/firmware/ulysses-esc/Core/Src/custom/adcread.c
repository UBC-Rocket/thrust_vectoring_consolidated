#include "adc.h"
#include "adcread.h" 
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_adc_ex.h"

// [0] rank 1, adc channel 2 = BEMF_COM
// [1] rank 2, adc channel 1 = BEMF_A
// [2] rank 3, adc channel 3 = BEMF_B
// [3] rank 4, adc channel 4 = BEMF_C
volatile uint16_t adc_dma_buffer[4] = {0}; 

void adcread_init( void ){
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)(adc_dma_buffer), 4);
}
