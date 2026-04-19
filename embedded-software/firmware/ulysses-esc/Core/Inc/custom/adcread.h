#ifndef ADCREAD_H
#define ADCREAD_H

#include <stdint.h> 

extern volatile uint16_t adc_dma_buffer[4];

#define BEMF_COM (adc_dma_buffer[0]) //PA1, adc1_in2 for the neutral
#define BEMF_A (adc_dma_buffer[1]) //adc1_in1 -> PA0 
#define BEMF_B (adc_dma_buffer[2]) // adc2_in2 -> PA2
#define BEMF_C (adc_dma_buffer[3]) // adc3_in3 -> PA3 

void adcread_init( void );

#endif 