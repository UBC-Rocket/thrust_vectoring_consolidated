#include "stm32h5xx_hal.h"
#include "SPI_queue.h"
#include "BMI088_accel.h"
#include "main.h"
#include "SPI_device_interactions.h"
#include "gnss_radio_master.h"

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BMI_ACC_INT_1_Pin) {
        if (!bmi088_accel_ready) {
            return;
        }
        bmi088_accel_interrupt();
    } else if (GPIO_Pin == BMI_GYRO_INT_1_Pin) {
        if (!bmi088_gyro_ready) {
            return;
        }
        bmi088_gyro_interrupt();
    } else if (GPIO_Pin == EXT_INT_2_Pin) {
        /* GNSS Radio slave asserts IRQ (active high) when it has data to push */
        gnss_radio_irq_handler();
    }
    
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    (void)GPIO_Pin;
}
