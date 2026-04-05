#include "stm32h5xx_hal.h"
#include "SPI_queue.h"
#include "main.h"
#include "SPI_device_interactions.h"
#include "gnss_radio_master.h"

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    /* BMI088 EXTI routing — commented out, replaced by ICM-40609 */
    // if (GPIO_Pin == BMI_ACC_INT_1_Pin) {
    //     if (!bmi088_accel_ready) {
    //         return;
    //     }
    //     bmi088_accel_interrupt();
    // } else if (GPIO_Pin == BMI_GYRO_INT_1_Pin) {
    //     if (!bmi088_gyro_ready) {
    //         return;
    //     }
    //     bmi088_gyro_interrupt();
    // } else

    /* TODO: Replace ICM40609_INT1_Pin with actual pin from main.h once hardware is assigned */
    if (GPIO_Pin == ICM40609_INT1_Pin) {
        if (!icm40609_ready) {
            return;
        }
        icm40609_data_ready_interrupt();
    } else if (GPIO_Pin == EXT_INT_2_Pin) {
        /* GNSS Radio slave asserts IRQ (active high) when it has data to push */
        gnss_radio_irq_handler();
    }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    (void)GPIO_Pin;
}
