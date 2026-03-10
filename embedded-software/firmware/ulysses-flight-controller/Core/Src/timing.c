#include "stm32h5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_freertos.h"
#include "motor_drivers/servo_driver.h"
#include "motor_drivers/esc_driver.h"

static uint8_t servo_div = 0;

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
            /* 800Hz — notify controls task */
            BaseType_t woken = pdFALSE;
            vTaskNotifyGiveFromISR((TaskHandle_t)ControlsHandle, &woken);
            portYIELD_FROM_ISR(woken);
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            /* ESC: called at 800Hz, internal divider makes 400Hz effective */
            ESC_apply_pair();

            /* Servo: 800Hz / 4 = 200Hz */
            if (++servo_div >= 4) {
                servo_div = 0;
                apply_servo_pair_degrees();
            }
        }
    }
}
