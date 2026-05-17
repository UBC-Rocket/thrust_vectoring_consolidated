#include "stm32h5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_freertos.h"
#include "motor_drivers/servo_driver.h"
#include "motor_drivers/esc_driver.h"
#include "sensors_init.h"
#include "spi_drivers/ms5611_poller.h"
#include "spi_drivers/ms5607_poller.h"

static uint8_t esc_div = 0;
static uint8_t servo_div = 0;

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
            /* 800 Hz — notify controls task */
            BaseType_t woken = pdFALSE;
            vTaskNotifyGiveFromISR((TaskHandle_t)ControlsHandle, &woken);
            portYIELD_FROM_ISR(woken);
        } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            /* 800 Hz base — ESC 400 Hz, servo 200 Hz, baro pollers 800 Hz */

            /* ESC: 800 Hz / 2 = 400 Hz */
            if (++esc_div >= 2) {
                esc_div = 0;
                esc_pair_apply();
            }

            /* Servo: 800 Hz / 4 = 200 Hz */
            if (++servo_div >= 4) {
                servo_div = 0;
                apply_servo_pair_degrees();
            }

            /* Baro pollers — tick every call (state machine no-ops are trivial,
             * and 200 Hz baro output requires ~800 Hz tick to traverse 5 states) */
            ms5611_poller_tick(sensors_get_baro_poller());
            ms5607_poller_tick(sensors_get_baro2_poller());
        }
    }
}
