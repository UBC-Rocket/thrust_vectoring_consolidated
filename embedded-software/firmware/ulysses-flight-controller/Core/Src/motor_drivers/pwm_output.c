#include "motor_drivers/pwm_output.h"

#include <stddef.h>

uint32_t pwm_us_to_ticks(const pwm_output_t *pwm, uint16_t us) {
    if (pwm == NULL || pwm->timer_hz == 0U) {
        return 0U;
    }

    uint64_t ticks = (uint64_t)us * (uint64_t)pwm->timer_hz;
    ticks /= 1000000ULL;
    if (ticks > 0xFFFFFFFFULL) {
        ticks = 0xFFFFFFFFULL;
    }
    return (uint32_t)ticks;
}

uint32_t pwm_clamp_ticks(const pwm_output_t *pwm, uint32_t ticks) {
    if (pwm == NULL || pwm->period_ticks == 0U) {
        return ticks;
    }
    if (ticks >= pwm->period_ticks) {
        return pwm->period_ticks - 1U;
    }
    return ticks;
}

void pwm_set_compare(const pwm_output_t *pwm, uint32_t ticks) {
    if (pwm == NULL || pwm->htim == NULL) {
        return;
    }

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(pwm->htim);
    if (ticks > arr) {
        ticks = arr;
    }

    __HAL_TIM_SET_COMPARE(pwm->htim, pwm->channel, ticks);
}
