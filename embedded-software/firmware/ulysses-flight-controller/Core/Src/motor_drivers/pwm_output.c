#include "motor_drivers/pwm_output.h"
#include "utilities/clamp.h"

#include <stddef.h>

uint32_t pwm_us_to_ticks(const pwm_output_t *pwm, uint16_t us) {
    if (pwm == NULL || pwm->timer_hz == 0U) {
        return 0U;
    }

    uint64_t ticks = ((uint64_t)us * (uint64_t)pwm->timer_hz) / 1000000ULL;

    if (ticks > UINT32_MAX) {
        return UINT32_MAX;
    }

    return (uint32_t)ticks;
}

uint32_t pwm_clamp_ticks(const pwm_output_t *pwm, uint32_t ticks) {
    if (pwm == NULL || pwm->period_ticks == 0U) {
        return 0U;
    }

    return clamp_u32(ticks, 0U, pwm->period_ticks - 1U);
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
