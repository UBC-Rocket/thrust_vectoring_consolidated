/**
 * @file    servo_bus_gpio.h
 * @brief   UART8 servo bus GPIO: 74LVC2G241 OE (PD3).
 *
 * PE0 = UART8_RX, PE1 = UART8_TX.
 *
 * UBC Rocket, 2026
 */
#ifndef SERVO_BUS_GPIO_H
#define SERVO_BUS_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

void servo_bus_gpio_init(void);
void servo_bus_gpio_enable(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_BUS_GPIO_H */
