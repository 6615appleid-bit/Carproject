/* Host-only shim: retain the real SPL types/prototypes, replace MMIO bases. */
#ifndef MOTOR_TEST_STM32_H
#define MOTOR_TEST_STM32_H
#include "../../Libraries/CMSIS/stm32f10x.h"
#undef GPIOA
#undef GPIOB
#undef TIM1
extern GPIO_TypeDef test_gpio_a;
extern GPIO_TypeDef test_gpio_b;
extern TIM_TypeDef test_tim1;
#define GPIOA (&test_gpio_a)
#define GPIOB (&test_gpio_b)
#define TIM1 (&test_tim1)
#endif
