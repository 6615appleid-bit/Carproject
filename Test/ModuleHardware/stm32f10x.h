#ifndef MODULE_TEST_STM32_H
#define MODULE_TEST_STM32_H
#include "../../Libraries/CMSIS/stm32f10x.h"
#undef GPIOA
#undef GPIOB
#undef GPIOC
#undef TIM3
#undef USART2
extern GPIO_TypeDef module_gpio_a, module_gpio_b, module_gpio_c;
extern TIM_TypeDef module_tim3;
extern USART_TypeDef module_usart2;
#define GPIOA (&module_gpio_a)
#define GPIOB (&module_gpio_b)
#define GPIOC (&module_gpio_c)
#define TIM3 (&module_tim3)
#define USART2 (&module_usart2)
uint32_t Module_GetPrimask(void);
void Module_SetPrimask(uint32_t value);
void Module_DisableIrq(void);
#define __get_PRIMASK Module_GetPrimask
#define __set_PRIMASK Module_SetPrimask
#define __disable_irq Module_DisableIrq
#endif
