#ifndef GPIO_GPIO_C_
#define GPIO_GPIO_C_

#include <peripheral/gpio_pins.h>
#include <stm32g4xx_ll_bus.h>

typedef enum {
  GPIO_HIGH = 1,
  GPIO_LOW = 0,
} PinLevel;

typedef enum {
  OUTPUT = LL_GPIO_MODE_OUTPUT,
  INPUT = LL_GPIO_MODE_INPUT,
  INPUT_PULLUP = 4,
  INPUT_PULLDOWN = 5,
} PinMode;

void gpio_port_enable_clock(GPIO_TypeDef *port);

void gpio_init(GPIO_TypeDef *port, uint32_t pins, PinMode dir);

void gpio_init(GPIO_Pin &pin, PinMode dir);

void digitalWrite(GPIO_TypeDef *port, uint32_t pins, PinLevel state);

void digitalWrite(GPIO_Pin &pin, PinLevel state);

void digitalWrite(GPIO_Pin &pin, bool state);

bool digitalRead(GPIO_Pin &pin);

void togglePin(GPIO_Pin &pin);

void setPins(GPIO_TypeDef *port, uint32_t pins);

void setPin(GPIO_Pin &pin);

void resetPins(GPIO_TypeDef *port, uint32_t pins);

void resetPin(GPIO_Pin &pin);

#endif // GPIO_GPIO_C_

