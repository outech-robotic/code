#include "gpio.h"

void gpio_port_enable_clock(GPIO_TypeDef *port) {
#ifdef GPIOA
  if (port == GPIOA) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  }
#endif

#ifdef GPIOB
  if (port == GPIOB) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  }
#endif

#ifdef GPIOC
  if (port == GPIOC) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  }
#endif

#ifdef GPIOD
  if(port==GPIOD){
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  }
#endif

#ifdef GPIOE
  if(port==GPIOE){
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
  }
#endif

#ifdef GPIOH
  if(port==GPIOH){
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  }
#endif
}

void gpio_init(GPIO_TypeDef *port, uint32_t pins, PinMode mode) {
  gpio_port_enable_clock(port);
  LL_GPIO_InitTypeDef gpio_init;
  LL_GPIO_StructInit(&gpio_init);

  gpio_init.Pin = pins;

  bool in = mode == INPUT || mode == INPUT_PULLUP || mode == INPUT_PULLDOWN || mode == INPUT_ANALOG;
  bool analog = mode == INPUT_ANALOG;
  bool pullup = mode == INPUT_PULLUP;
  bool pulldown = mode == INPUT_PULLDOWN;

  gpio_init.Mode = analog ? LL_GPIO_MODE_ANALOG : in ? INPUT : OUTPUT;
  gpio_init.Pull = pullup ? LL_GPIO_PULL_UP : pulldown ? LL_GPIO_PULL_DOWN : LL_GPIO_PULL_NO;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

#ifdef LL_GPIO_SPEED_FREQ_VERY_HIGH
  gpio_init.Speed=LL_GPIO_SPEED_FREQ_VERY_HIGH;
#else
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
#endif
  LL_GPIO_Init(port, &gpio_init);
}

void gpio_init(GPIO_Pin &pin, PinMode mode) {
  gpio_init(pin.port, pin.pin, mode);
}

void digitalWrite(GPIO_TypeDef *port, uint32_t pins, PinLevel state) {
  port->BSRR = pins << (state ? 0 : 16);
}

void digitalWrite(GPIO_Pin &pin, PinLevel state) {
  pin.port->BSRR = (pin.pin) << (state ? 0 : 16);
}

void digitalWrite(GPIO_Pin &pin, bool state) {
  pin.port->BSRR = (pin.pin) << (state ? 0 : 16);
}

bool digitalRead(GPIO_Pin &pin) {
  return (pin.port->IDR & pin.pin);
}

void togglePin(GPIO_Pin &pin) {
  digitalWrite(pin, !digitalRead(pin));
}

void resetPins(GPIO_TypeDef *port, uint32_t pins) {
  digitalWrite(port, pins, GPIO_LOW);
}

void resetPin(GPIO_Pin &pin) {
  digitalWrite(pin, GPIO_LOW);
}

void setPins(GPIO_TypeDef *port, uint32_t pins) {
  digitalWrite(port, pins, GPIO_HIGH);
}

void setPin(GPIO_Pin &pin) {
  digitalWrite(pin, GPIO_HIGH);
}
