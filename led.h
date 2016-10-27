#include <stdint.h>

#include "driverlib/gpio.h"


static inline uint32_t
my_gpio_read(unsigned long gpio_base, uint32_t bits)
{
  return HWREG(gpio_base + GPIO_O_DATA + (bits << 2));
}


static inline void
my_gpio_write(unsigned long gpio_base, uint32_t bits, uint32_t val)
{
  HWREG(gpio_base + GPIO_O_DATA + (bits << 2)) = val;
}

void config_led(void);
void led_on(void);
void led_off(void);
