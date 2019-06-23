#include <stdint.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

/* To change this, must fix clock setup in the code. */
#define MCU_HZ 80000000

static const float F_PI = 3.141592654f;

#define likely(x) __builtin_expect((x)!=0, 1)
#define unlikely(x) __builtin_expect((x)!=0, 0)


static inline void
hw_set_bit(uint32_t addr, uint32_t bitnum)
{
  uint32_t bitband_addr = 0x42000000 + (addr-0x40000000)*32 + bitnum*4;
  HWREG(bitband_addr) = 1;
}


static inline void
hw_clear_bit(uint32_t addr, uint32_t bitnum)
{
  uint32_t bitband_addr = 0x42000000 + (addr-0x40000000)*32 + bitnum*4;
  HWREG(bitband_addr) = 0;
}


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


extern uint32_t udma_control_block[256];


/* led.c */
void config_led(void);
void led_on(void);
void led_off(void);


/* nrf.c */
extern void setup_nrf(void);


/* ps2.c */
extern uint8_t ps2_button_state[18];
extern void setup_ps2(void);
extern void start_ps2_interrupts(void);
