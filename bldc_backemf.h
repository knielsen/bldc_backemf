#include <stdint.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "inc/hw_nvic.h"
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


static inline void
my_disable_timera(unsigned long timer_base)
{
  //HWREG(timer_base + TIMER_O_CTL) &= ~(uint32_t)TIMER_CTL_TAEN;
  hw_clear_bit(timer_base + TIMER_O_CTL, 0);
}


static inline void
my_disable_timerb(unsigned long timer_base)
{
  //HWREG(timer_base + TIMER_O_CTL) &= ~(uint32_t)TIMER_CTL_TBEN;
  hw_clear_bit(timer_base + TIMER_O_CTL + 1, 0);
}


static inline void
my_enable_timera(unsigned long timer_base)
{
  //HWREG(timer_base + TIMER_O_CTL) |= (uint32_t)TIMER_CTL_TAEN;
  hw_set_bit(timer_base + TIMER_O_CTL, 0);
}


static inline void
my_enable_timerb(unsigned long timer_base)
{
  //HWREG(timer_base + TIMER_O_CTL) |= (uint32_t)TIMER_CTL_TBEN;
  hw_set_bit(timer_base + TIMER_O_CTL + 1, 0);
}


static inline void
my_clear_timera_timeout_int(unsigned long timer_base)
{
  //HWREG(timer_base + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT;
  hw_set_bit(timer_base + TIMER_O_ICR, 0);
}


static inline void
my_clear_timerb_timeout_int(unsigned long timer_base)
{
  //HWREG(timer_base + TIMER_O_ICR) = TIMER_TIMB_TIMEOUT;
  hw_set_bit(timer_base + TIMER_O_ICR + 1, 0);
}


static inline uint8_t RBIT(uint8_t byte_val)
{
  uint32_t res;
  uint32_t word_val = byte_val;
  __asm ("rbit %0, %1" : "=r" (res) : "r" (word_val));
  return res>>24;
}


static inline uint32_t
get_time(void)
{
  return HWREG(NVIC_ST_CURRENT);
}


static inline uint32_t
calc_time_from_val(uint32_t start, uint32_t stop)
{
  return (start - stop) & 0xffffff;
}


static inline uint32_t
calc_time(uint32_t start)
{
  uint32_t stop = HWREG(NVIC_ST_CURRENT);
  return calc_time_from_val(start, stop);
}


static inline uint32_t
dec_time(uint32_t val, uint32_t inc)
{
  return (val - inc) & 0xffffff;
}


extern uint32_t udma_control_block[256];
extern volatile uint32_t motor_idle;

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


/* usb.c */
extern void usb_data_put(const unsigned char *buf, uint32_t size);
extern void config_usb(void);

/* bldc_backemf.c */
extern void delay_us(uint32_t us);
extern void start_motor(void);
extern void stop_motor(void);
