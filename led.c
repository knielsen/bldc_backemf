#include <stdint.h>

#include "bldc_backemf.h"

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"


void
config_led(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_AHB_BASE, GPIO_PIN_7);
}


void
led_on(void)
{
  my_gpio_write(GPIO_PORTA_AHB_BASE, GPIO_PIN_7, GPIO_PIN_7);
}


void
led_off(void)
{
  my_gpio_write(GPIO_PORTA_AHB_BASE, GPIO_PIN_7, 0);
}
