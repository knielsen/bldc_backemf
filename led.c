#include <stdint.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

#include "led.h"


void
config_led(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
}


void
led_on(void)
{
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
}


void
led_off(void)
{
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
}
