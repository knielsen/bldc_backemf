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

/* nrf.c */
extern void config_ssi_gpio(void);
