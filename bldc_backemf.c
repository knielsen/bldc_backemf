#include <stdint.h>
#include <math.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "inc/hw_nvic.h"
#include "inc/hw_adc.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"

#include "dbg.h"
#include "led.h"


/* To change this, must fix clock setup in the code. */
#define MCU_HZ 80000000


/* Electric rotations per mechanical rotation. */
#define ELECTRIC2MECHANICAL 6

#define PWM_FREQ 50000
#define PWM_PERIOD (MCU_HZ/PWM_FREQ)

/* L6234 adds 300 ns of deadtime. */
#define DEADTIME (MCU_HZ/1000*300/1000000)


static const float F_PI = 3.141592654f;


static void motor_update(void);


static void
setup_timer_pwm(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
  ROM_GPIOPinConfigure(GPIO_PG0_T4CCP0);
  ROM_GPIOPinConfigure(GPIO_PG1_T4CCP1);
  ROM_GPIOPinConfigure(GPIO_PG2_T5CCP0);
  ROM_GPIOPinTypeTimer(GPIO_PORTG_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);
  //ROM_GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2,
  //                     GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);
  /* Set EN pins low, for off. */
  ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_3, 0);
  ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_4, 0);
  ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_5, 0);

  /*
    We use TIMER4A, TIMER4B, TIMER5A to drive IN1, IN2, IN3 with PWM.
    TIMER5B is used in dummy PWM mode just to trigger an interrupt at the
    start of every PWM period - it does not seem possible(?) to get such an
    interrupt from a PWM timer with a zero duty cycle (match=reload).
  */
  ROM_TimerConfigure(TIMER4_BASE,
                     TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM);
  ROM_TimerConfigure(TIMER5_BASE,
                     TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM);

  ROM_TimerLoadSet(TIMER4_BASE, TIMER_BOTH, PWM_PERIOD-1);
  ROM_TimerLoadSet(TIMER5_BASE, TIMER_BOTH, PWM_PERIOD-1);
  ROM_TimerMatchSet(TIMER4_BASE, TIMER_A, PWM_PERIOD-2);
  ROM_TimerMatchSet(TIMER4_BASE, TIMER_B, PWM_PERIOD-2);
  ROM_TimerMatchSet(TIMER5_BASE, TIMER_A, PWM_PERIOD-2);
  /* The dummy timer to get an interrupt can arbitrarily have 50% duty cycle. */
  ROM_TimerMatchSet(TIMER5_BASE, TIMER_B, (PWM_PERIOD-1)/2);

  /*
    Set the MRSU bit in config register, so that we can change the PWM duty
    cycle on-the-fly, and the new value will take effect at the start of the
    next period.

    And set the PLO bit, which disables legacy operation and allows a 0%
    duty cycle when match equals reload value.
  */
  HWREG(TIMER4_BASE + TIMER_O_TAMR) |= TIMER_TAMR_TAMRSU | TIMER_TAMR_TAPLO;
  HWREG(TIMER4_BASE + TIMER_O_TBMR) |= TIMER_TBMR_TBMRSU | TIMER_TBMR_TBPLO;
  HWREG(TIMER5_BASE + TIMER_O_TAMR) |= TIMER_TAMR_TAMRSU | TIMER_TAMR_TAPLO;
  HWREG(TIMER5_BASE + TIMER_O_TBMR) |= TIMER_TBMR_TBMRSU | TIMER_TBMR_TBPLO;

  ROM_IntMasterEnable();
  ROM_TimerControlEvent(TIMER4_BASE, TIMER_BOTH, TIMER_EVENT_POS_EDGE);
  ROM_TimerControlEvent(TIMER5_BASE, TIMER_BOTH, TIMER_EVENT_POS_EDGE);
  if (0) {
    /* We don't really need to enable these interrupts at the moment. */
    ROM_TimerIntEnable(TIMER4_BASE, TIMER_CAPA_EVENT|TIMER_CAPB_EVENT);
    ROM_TimerIntEnable(TIMER5_BASE, TIMER_CAPA_EVENT);
    ROM_IntEnable(INT_TIMER4A);
    ROM_IntEnable(INT_TIMER4B);
    ROM_IntEnable(INT_TIMER5A);
  }
  /* Enable the dummy PWM timer to trigger interrupts. */
  ROM_TimerIntEnable(TIMER5_BASE, TIMER_CAPB_EVENT);
  ROM_IntEnable(INT_TIMER5B);

  ROM_TimerEnable(TIMER4_BASE, TIMER_BOTH);
  ROM_TimerEnable(TIMER5_BASE, TIMER_BOTH);

  /*
    Synchronise the timers.

    We can not use wait-for-trigger, as there is an errata GPTM#04 that
    wait-for-trigger is not available for PWM mode.

    So we need to use the SYNC register.
    There is also an errata for SYNC:

      "GPTM#01 GPTMSYNC Bits Require Manual Clearing"

    Since the sync register for all timers is in timer 0, that timer must be
    enabled.
  */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  HWREG(TIMER0_BASE+TIMER_O_SYNC) |=
    (uint32_t)(TIMER_4A_SYNC|TIMER_4B_SYNC|TIMER_5A_SYNC|TIMER_5B_SYNC);
  HWREG(TIMER0_BASE+TIMER_O_SYNC) &=
    ~(uint32_t)(TIMER_4A_SYNC|TIMER_4B_SYNC|TIMER_5A_SYNC|TIMER_5B_SYNC);
}


/*
  We use ADC channels to measure the back-emf.
    PE2   phase A   AIN1   (xlat2 on POV PCB)
    PE3   phase B   AIN0   (blank2)
    PB4   phase C   AIN10  (sclk2)
    PD3   neutral   AIN4   (sin3)
*/
static void
setup_adc(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  ROM_GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_4);
  ROM_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);
  ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
  ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

  ROM_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
  /* For now, just sample phase A. */
  ROM_ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                               ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
  ROM_ADCSequenceEnable(ADC0_BASE, 3);
}

static void
adc_start(void)
{
  ROM_ADCIntClear(ADC0_BASE, 3);
  ROM_ADCProcessorTrigger(ADC0_BASE, 3);
}


static uint32_t
adc_ready(void)
{
  return ROM_ADCIntStatus(ADC0_BASE, 3, false) ? 1 : 0;
}


static uint32_t
adc_read(void)
{
  unsigned long tmp;
  ROM_ADCSequenceDataGet(ADC0_BASE, 3, &tmp);
  return tmp;
}


#define DBG_NUM_SAMPLES 12000
static volatile uint16_t dbg_adc_samples[DBG_NUM_SAMPLES];
static volatile uint32_t dbg_adc_idx = 0;

static void
dbg_add_sample(uint16_t sample)
{
  uint32_t l_idx = dbg_adc_idx;
  if (l_idx < DBG_NUM_SAMPLES) {
    dbg_adc_samples[l_idx++] = sample;
    dbg_adc_idx = l_idx;
  }
}


static void
dbg_do_sample(void)
{
  static uint32_t dbg_called_before = 0;

  if (dbg_called_before) {
    if (adc_ready())
      dbg_add_sample(adc_read());
  }
  else
    dbg_called_before = 1;

  adc_start();
}


static void
dbg_dump_samples(void)
{
  uint32_t l_idx, i;

  while ((l_idx = dbg_adc_idx) < DBG_NUM_SAMPLES)
    ;

  serial_output_str
    ("-----------------------------------------------------------------------\r\n");
  for (i = 0; i < l_idx; ++i)
    println_uint32(dbg_adc_samples[i]);

  dbg_adc_idx = 0;
}


static inline void
hw_pa_enable(void)
{
  ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_3, GPIO_PIN_3);
}


static inline void
hw_pb_enable(void)
{
  ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_4, GPIO_PIN_4);
}


static inline void
hw_pc_enable(void)
{
  ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_5, GPIO_PIN_5);
}


static inline void
hw_pa_disable(void)
{
  ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_3, 0);
}


static inline void
hw_pb_disable(void)
{
  ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_4, 0);
}


static inline void
hw_pc_disable(void)
{
  ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_5, 0);
}


static inline void
hw_pa_duty(uint32_t pwm_match_value)
{
  HWREG(TIMER4_BASE + TIMER_O_TAMATCHR) = pwm_match_value;
}


static inline void
hw_pb_duty(uint32_t pwm_match_value)
{
  HWREG(TIMER4_BASE + TIMER_O_TBMATCHR) = pwm_match_value;
}


static inline void
hw_pc_duty(uint32_t pwm_match_value)
{
  HWREG(TIMER5_BASE + TIMER_O_TAMATCHR) = pwm_match_value;
}


void
IntHandlerTimer4A(void)
{
  /* Clear the interrupt. */
  HWREG(TIMER4_BASE + TIMER_O_ICR) = TIMER_CAPA_EVENT;
}


void
IntHandlerTimer4B(void)
{
  /* Clear the interrupt. */
  HWREG(TIMER4_BASE + TIMER_O_ICR) = TIMER_CAPB_EVENT;
}


void
IntHandlerTimer5A(void)
{
  /* Clear the interrupt. */
  HWREG(TIMER5_BASE + TIMER_O_ICR) = TIMER_CAPA_EVENT;
}


void
IntHandlerTimer5B(void)
{
  /* Clear the interrupt. */
  HWREG(TIMER5_BASE + TIMER_O_ICR) = TIMER_CAPB_EVENT;

  motor_update();
}


static void
setup_systick(void)
{
  ROM_SysTickPeriodSet(0xffffff+1);
  /* Force reload. */
  HWREG(NVIC_ST_CURRENT) = 0;
  ROM_SysTickEnable();
}


static const uint32_t time_period= 0x1000000;
static inline uint32_t
get_time(void)
{
  return HWREG(NVIC_ST_CURRENT);
}


static const float damper = 0.05f;
/* ToDo: Ability to dynamically vary the voltage by changing duty cycle. */
static const uint32_t current_pwm_match_value = (PWM_PERIOD-DEADTIME) -
  (uint32_t)(1.0f * (/*damper*/0.05f*(float)(PWM_PERIOD-2*DEADTIME)));

static uint32_t pa_enabled = 0;
static uint32_t pb_enabled = 0;
static uint32_t pc_enabled = 0;

static void
setup_commute(int32_t pa, int32_t pb, int32_t pc)
{
  if (pa == 0)
  {
    pa_enabled = 0;
    hw_pa_duty(PWM_PERIOD-1);
  } else {
    pa_enabled = 1;
    if (pa < 0)
      hw_pa_duty(PWM_PERIOD-1);
    else
      hw_pa_duty(current_pwm_match_value);
  }

  if (pb == 0) {
    pb_enabled = 0;
    hw_pb_duty(PWM_PERIOD-1);
  } else {
    pb_enabled = 1;
    if (pb < 0)
      hw_pb_duty(PWM_PERIOD-1);
    else
      hw_pb_duty(current_pwm_match_value);
  }

  if (pc == 0) {
    pc_enabled = 0;
    hw_pc_duty(PWM_PERIOD-1);
  } else {
    pc_enabled = 1;
    if (pc < 0)
      hw_pc_duty(PWM_PERIOD-1);
    else
      hw_pc_duty(current_pwm_match_value);
  }
}

/*
  Setup the three phases as appropriate for the given commutation step, 0..5.

     1: Phase PWM'ed between supply voltage and GND (voltage=duty cycle).
    -1: Phase permanenty connected to GND (voltage=0).
     0: Phase disconnected (enable=0)
*/
static void
setup_commute_step(uint32_t step)
{
  switch (step) {
  case 0:
    setup_commute( 1, -1,  0);
    break;
  case 1:
    setup_commute( 1,  0, -1);
    break;
  case 2:
    setup_commute( 0,  1, -1);
    break;
  case 3:
    setup_commute(-1,  1,  0);
    break;
  case 4:
    setup_commute(-1,  0,  1);
    break;
  case 5:
    setup_commute( 0, -1,  1);
    break;
  default:
    /* NotReached */
    break;
  }
}


static void
do_enable_disable(void)
{
  /*
    First enable phases, then disable.
    This way, when switching to the next commutation step, we avoid having
    a (short) interval where two phases are unconnected, which would leave
    no route for any residual current in the windings to flow.
    (In most cycles, this will do nothing, since the enable/disable values
    did not change.)
  */
  if (pa_enabled)
    hw_pa_enable();
  if (pb_enabled)
    hw_pb_enable();
  if (pc_enabled)
    hw_pc_enable();
  if (!pa_enabled)
    hw_pa_disable();
  if (!pb_enabled)
    hw_pb_disable();
  if (!pc_enabled)
    hw_pc_disable();
}


/* Number of mechanical revolutions made by motor. */
static volatile uint32_t motor_revolutions = 0;
/* Number of ticks (PWM periods). */
static volatile uint32_t motor_tick = 0;
/* Tick counter at last commute step. */
static volatile uint32_t motor_last_commute = 0;
/* Current motor commute step (0..5). */
static volatile uint32_t motor_commute_step = 0;
/* Current speed of motor, in mechanical revolutions per second. */
static volatile float motor_cur_speed = 0.0f;
/* Speed control. */
static volatile uint32_t motor_adjusting_speed = 0;
static volatile uint32_t motor_speed_change_start = 0;
static volatile uint32_t motor_speed_change_end = 0;
static volatile float motor_speed_start = 0.0f;
static volatile float motor_speed_end = 0.0f;

/*
  Runs once at the start of every PWM period.
  Updates the commutation step when necessary.
*/
static void
motor_update()
{
  uint32_t l_motor_tick;
  float l_motor_cur_speed;
  uint32_t delta, target;

  /*
    When switching commutation step, set the new PWM period (=duty cycle)
    immedately in the timer shadow registers; this will take effect from the
    start of the next PWM period. And then early in next PWM period, put the
    appropriate enable/disable settings into effect. This way, the phases
    have the correct polarity when they become enabled.
  */
  do_enable_disable();

  l_motor_tick = motor_tick;
  l_motor_cur_speed = motor_cur_speed;

  /* Check if it is time for a new commute step. */
  delta = l_motor_tick - motor_last_commute;
  target = (uint32_t)floorf(0.5f + (float)PWM_FREQ /
                            (l_motor_cur_speed*(float)ELECTRIC2MECHANICAL));
  if (delta >= target) {
    uint32_t l_step = motor_commute_step + 1;
    if (l_step == 6) {
      l_step = 0;
      ++motor_revolutions;
    }
    setup_commute_step(l_step);
    motor_commute_step = l_step;
    motor_last_commute = l_motor_tick;

    /* Adjust speed, if appropriate. */
    if (motor_adjusting_speed) {
      uint32_t l_speed_change_start = motor_speed_change_start;
      uint32_t l_speed_change_end = motor_speed_change_end;
      uint32_t range = l_speed_change_end - l_speed_change_start;
      uint32_t sofar = l_motor_tick - l_speed_change_start;
      float frac_inc = (float)sofar/(float)range;
      float new_speed;
      float l_start = motor_speed_start;
      float l_end = motor_speed_end;
      if (frac_inc < 0.0f)
        frac_inc = 0.0f;
      else if (frac_inc > 1.0f)
        frac_inc = 1.0f;
      new_speed = l_start + frac_inc*(l_end - l_start);
      motor_cur_speed = new_speed;
      if (sofar >= range)
        motor_adjusting_speed = 0;
    }
  }

  if (!motor_adjusting_speed)
    dbg_do_sample();

  motor_tick = l_motor_tick + 1;
}


int main()
{
  uint32_t last_time;
  uint32_t led_state;

  /* Use the full 80MHz system clock. */
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                     SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  ROM_FPULazyStackingEnable();

  config_led();
  config_serial_dbg();

  /* Set interrupts to use no sub-priorities. */
  ROM_IntPriorityGroupingSet(7);
  /* Setup the priorities of the interrupts we use. */
  ROM_IntPrioritySet(INT_TIMER4A, 0 << 5);
  ROM_IntPrioritySet(INT_TIMER4B, 0 << 5);
  ROM_IntPrioritySet(INT_TIMER5A, 0 << 5);

  setup_adc();
  setup_timer_pwm();
  setup_systick();

  serial_output_str("Motor controller init done\r\n");

  /* Spin up the motor a bit. */
  motor_cur_speed = ELECTRIC2MECHANICAL*0.05f;
  motor_speed_change_start = motor_tick;
  motor_speed_change_end = motor_speed_change_start + PWM_FREQ*8;
  motor_speed_start = motor_cur_speed;
  motor_speed_end = ELECTRIC2MECHANICAL*5.0f;
  motor_adjusting_speed = 1;

  last_time = get_time();
  led_state = 0;
  for (;;) {
    uint32_t cur_time;

    if (!led_state) {
      led_on();
      led_state = 1;
    } else {
      led_off();
      led_state = 0;
    }

    do {
      cur_time = get_time();
    } while (((last_time - cur_time) & (time_period - 1)) < MCU_HZ/10);
    last_time = cur_time;

    dbg_dump_samples();
  }
}
