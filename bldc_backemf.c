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


/*
  Switch 1 (?): PC4
  switch 3 (start/stop motor): PA6
*/


/* To change this, must fix clock setup in the code. */
#define MCU_HZ 80000000


#define DAMPER_VALUE 0.20f

/* Electric rotations per mechanical rotation. */
#define ELECTRIC2MECHANICAL 6

#define PWM_FREQ 25000
#define PWM_PERIOD (MCU_HZ/PWM_FREQ)

/* L6234 adds 300 ns of deadtime. */
#define DEADTIME (MCU_HZ/1000*300/1000000)


static const float F_PI = 3.141592654f;


static void motor_update(void);


static void
setup_controlpanel(void)
{

  /* Switch 1 & 3 on PC4 & PA6. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOA);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  ROM_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOC);
  ROM_GPIODirModeSet(GPIO_PORTA_AHB_BASE, GPIO_PIN_6, GPIO_DIR_MODE_IN);
  ROM_GPIOPadConfigSet(GPIO_PORTA_AHB_BASE, GPIO_PIN_6,
                       GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  ROM_GPIODirModeSet(GPIO_PORTC_AHB_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN);
  ROM_GPIOPadConfigSet(GPIO_PORTC_AHB_BASE, GPIO_PIN_4,
                       GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}


static uint32_t
check_start_stop_switch(void)
{
  long start_stop_switch = my_gpio_read(GPIO_PORTA_AHB_BASE, GPIO_PIN_6);
  return (start_stop_switch == 0);
}


static void
setup_timer_pwm(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
  ROM_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOG);
  ROM_GPIOPinConfigure(GPIO_PG0_T4CCP0);
  ROM_GPIOPinConfigure(GPIO_PG1_T4CCP1);
  ROM_GPIOPinConfigure(GPIO_PG2_T5CCP0);
  ROM_GPIOPinTypeTimer(GPIO_PORTG_AHB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);
  //ROM_GPIOPadConfigSet(GPIO_PORTG_AHB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2,
  //                     GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTG_AHB_BASE, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);
  /* Set EN pins low, for off. */
  ROM_GPIOPinWrite(GPIO_PORTG_AHB_BASE, GPIO_PIN_3, 0);
  ROM_GPIOPinWrite(GPIO_PORTG_AHB_BASE, GPIO_PIN_4, 0);
  ROM_GPIOPinWrite(GPIO_PORTG_AHB_BASE, GPIO_PIN_5, 0);

  /*
    We use TIMER4A, TIMER4B, TIMER5A to drive IN1, IN2, IN3 with PWM.

    TIMER3A is used in periodic mode to trigger an interrupt at the start of
    every PWM period - it does not seem possible(?) to get such an interrupt
    from a PWM timer with a zero duty cycle (match=reload).

    TIMER3A also triggers a one-shot TIMER3B, which in turn triggers start
    of ADC measurements of the floating phase and the neutral. This way, the
    exact start time of the measurements can be controlled. The best
    measurements are obtained near the end of the duty cycle, when the current
    in the windings has had time to stabilise, and there is still voltage on
    the neutral to avoid ADC clipping a negative floating phase voltage.
  */
  ROM_TimerConfigure(TIMER4_BASE,
                     TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM);
  ROM_TimerConfigure(TIMER5_BASE,
                     TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_ONE_SHOT);
  ROM_TimerConfigure(TIMER3_BASE,
                     TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PERIODIC|TIMER_CFG_B_ONE_SHOT);

  ROM_TimerLoadSet(TIMER4_BASE, TIMER_BOTH, PWM_PERIOD-1);
  ROM_TimerLoadSet(TIMER5_BASE, TIMER_A, PWM_PERIOD-1);
  ROM_TimerMatchSet(TIMER4_BASE, TIMER_A, PWM_PERIOD-1);
  ROM_TimerMatchSet(TIMER4_BASE, TIMER_B, PWM_PERIOD-1);
  ROM_TimerMatchSet(TIMER5_BASE, TIMER_A, PWM_PERIOD-1);
  ROM_TimerLoadSet(TIMER3_BASE, TIMER_A, PWM_PERIOD-1);
  ROM_TimerLoadSet(TIMER3_BASE, TIMER_B, 120);

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

  /* Set the TIMER3B to be started from TIMER3A timeout.*/
  ROM_TimerControlWaitOnTrigger(TIMER3_BASE, TIMER_B, 1);

  ROM_IntMasterEnable();
  ROM_TimerControlEvent(TIMER4_BASE, TIMER_BOTH, TIMER_EVENT_POS_EDGE);
  ROM_TimerControlEvent(TIMER5_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  if (0) {
    /* We don't really need to enable these interrupts at the moment. */
    ROM_TimerIntEnable(TIMER4_BASE, TIMER_CAPA_EVENT|TIMER_CAPB_EVENT);
    ROM_TimerIntEnable(TIMER5_BASE, TIMER_CAPA_EVENT);
    ROM_IntEnable(INT_TIMER4A);
    ROM_IntEnable(INT_TIMER4B);
    ROM_IntEnable(INT_TIMER5A);
  }
  /* Enable the periodic timer to trigger interrupts. */
  ROM_TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
  ROM_IntEnable(INT_TIMER3A);
  /* Let the one-shot timer trigger start of ADC measurements. */
  ROM_TimerControlTrigger(TIMER3_BASE, TIMER_B, 1);
  /*
    Enable interrupts for the one-shot timer.
    This interrupt is used to re-enable the timer to be triggered again from
    timer 3A.
  */
  ROM_TimerIntEnable(TIMER3_BASE, TIMER_TIMB_TIMEOUT);
  ROM_IntEnable(INT_TIMER3B);

  ROM_TimerEnable(TIMER4_BASE, TIMER_BOTH);
  ROM_TimerEnable(TIMER5_BASE, TIMER_A);
  ROM_TimerEnable(TIMER3_BASE, TIMER_BOTH);

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
    (uint32_t)(TIMER_4A_SYNC|TIMER_4B_SYNC|TIMER_5A_SYNC|TIMER_3A_SYNC);
  HWREG(TIMER0_BASE+TIMER_O_SYNC) &=
    ~(uint32_t)(TIMER_4A_SYNC|TIMER_4B_SYNC|TIMER_5A_SYNC|TIMER_3A_SYNC);
}


#define ADC_SEQUENCER 0
#define ADC_SEQUENCER_FIFO ADC_O_SSFIFO0
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
  uint32_t i;

  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOB);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  ROM_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOD);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  ROM_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOE);
  ROM_GPIOPinTypeADC(GPIO_PORTB_AHB_BASE, GPIO_PIN_4);
  ROM_GPIOPinTypeADC(GPIO_PORTD_AHB_BASE, GPIO_PIN_3);
  ROM_GPIOPinTypeADC(GPIO_PORTE_AHB_BASE, GPIO_PIN_2);
  ROM_GPIOPinTypeADC(GPIO_PORTE_AHB_BASE, GPIO_PIN_3);
  /* Set 1 Msamples/second speed. */
  HWREG(ADC0_BASE + ADC_O_PC) = 0x7;
  HWREG(ADC1_BASE + ADC_O_PC) = 0x7;

  ROM_ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCER, ADC_TRIGGER_TIMER, 0);
  /* Sample the phase 8 times. */
  for (i = 0; i < 8; ++i)
    ROM_ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCER, i,
                                 ADC_CTL_CH1 |
                                 (i==7 ? (ADC_CTL_IE | ADC_CTL_END) : 0));
  ROM_ADCIntEnable(ADC0_BASE, ADC_SEQUENCER);
  ROM_IntEnable(INT_ADC0SS0);
  ROM_ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCER);

  /* Setup ADC1 for sampling the neutral point. */
  ROM_ADCSequenceConfigure(ADC1_BASE, ADC_SEQUENCER, ADC_TRIGGER_TIMER, 0);
  /* Sample neutral phase 8 times. */
  for (i = 0; i < 8; ++i)
    ROM_ADCSequenceStepConfigure(ADC1_BASE, ADC_SEQUENCER, i,
                                 ADC_CTL_CH4 |
                                 (i==7 ? ( ADC_CTL_IE | ADC_CTL_END) : 0));
  ROM_ADCIntEnable(ADC1_BASE, ADC_SEQUENCER);
  ROM_IntEnable(INT_ADC1SS0);
  ROM_ADCSequenceEnable(ADC1_BASE, ADC_SEQUENCER);
}

static void
adc_start(void)
{
  /*
    For efficiency, let's just assume the ADC is done, don't check it.
    So we don't need to clear the completion flag.
    One PWM cycle of 50 kHz is plenty of time to do an ADC measurement.

    ROM_ADCIntClear(ADC0_BASE, 3);
    ROM_ADCIntClear(ADC1_BASE, 3);
  */
  // ROM_ADCProcessorTrigger(ADC0_BASE, ADC_SEQUENCER);
  HWREG(ADC0_BASE + ADC_O_PSSI) |= (1 << ADC_SEQUENCER);
  // ROM_ADCProcessorTrigger(ADC1_BASE, ADC_SEQUENCER);
  HWREG(ADC1_BASE + ADC_O_PSSI) |= (1 << ADC_SEQUENCER);
}


static uint32_t
adc_ready(void)
{
  return ROM_ADCIntStatus(ADC0_BASE, ADC_SEQUENCER, false) &&
    ROM_ADCIntStatus(ADC1_BASE, ADC_SEQUENCER, false) ?
    1 : 0;
}


static inline int16_t
adc_read(void)
{
  unsigned long phase, neutral;
  // ROM_ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCER, &phase);
  phase = HWREG(ADC0_BASE + ADC_SEQUENCER_FIFO);
  // ROM_ADCSequenceDataGet(ADC1_BASE, ADC_SEQUENCER, &neutral);
  neutral = HWREG(ADC1_BASE + ADC_SEQUENCER_FIFO);
  return (int16_t)phase - (int16_t)neutral;
}


#define DBG_NUM_SAMPLES 12000
static volatile int16_t dbg_adc_samples[DBG_NUM_SAMPLES];
static volatile uint32_t dbg_adc_idx = 0;

static void
dbg_add_sample(int16_t sample)
{
  uint32_t l_idx = dbg_adc_idx;
  if (l_idx < DBG_NUM_SAMPLES) {
    dbg_adc_samples[l_idx++] = sample;
    dbg_adc_idx = l_idx;
  }
}


__attribute__ ((unused))
static void
dbg_add_samples(uint32_t count)
{
  uint32_t l_idx = dbg_adc_idx;
  while (count > 0) {
    int16_t sample;

    sample = adc_read();
    --count;
    if (l_idx < DBG_NUM_SAMPLES)
      dbg_adc_samples[l_idx++] = sample;
  }
  dbg_adc_idx = l_idx;
}


static void
dbg_add_samples_buf(uint16_t *phases, uint16_t *neutrals, uint32_t count)
{
  uint32_t l_idx = dbg_adc_idx;
  while (count > 0 && l_idx < DBG_NUM_SAMPLES) {
    dbg_adc_samples[l_idx++] = (int16_t)*phases++ - (int16_t)*neutrals++;
    --count;
  }
  dbg_adc_idx = l_idx;
}


__attribute__ ((unused))
static void
dbg_do_sample(void)
{
  static uint32_t dbg_called_before = 0;

  if (dbg_called_before) {
    if (adc_ready())
      dbg_add_sample(adc_read());
    else
      for (;;) ;
  }
  else
    dbg_called_before = 1;

  adc_start();
}


static void
dbg_dump_samples(void)
{
  uint32_t l_idx, i;

  if ((l_idx = dbg_adc_idx) < DBG_NUM_SAMPLES)
    return;

  serial_output_str
    ("-----------------------------------------------------------------------\r\n");
  for (i = 0; i < l_idx; ++i) {
    int32_t diff = dbg_adc_samples[i];
    /*
      Resistor ladder with 10k and 2.2k ohm.
      V = (10k+2.2k)/2.2k * V_measured
      V_measured = adc_val/4095*3.3V
    */
    float voltage = ((10.0f+2.2f)/2.2f*3.3f/4095.0f)*(float)diff;
    println_float(voltage, 2, 5);
  }

  dbg_adc_idx = 0;
}


static inline void
hw_pa_enable(void)
{
  my_gpio_write(GPIO_PORTG_AHB_BASE, GPIO_PIN_3, GPIO_PIN_3);
}


static inline void
hw_pb_enable(void)
{
  my_gpio_write(GPIO_PORTG_AHB_BASE, GPIO_PIN_4, GPIO_PIN_4);
}


static inline void
hw_pc_enable(void)
{
  my_gpio_write(GPIO_PORTG_AHB_BASE, GPIO_PIN_5, GPIO_PIN_5);
}


static inline void
hw_pa_disable(void)
{
  my_gpio_write(GPIO_PORTG_AHB_BASE, GPIO_PIN_3, 0);
}


static inline void
hw_pb_disable(void)
{
  my_gpio_write(GPIO_PORTG_AHB_BASE, GPIO_PIN_4, 0);
}


static inline void
hw_pc_disable(void)
{
  my_gpio_write(GPIO_PORTG_AHB_BASE, GPIO_PIN_5, 0);
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
IntHandlerTimer3A(void)
{
  /* Clear the interrupt. */
  HWREG(TIMER3_BASE + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT;

  motor_update();
}


static float motor_damper = 0;
/* ToDo: Ability to dynamically vary the voltage by changing duty cycle. */
static uint32_t current_pwm_match_value = (PWM_PERIOD-DEADTIME) -
  (uint32_t)(1.0f * (0.0f*(float)(PWM_PERIOD-2*DEADTIME)));

void
IntHandlerTimer3B(void)
{
  uint32_t adc_delay;

  /* Clear the interrupt. */
  HWREG(TIMER3_BASE + TIMER_O_ICR) = TIMER_TIMB_TIMEOUT;

  /*
    Set the delay of start of next ADC sampling.
    We want to sample near the end of the next duty cycle.
    2 microseconds from the end of the duty cycle seems to work well,
    as long as the duty cycle is > 20%. Presumably, there is a bit of noise
    injected into the ADC circuit when the next commute step starts, and
    with 2 microseconds head start and one microsecond needed for ADC
    conversion, things are better?

    Currently, we are taking 8 samples, mostly for debugging purposes.
    So we start 3 ADC samples earlier, so that the fourth sample is
    the one we want to use for detecting zero-crossing.
  */
  adc_delay = PWM_PERIOD
    - 2*MCU_HZ/1000000                  // Two sample before end of duty cycle
    - 3*MCU_HZ/1000000                  // Three more samples back, for debug
    - current_pwm_match_value;          // ... relative to duty cycle end
  if (adc_delay >= (uint32_t)0x80000000)
    adc_delay = 5;                      // underflow
  else if (adc_delay > PWM_PERIOD-10*MCU_HZ/1000000)
    adc_delay = PWM_PERIOD-10*MCU_HZ/1000000;  // Not too late
  HWREG(TIMER3_BASE + TIMER_O_TBILR) = adc_delay;

  /* Re-enable the timer for another triggered one-shot run. */
  HWREG(TIMER3_BASE + TIMER_O_CTL) |= TIMER_CTL_TBEN;
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


static uint32_t pa_enabled = 0;
static uint32_t pb_enabled = 0;
static uint32_t pc_enabled = 0;

static uint16_t adc_phase_samples[8];
static uint16_t adc_neutral_samples[8];
static uint32_t adc_done_counter = 0;
/* Flag set when we start dumping samples to dbg. */
static uint32_t motor_adc_dbg = 0;

static void
dbg_save_samples(void)
{
  uint32_t l_done_count = adc_done_counter;
  ++l_done_count;
  if (l_done_count == 2) {
    if (motor_adc_dbg)
      dbg_add_samples_buf(adc_phase_samples, adc_neutral_samples, 8);
    adc_done_counter = 0;
  }
  else
    adc_done_counter = l_done_count;
}

static int16_t
adc_current_backemf(void)
{
  /*
    The timing of the ADC is so that the fourth sample is taken 2 microseconds
    before the end of the duty cycle, which seems to give good results. See
    IntHandlerTimer3B().
  */
  return (int16_t)adc_phase_samples[3] - (int16_t)adc_neutral_samples[3];
}


void
IntHandlerADC0Seq0(void)
{
  int i;

  /* Clear the interrupt. */
  HWREG(ADC0_BASE + ADC_O_ISC) = 1 << ADC_SEQUENCER;

  for (i = 0; i < 8; ++i) {
#if 0
    if (HWREG(ADC0_BASE+ADC_O_SSFSTAT0) & ADC_SSFSTAT0_EMPTY) {
      serial_output_str("Fifo empty ADC0\n");
      serial_output_hexbyte(HWREG(ADC0_BASE+ADC_O_SSFSTAT0));
      println_uint32(i);
      for (;;);
    }
#endif
    adc_phase_samples[i] = HWREG(ADC0_BASE + ADC_SEQUENCER_FIFO);
  }

  /*
    Configure which phase to measure next PWM cycle.
    The timer interrupt that switches which phase is the floating one runs at
    a higher priority than this one, and it triggers at the same time as the
    ADC starts measuring. So we are guaranteed at this point that the values
    will be updated (and not change while we access them).
  */

  /*
    Disable the sequencer. The datasheet recommends, but does not require
    disabling before reconfiguring. We shouldn't risk spurious triggering
    here while reconfiguring, because the trigger will not happen until the
    next PWM cycle (and if we slip into that, we are in any case in trouble,
    using too much time to maintain real-time motor commutation). So for now,
    let's try without sequencer disable/enable around reconfiguration.
  */
  //HWREG(ADC0_BASE + ADC_O_ACTSS &= ~(uint32_t)(1 << ADC_SEQUENCER);

  /*
    Reconfigure the sequence to sample the phase that will be floating next
    PWM cycle.
    Phases A, B, C are on ADC channels 1, 0, 10, respectively.
  */
  if (!pa_enabled)
    HWREG(ADC0_BASE + ADC_O_SSMUX0) = 1 * (uint32_t)0x11111111;
  else if (!pb_enabled)
    HWREG(ADC0_BASE + ADC_O_SSMUX0) = 0 * (uint32_t)0x11111111;
  else
    HWREG(ADC0_BASE + ADC_O_SSMUX0) = 10 * (uint32_t)0x11111111;

  /* Re-enable sequencer, optimised away for now. */
  //HWREG(ADC0_BASE + ADC_O_ACTSS |= (uint32_t)(1 << ADC_SEQUENCER);

  dbg_save_samples();
}


void
IntHandlerADC1Seq0(void)
{
  int i;

  /* Clear the interrupt. */
  HWREG(ADC1_BASE + ADC_O_ISC) = 1 << ADC_SEQUENCER;

  for (i = 0; i < 8; ++i) {
#if 0
    if (HWREG(ADC1_BASE+ADC_O_SSFSTAT0) & ADC_SSFSTAT0_EMPTY) {
      serial_output_str("Fifo empty ADC0\n");
      serial_output_hexbyte(HWREG(ADC1_BASE+ADC_O_SSFSTAT0));
      println_uint32(i);
      for (;;);
    }
#endif
    adc_neutral_samples[i] = HWREG(ADC1_BASE + ADC_SEQUENCER_FIFO);
  }

  dbg_save_samples();
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


static void
setup_commute(int32_t pa, int32_t pb, int32_t pc)
{
  if (pa == 0) {
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
/* Duration, in motor ticks, of last commute step. */
static uint32_t motor_last_commute_duration = 0;
/* Target duration of this commute step, or 0 if not yet known. */
static uint32_t motor_commute_target = 0;
/* Current motor commute step (0..5). */
static volatile uint32_t motor_commute_step = 0;
/* Current speed of motor, in electric revolutions per second. */
static volatile float motor_cur_speed = 0.0f;
/* Speed control. */
static volatile uint32_t motor_spinning_up = 0;
static volatile uint32_t motor_speed_change_start = 0;
static volatile uint32_t motor_speed_change_end = 0;
static volatile float motor_speed_start = 0.0f;
static volatile float motor_speed_end = 0.0f;

static volatile uint32_t motor_idle = 1;
static volatile uint32_t motor_spinning_down = 0;
static volatile uint32_t motor_spin_down_start = 0;
static volatile uint32_t motor_spin_down_end = 0;
static volatile uint32_t motor_spin_down_idle = 0;
static volatile float motor_spin_down_initial_damper = 0;

/* Set the value of motor current speed, in electric RPS. */
static void
motor_set_current_speed(float speed)
{
  /* The commutation algorithm currently does not work with zero speed. */
  if (speed < 0.01f)
    speed = 0.01f;
  motor_cur_speed = speed;
  motor_commute_target =
    (uint32_t)floorf(0.5f + (float)PWM_FREQ / (speed*6.0f));
}


static void
motor_set_commute_target(uint32_t target)
{
  motor_commute_target = target;
  motor_cur_speed = ((float)PWM_FREQ/6.0f) / (float)target;
}


static void
motor_set_damper(float damper)
{
  if (damper < 0.0f)
    damper = 0.0f;
  else if (damper > 1.0f)
    damper = 1.0f;
  motor_damper = damper;
  current_pwm_match_value =
    (PWM_PERIOD-DEADTIME) - (uint32_t)(damper*(float)(PWM_PERIOD-2*DEADTIME));
}


static void
motor_adjust_damper(uint32_t l_motor_tick, uint32_t l_step)
{
  if (motor_spinning_down) {
    float new_damper;
    uint32_t l_spindown_start = motor_spin_down_start;
    uint32_t l_spindown_end = motor_spin_down_end;
    uint32_t range = l_spindown_end - l_spindown_start;
    uint32_t sofar = l_motor_tick - l_spindown_start;
    float frac = 1.0f - (float)sofar/(float)range;

    if (frac < 0.0f)
      frac = 0.0f;
    else if (frac > 1.0f)
      frac = 1.0f;
    new_damper = frac * motor_spin_down_initial_damper;
    motor_set_damper(new_damper);
    setup_commute_step(l_step);
    if (sofar >= (motor_spin_down_idle - l_spindown_start)) {
      motor_spinning_down = 0;
      motor_idle = 1;
      motor_adc_dbg = 0;
    }
  }
}


static void
open_loop_adjust_speed(uint32_t l_motor_tick, uint32_t l_step)
{
  /* Adjust speed as appropriate in open loop operation. */
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
  motor_set_current_speed(new_speed);
  /* Once target is reached, go to closed-loop operation. */
  if (sofar >= range && l_step == 0)
    motor_spinning_up = 0;
}


static void
closed_loop_detect_zero_crossing(uint32_t delta, uint32_t l_step)
{
  int16_t backemf = adc_current_backemf();
  uint32_t l_target;

  /*
    In even (0,2,4) commute steps, we are looking for downwards zero crossing.
    In odd (1,3,5), for upwards.

    ToDo: Probably need some filtering here.
  */
  if ((!(l_step & 1) && backemf < 0) || ((l_step & 1) && backemf > 0)) {
    /*
      Zero-crossing detected. Set next commute target to estimated 30
      electrical degrees (1/2 of last commute period).
    */
    l_target = delta + motor_last_commute_duration/2;
    if (l_target < 10)
      l_target = 10;
    else if (l_target > 2000)
      l_target = 2000;
    motor_set_commute_target(l_target);
  }
}


/*
  Runs once at the start of every PWM period.
  Updates the commutation step when necessary.
*/
static void
motor_update()
{
  uint32_t l_motor_tick;
  uint32_t delta, l_target;
  uint32_t l_spinning_up;
  uint32_t l_step;

  /*
    ADC measurements of the phase and the neutral are started by timer 3B
    (triggered from timer 3A) at a controlled delay from the start of this
    interrupt routine.

    The measurements will be collected by the ADC interrupts when done.

    The ADC interrupts are lower priority than the PWM timer interrupts, so we
    can safely read the values stored during the previous PWM cycle here,
    without risk that they will be overwritten midway by the new, ongoing
    measurements.
  */

  /*
    When switching commutation step, set the new PWM period (=duty cycle)
    immedately in the timer shadow registers; this will take effect from the
    start of the next PWM period. And then early in next PWM period, put the
    appropriate enable/disable settings into effect. This way, the phases
    have the correct polarity when they become enabled.
  */
  do_enable_disable();

  l_motor_tick = motor_tick;

  if (!motor_idle) {
    l_spinning_up = motor_spinning_up;
    l_step = motor_commute_step;

    /* Check if it is time for a new commute step. */
    delta = l_motor_tick - motor_last_commute;
    l_target = motor_commute_target;
    if (l_target && delta >= l_target) {
      ++l_step;
      if (l_step == 6) {
        l_step = 0;
        ++motor_revolutions;
      }
      setup_commute_step(l_step);
      motor_commute_step = l_step;
      motor_last_commute = l_motor_tick;
      motor_last_commute_duration = delta;

      if (l_spinning_up) {
        /* In open-loop operation we adjust speed at every commute step. */
        open_loop_adjust_speed(l_motor_tick, l_step);
      } else {
        /*
          In closed loop operation, measure back-EMF zero-crossing to find when
          to perform the next commutation step.
        */
        motor_commute_target = 0;

        /*
          Start dumping samples for debug once we have reached target speed and
          enter closed-loop operation.
        */
        motor_adc_dbg = 1;
      }
    }

    /* Back-EMF zero-crossing detection in closed loop operation. */
    if (!l_spinning_up && !l_target && delta >= 8) {
      closed_loop_detect_zero_crossing(delta, l_step);

      /*
        ToDo: Should we be able to spin down also in the open-loop state?
        But then we need to make sure that the two states do not conflict in
        strange ways.
      */
      motor_adjust_damper(l_motor_tick, l_step);
    }
  }

  motor_tick = l_motor_tick + 1;
}


static void
start_open_loop(float start_mech_rps, float end_mech_rps, float duration_sec)
{
  if (!motor_idle)
    return;
  if (motor_spinning_up || motor_spinning_down)
    return;
  motor_set_damper(DAMPER_VALUE);
  motor_set_current_speed(ELECTRIC2MECHANICAL*start_mech_rps);
  motor_speed_change_start = motor_tick;
  motor_speed_change_end =
    motor_speed_change_start + (uint32_t)(PWM_FREQ*duration_sec + 0.5f);
  motor_speed_start = motor_cur_speed;
  motor_speed_end = ELECTRIC2MECHANICAL*end_mech_rps;
  motor_spinning_up = 1;
  motor_idle = 0;
}


static void
start_spin_down(void)
{
  if (motor_spinning_down || motor_idle || motor_spinning_up)
    return;
  motor_spin_down_start = motor_tick;
  motor_spin_down_initial_damper = motor_damper;
  /*
    Spin down time of two seconds. After that, wait another 10 seconds for
    the motor to stop on its own - currently, no active breaking.
  */
  motor_spin_down_end = motor_tick + PWM_FREQ*2;
  motor_spin_down_idle = motor_spin_down_end + PWM_FREQ*10;
  motor_spinning_down = 1;
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
  ROM_IntPrioritySet(INT_TIMER3A, 0 << 5);
  ROM_IntPrioritySet(INT_TIMER3B, 2 << 5);
  ROM_IntPrioritySet(INT_ADC0SS0, 1 << 5);
  ROM_IntPrioritySet(INT_ADC1SS0, 1 << 5);

  setup_controlpanel();
  setup_adc();
  motor_idle = 1;
  motor_set_damper(0.0f);
  setup_timer_pwm();
  setup_systick();

  serial_output_str("Motor controller init done\r\n");

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

    serial_output_str("Speed: ");
    println_float(motor_cur_speed*(1.0f/(float)ELECTRIC2MECHANICAL), 2, 2);
    if (check_start_stop_switch()) {
      if (motor_idle)
        /* Spin up the motor a bit. */
        start_open_loop(0.05f, 3.0f, 3.0f);
    } else {
      start_spin_down();
    }

    dbg_dump_samples();
  }
}
