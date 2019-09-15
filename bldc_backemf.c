#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "bldc_backemf.h"
#include "dbg.h"
#include "ps2.h"
#include "nrf.h"
#include "usb.h"

#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "inc/hw_nvic.h"
#include "inc/hw_adc.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"


/*
  nRF24L01 pinout:

  Tx:
    PF2  SCK        GND *1 2. VCC
    PF3  CSN        PB3 .3 4. PF3
    PF0  MISO       PF2 .5 6. PF1
    PF1  MOSI       PF0 .7 8. PB0
    PB0  IRQ
    PB3  CE

  L6234 motor controller pinout:

    IN1   PG0   (timer t4ccp0)
    IN2   PG1   (timer t4ccp1)
    IN3   PG2   (timer t5ccp0)
    EN1   PG3
    EN2   PG4
    EN3   PG5

  ADC channels to measure the back-emf:

    PE2   phase A   AIN1
    PE3   phase B   AIN0
    PB4   phase C   AIN10
    PD3   neutral   AIN4

  Switches and buttons pintout:

    PC4   Switch 1
    PC7   Switch 2
    PA6   Switch 3  (start/stop motor)

  Playstation 2 / DualShock controller:
    PA2   PS2 SCLK
    PA3   PS2 controller 2 "attention" (slave select)
    PA4   PS2 data
    PA5   PS2 cmd
    PC5   PS2 ack
    PC6   PS2 controller 1 "attention"
*/


/*
  Peripheral interrupt usage:

    gpiob         nRF24L01+ interrupt pin

    (timer1a)     Not used for interrupt, but to trigger dma to ssi0 Tx
    timer1b       PS2 controller periodic poll
    timer2a       nRF async delay timer
    timer3a       BLDC interrupt at the start of PWM period
    timer3b       BLDC trigger ADC measurements

    timer4a  \
    timer4b  +--  BLDC motor pwm
    timer5a  /

    adc0          BLDC back-emf measurement of phases
    adc1          BLDC back-emf measurement of neutral

    ssi0          PS2 controller readout
    ssi1          nRF24L01+ access
    usb0
*/

#define DAMPER_VALUE 0.25f

/* Electric rotations per mechanical rotation. */
#define ELECTRIC2MECHANICAL 6

#define SUPPLY_VOLTAGE 12.04f

#define PWM_FREQ 25000
#define PWM_PERIOD (MCU_HZ/PWM_FREQ)

/* L6234 adds 300 ns of deadtime. */
#define DEADTIME (MCU_HZ/1000*300/1000000)

//#define BIAS_VERBOSE

/* Control block for DMA. */
uint32_t udma_control_block[256] __attribute__ ((aligned(1024)));


/*
  Measured conversion factors from measured ADC level to actual voltage.
  Index 0..2 are for phases A, B, and C. Index 3 is for the neutral.
*/
static float phase_factor[4];


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


/*
  Set up initial config of GPIOs connected to L6234 motor controller.
  This allows to do initial "manual" control.
  Afterwards, we will reconfigure the GPIOs to be controlled from timer PWM.
*/
static void
setup_manual_l6234(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
  ROM_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOG);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTG_AHB_BASE,
                            GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|
                            GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);
  /* Set EN pins low, for off. */
  ROM_GPIOPinWrite(GPIO_PORTG_AHB_BASE, GPIO_PIN_3, 0);
  ROM_GPIOPinWrite(GPIO_PORTG_AHB_BASE, GPIO_PIN_4, 0);
  ROM_GPIOPinWrite(GPIO_PORTG_AHB_BASE, GPIO_PIN_5, 0);
  /* Set IN pins low, for low-side (connect-to-Gnd). */
  ROM_GPIOPinWrite(GPIO_PORTG_AHB_BASE, GPIO_PIN_0, 0);
  ROM_GPIOPinWrite(GPIO_PORTG_AHB_BASE, GPIO_PIN_1, 0);
  ROM_GPIOPinWrite(GPIO_PORTG_AHB_BASE, GPIO_PIN_2, 0);
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
setup_adc_basic(void)
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

  ROM_ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCER, ADC_TRIGGER_PROCESSOR, 0);
  /* Sample the phase 8 times. */
  for (i = 0; i < 8; ++i)
    ROM_ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCER, i,
                                 ADC_CTL_CH1 |
                                 (i==7 ? (ADC_CTL_IE | ADC_CTL_END) : 0));
  ROM_ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCER);

  /* Setup ADC1 for sampling the neutral point. */
  ROM_ADCSequenceConfigure(ADC1_BASE, ADC_SEQUENCER, ADC_TRIGGER_PROCESSOR, 0);
  /* Sample neutral phase 8 times. */
  for (i = 0; i < 8; ++i)
    ROM_ADCSequenceStepConfigure(ADC1_BASE, ADC_SEQUENCER, i,
                                 ADC_CTL_CH4 |
                                 (i==7 ? ( ADC_CTL_IE | ADC_CTL_END) : 0));
  ROM_ADCSequenceEnable(ADC1_BASE, ADC_SEQUENCER);
}


static void
setup_adc_timer_interrupts(void)
{
  ROM_ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCER, ADC_TRIGGER_TIMER, 0);
  ROM_ADCIntEnable(ADC0_BASE, ADC_SEQUENCER);
  ROM_IntEnable(INT_ADC0SS0);
  ROM_ADCSequenceConfigure(ADC1_BASE, ADC_SEQUENCER, ADC_TRIGGER_TIMER, 0);
  ROM_ADCIntEnable(ADC1_BASE, ADC_SEQUENCER);
  ROM_IntEnable(INT_ADC1SS0);
}


static void
adc_clear(void)
{
  ROM_ADCIntClear(ADC0_BASE, ADC_SEQUENCER);
  ROM_ADCIntClear(ADC1_BASE, ADC_SEQUENCER);
}


static void
adc_start(void)
{
  adc_clear();
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


/*
  Buffer to save ADC measurements, for later dumping over serial for debug.
  We measure a phase and a neutral, and save them in the buffer.
  ADC measurements are unsigned 12 bit 0..4095. This leaves free to use the
  upper bits for flags. We use the upper two bits to store which phase
  (0..2) or neutral (3) was measured.
*/
#define DBG_NUM_SAMPLES 8000
static volatile uint16_t dbg_adc_samples[DBG_NUM_SAMPLES];
static volatile uint32_t dbg_adc_idx = 0;

static void
dbg_add_samples_buf(uint16_t *phases, uint16_t *neutrals, uint32_t count,
                    uint32_t which_phase)
{
  uint32_t l_idx = dbg_adc_idx;
  which_phase = (which_phase & 3) << 14;
  while (count > 0 && l_idx < DBG_NUM_SAMPLES) {
    dbg_adc_samples[l_idx++] = (*phases++ & 0xfff) | which_phase;
    dbg_adc_samples[l_idx++] = (*neutrals++ & 0xfff) | ((uint16_t)3 << 14);
    --count;
  }
  dbg_adc_idx = l_idx;
}


static void __attribute__((unused))
dbg_dump_samples(void)
{
  uint32_t l_idx, i;

  if ((l_idx = dbg_adc_idx) < DBG_NUM_SAMPLES)
    return;

  serial_output_str
    ("-----------------------------------------------------------------------\r\n");
  for (i = 0; i < l_idx; i += 2) {
    uint32_t phase_n_flag = dbg_adc_samples[i];
    uint32_t neutral_n_flag = dbg_adc_samples[i+1];
    uint32_t phase = phase_n_flag & 0xfff;
    uint32_t neutral = neutral_n_flag & 0xfff;
    float phase_voltage = phase_factor[phase_n_flag >> 14] * (float)phase;
    float neutral_voltage = phase_factor[neutral_n_flag >> 14] * (float)neutral;
    float voltage = phase_voltage - neutral_voltage;
    println_float(voltage, 2, 5);
  }

  dbg_adc_idx = 0;
}


static inline void
hw_pa_high(void)
{
  my_gpio_write(GPIO_PORTG_AHB_BASE, GPIO_PIN_0, GPIO_PIN_0);
}


static inline void
hw_pb_high(void)
{
  my_gpio_write(GPIO_PORTG_AHB_BASE, GPIO_PIN_1, GPIO_PIN_1);
}


static inline void
hw_pc_high(void)
{
  my_gpio_write(GPIO_PORTG_AHB_BASE, GPIO_PIN_2, GPIO_PIN_2);
}


static inline void
hw_pa_low(void)
{
  my_gpio_write(GPIO_PORTG_AHB_BASE, GPIO_PIN_0, 0);
}


static inline void
hw_pb_low(void)
{
  my_gpio_write(GPIO_PORTG_AHB_BASE, GPIO_PIN_1, 0);
}


static inline void
hw_pc_low(void)
{
  my_gpio_write(GPIO_PORTG_AHB_BASE, GPIO_PIN_2, 0);
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
/* Number of ticks (PWM periods). */
static volatile uint32_t motor_tick = 0;
/* Which phase is currently measured (0..2). */
static uint32_t which_adc_phase;
/* Which phase will be measured in next ADC measurement. */
static uint32_t next_adc_phase;

static void
dbg_save_samples(void)
{
  uint32_t l_done_count = adc_done_counter;
  ++l_done_count;
  if (l_done_count == 2) {
    if (motor_adc_dbg)
      dbg_add_samples_buf(adc_phase_samples, adc_neutral_samples, 8, which_adc_phase);
    adc_done_counter = 0;
  }
  else
    adc_done_counter = l_done_count;
}

static float
adc_current_backemf(void)
{
  /*
    The timing of the ADC is so that the fourth sample is taken 2 microseconds
    before the end of the duty cycle, which seems to give good results. See
    IntHandlerTimer3B().
  */
  uint32_t phase_sample = adc_phase_samples[3];
  uint32_t neutral_sample = adc_neutral_samples[3];
  float phase_voltage = phase_factor[which_adc_phase]*(float)phase_sample;
  float neutral_voltage = phase_factor[3]*(float)neutral_sample;
  return phase_voltage - neutral_voltage;
}


void
IntHandlerADC0Seq0(void)
{
  int i;
  uint32_t l_next;

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
  if (!pa_enabled) {
    l_next = 0;
    HWREG(ADC0_BASE + ADC_O_SSMUX0) = 1 * (uint32_t)0x11111111;
  } else if (!pb_enabled) {
    l_next = 1;
    HWREG(ADC0_BASE + ADC_O_SSMUX0) = 0 * (uint32_t)0x11111111;
  } else {
    l_next = 2;
    HWREG(ADC0_BASE + ADC_O_SSMUX0) = 10 * (uint32_t)0x11111111;
  }

  /* Re-enable sequencer, optimised away for now. */
  //HWREG(ADC0_BASE + ADC_O_ACTSS |= (uint32_t)(1 << ADC_SEQUENCER);

  which_adc_phase = next_adc_phase;
  dbg_save_samples();
  next_adc_phase = l_next;
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
#define SAVED_COMMUTE_STEP_TICKS 8
/* Tick counter at last commute steps. */
static volatile uint32_t motor_last_commute[SAVED_COMMUTE_STEP_TICKS];
/* Current index into motor_last_commute (last updated value). */
static volatile uint32_t motor_last_commute_idx = 0;
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

volatile uint32_t motor_idle = 1;
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


static int
cmp_ushort(const void *pa, const void *pb)
{
  uint16_t a = *(const uint16_t *)pa;
  uint16_t b = *(const uint16_t *)pb;
  if (a < b)
    return -1;
  else if (a > b)
    return 1;
  else
    return 0;
}


#define REPEATS 32
#define PHASE_MEASURE_SIZE (8*REPEATS*3*2)
#if 6*PHASE_MEASURE_SIZE > DBG_NUM_SAMPLES*2
#error Too many phase measurement samples to fit in buffer
#endif

/*
  Put voltage on one winding, while the other two windings are disconnected.
  This will draw no current in the motor, but will put the supply voltage on
  all three phases as well as on the neutral.

  Measure the voltage on each phase/neutral using the ADC. This way, we can
  measure any bias between them (eg. due to resistor tolerances), and later
  adjust measurements to hopefully eliminate such bias.
*/
static void
measure_voltage_divider_bias(void)
{
  uint32_t i, j, k, l;
  uint16_t *val_phase[3], *val_neutral;
  uint16_t med_neutral;

  /* Reuse the dbg_adc_samples buffer, to save RAM. */
  val_phase[0] = (uint16_t *)((char *)dbg_adc_samples + 0);
  val_phase[1] = (uint16_t *)((char *)dbg_adc_samples + PHASE_MEASURE_SIZE);
  val_phase[2] = (uint16_t *)((char *)dbg_adc_samples + 2*PHASE_MEASURE_SIZE);
  val_neutral  = (uint16_t *)((char *)dbg_adc_samples + 3*PHASE_MEASURE_SIZE);

  for (k = 0; k < 3; ++k) {
    const char *got_voltage;

    switch(k) {
    case 0:
      hw_pa_high();
      hw_pa_enable();
      got_voltage = "A";
      break;
    case 1:
      hw_pb_high();
      hw_pb_enable();
      got_voltage = "B";
      break;
    case 2:
      hw_pc_high();
      hw_pc_enable();
      got_voltage = "C";
      break;
    }
#ifdef BIAS_VERBOSE
    serial_output_str("Putting voltage on phase ");
    serial_output_str(got_voltage);
    serial_output_str("\r\n");
#else
    (void)got_voltage;                          /* Silence compiler warning */
#endif

    ROM_SysCtlDelay(MCU_HZ/3/10);

    for (j = 0; j < 3; ++j) {
      uint32_t chan;
      const char *which;

      switch (j)  {
      case 0:
        chan = 1;
        which = "A";
        break;
      case 1:
        chan = 0;
        which = "B";
        break;
      case 2:
        chan = 10;
        which = "C";
        break;
      }

#ifdef BIAS_VERBOSE
      serial_output_str("Start measure on phase ");
      serial_output_str(which);
      serial_output_str("\r\n");
#else
    (void)which;                                /* Silence compiler warning */
#endif
      /* Read phase and neutral. */
      HWREG(ADC0_BASE + ADC_O_SSMUX0) = chan * (uint32_t)0x11111111;
      next_adc_phase = j;
      /* Run 2*REPEATS measurements, only using the later half ("warm-up"). */
      for (l = 0; l < REPEATS*2; ++l) {
        /*
          A small delay before next measurement seems to greatly help get more
          stable ADC readings.
        */
        ROM_SysCtlDelay(MCU_HZ/3/1000);
        adc_start();
        while (!adc_ready())
          ;
        if (l < REPEATS)
          continue;

#ifdef BIAS_VERBOSE
        serial_output_str("Phase "); serial_output_str(which); serial_output_str(":");
#endif
        for (i = 0; i < 8; ++i) {
          uint32_t adc_val = HWREG(ADC0_BASE + ADC_SEQUENCER_FIFO) & 0xfff;
          val_phase[j][i+l*8+k*8*REPEATS] = adc_val;
#ifdef BIAS_VERBOSE
          serial_output_str(" ");
          print_uint32(adc_val);
#endif
        }
#ifdef BIAS_VERBOSE
        serial_output_str("\r\nNeutral:");
#endif
        for (i = 0; i < 8; ++i) {
          uint32_t adc_val = HWREG(ADC1_BASE + ADC_SEQUENCER_FIFO) & 0xfff;
          val_neutral[i+l*8+j*8*REPEATS+k*8*REPEATS*3] = adc_val;
#ifdef BIAS_VERBOSE
          serial_output_str(" ");
          print_uint32(adc_val);
#endif
        }
#ifdef BIAS_VERBOSE
        serial_output_str("\r\n");
#endif
      }
    }

    /* Restore things, preparing for normal operation. */
    hw_pa_low();
    hw_pb_low();
    hw_pc_low();
    adc_clear();
    ROM_SysCtlDelay(MCU_HZ/3/1000);
    hw_pa_disable();
    hw_pb_disable();
    hw_pc_disable();
  }

  /* Find the median. */
  qsort(val_neutral, 3*3*8*REPEATS, sizeof(uint16_t), cmp_ushort);
  med_neutral = val_neutral[3*3*8*REPEATS/2];
  phase_factor[3] = SUPPLY_VOLTAGE/(float)med_neutral;
  serial_output_str("Phase bias:");
  for (j = 0; j < 3; ++j) {
    uint16_t med_phase;
    qsort(val_phase[j], 3*8*REPEATS, sizeof(uint16_t), cmp_ushort);
    med_phase = val_phase[j][3*8*REPEATS/2];
    phase_factor[j] = SUPPLY_VOLTAGE/(float)med_phase;
    serial_output_str(" ");
    print_float(phase_factor[3]*(float)med_phase - phase_factor[3]*(float)med_neutral,
                2, 3);
  }
  serial_output_str("\r\n");
}
#undef REPEATS


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
  float backemf = adc_current_backemf();
  uint32_t l_target;

  /*
    In even (0,2,4) commute steps, we are looking for downwards zero crossing.
    In odd (1,3,5), for upwards.

    ToDo: Probably need some filtering here.
  */
  if ((!(l_step & 1) && backemf < 0.0f) || ((l_step & 1) && backemf > 0.0f)) {
    /*
      Zero-crossing detected. Set next commute target to estimated 30
      electrical degrees (1/12 of last 6 commute periods == last electrical
      revolution).
    */
    uint32_t cur_idx = motor_last_commute_idx;
    uint32_t prev_idx = (cur_idx + (SAVED_COMMUTE_STEP_TICKS - 6)) % SAVED_COMMUTE_STEP_TICKS;
    uint32_t motor_last_commute_duration =
      motor_last_commute[cur_idx] - motor_last_commute[prev_idx];
    l_target = delta + motor_last_commute_duration/12 - 2;
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
    uint32_t l_idx = motor_last_commute_idx;
    l_spinning_up = motor_spinning_up;
    l_step = motor_commute_step;

    /* Check if it is time for a new commute step. */
    delta = l_motor_tick - motor_last_commute[l_idx];
    l_target = motor_commute_target;
    if (l_target && delta >= l_target) {
      ++l_step;
      if (l_step == 6) {
        l_step = 0;
        ++motor_revolutions;
      }
      setup_commute_step(l_step);
      motor_commute_step = l_step;
      ++l_idx;
      if (l_idx == SAVED_COMMUTE_STEP_TICKS)
        l_idx = 0;
      motor_last_commute[l_idx] = l_motor_tick;
      motor_last_commute_idx = l_idx;
      delta = 0;

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


void
start_motor(void)
{
  if (motor_idle)
  {
    /* Spin up the motor a bit. */
    start_open_loop(0.05f, 3.0f, 3.0f);
  }
}


void
stop_motor(void)
{
  if (!motor_idle)
    start_spin_down();
}


static uint32_t
my_strlen(const char *s)
{
  uint32_t len = 0;
  while (*s++)
    ++len;
  return len;
}


static uint32_t last_button_time = 0xffffffff;
static uint32_t keypress_retransmit = 0;
static uint32_t keydata_sofar = 0;
static uint32_t usb_sofar = 0;


/*
  Read a packet from USB. A packet is 32 (NRF_PACKET_SIZE) bytes of data.

  Handles sync-up by resetting the state (and resetting the packet buffer to
  empty) whenever more than 0.2 seconds pass without any data received.

  Returns a true value if a reset was done, false if not.

  The KEYS argument enables checking for button presses and returning them
  as a fake packet.
*/
static uint32_t
usb_get_packet(uint8_t *packet_buf, uint32_t max_reset_count,
               uint32_t *timeout_flag, uint32_t keys)
{
  uint32_t reset = 0;
  uint32_t sofar = 0;
  uint32_t start_time;
  uint32_t cur_time;
  uint8_t val;
  uint32_t h, t;
  uint32_t reset_count;

  if (max_reset_count > 0)
    *timeout_flag = 0;
  while (sofar < NRF_PACKET_SIZE)
  {
    t = usb_recvbuf.tail;
    start_time = get_time();
    reset_count = 0;
    while ((h = usb_recvbuf.head) == t)
    {
      /* Simple sync method: if data stream pauses, then reset state. */
      if (calc_time(start_time) > MCU_HZ/5)
      {
        sofar = 0;
        reset = 1;
        start_time = get_time();
        if (max_reset_count > 0)
        {
          ++reset_count;
          if (reset_count >= max_reset_count)
          {
            *timeout_flag = 1;
            return reset;
          }
        }
      }

      /* Every 4 milliseconds, read the buttons. */
      cur_time = get_time();
      if (last_button_time == 0xffffffff ||
          calc_time_from_val(last_button_time, cur_time) > MCU_HZ/250 ||
          (keydata_sofar > 0 && !transmit_running))
      {
        last_button_time = cur_time;
        check_buttons();
#if 0
        {
          uint32_t i;
          for (i = 0 ; i < 19; ++i)
            serial_output_hexbyte(button_status[i]);
          serial_output_str("\r\n");
        }
#endif

        /* Only if not sending framebuffer data, and no partial usb packet */
        if (keys &&
            (transmit_running != TRANSMIT_RUNNING_FRAMEDATA) &&
            usb_sofar == 0)
        {
          int diff = memcmp(button_status, prev_button_status, sizeof(button_status));
          /*
            We return a keypress packet if key state has changed. And after
            such change, we return further keypress packets to increase the
            likeliness that at least one will arrive even in case of noise on
            the wireless transmission.

            We also return a keypress packet if there is any existing keypress
            data pending, and the nRf interrupt state machine is ready to send
            a new batch.
           */
          if (diff ||
              keypress_retransmit > 0 ||
              (keydata_sofar > 0 && !transmit_running))
          {
            if (diff)
              keypress_retransmit = KEY_RETRANSMITS;
            /* Create a fake packet containing key presses. */
            packet_buf[0] = POV_CMD_CONFIG;
            packet_buf[1] = POV_SUBCMD_KEYPRESSES;
            memcpy(packet_buf+2, button_status, sizeof(button_status));
            memset(packet_buf+2+sizeof(button_status), 0,
                   NRF_PACKET_SIZE-(2+sizeof(button_status)));
            memcpy(prev_button_status, button_status, sizeof(prev_button_status));
            if (keypress_retransmit > 0)
              --keypress_retransmit;
            return reset;
          }
        }
      }
    }
    val = usb_recvbuf.buf[t];
    ++t;
    if (t >= USB_RECV_BUF_SIZE)
      t = 0;
    usb_recvbuf.tail = t;
    packet_buf[sofar++] = val;
  }

  return reset;
}


/*
  Delay until specified amount of systicks have passed.

  As systick is a 24-bit counter, the amount cannot exceed 0xffffff, or a bit
  more than 16000000.
*/
static void
delay_systicks(uint32_t cycles)
{
  uint32_t start = get_time();

  while (calc_time(start) < cycles)
    ;
}


void
delay_us(uint32_t us)
{
  /* This assumes that MCU_HZ is divisible by 1000000. */
  uint32_t cycles = (MCU_HZ/1000000)*us;
#if (MCU_HZ % 1000000)
#error delay_us() computes delay incorrectly if MCU_HZ is not a multiple of 1000000
#endif

  while (cycles > 0xffffff)
  {
    delay_systicks(0xffffff/2);
    cycles -= 0xffffff/2;
  }
  delay_systicks(cycles);
}


static void
handle_cmd_debug(uint8_t *packet)
{
  uint8_t subcmd= packet[1];
  uint32_t usb_timeout = 0;
  static const uint32_t usb_timeout_seconds = 5;

  /* First we need to wait for any on-going transmit to complete. */
  while (transmit_running)
    ;
  delay_us(40000);

  if (subcmd == POV_SUBCMD_RESET_TO_BOOTLOADER)
  {
    /*
      This debug command is sent to the application, not to the bootloader.
      It requests the app to execute a software reset to get to the bootloader,
      avoiding the need for manual press of the reset button.

      Since we are using back-to-back transmission in the app for maximum
      throughput, without automatic ack and re-transmit, we here send the
      packet three times, to decrease the risk of it getting lost.
    */
    nrf_transmit_packet_nack(packet);
    delay_us(40000);
    nrf_transmit_packet_nack(packet);
    delay_us(40000);
    nrf_transmit_packet_nack(packet);
    delay_us(40000);
    /* Grab the next packet for the bootloader. */
    usb_get_packet(packet, usb_timeout_seconds*5, &usb_timeout, 0);
  }

  nrf_config_bootload_tx(NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);

  while (!usb_timeout)
  {
    const char *errmsg;
    uint8_t subcmd = packet[1];

    /*
      Normally, we should exit after we have seen POV_SUBCMD_EXIT_DEBUG.
      But if we happen to get interrupted in the middle of something,
      exit if we see any non-debug command (for now, unfortunately that
      command will be lost).
    */
    if (packet[0] != POV_CMD_DEBUG || subcmd == POV_SUBCMD_EXIT_DEBUG)
      break;

    errmsg = nrf_transmit_packet(packet);

    if (!errmsg &&
        (subcmd == POV_SUBCMD_FLASH_BUFFER ||
         subcmd == POV_SUBCMD_ENTER_BOOTLOADER))
    {
      /* Get the status reply from the bootloader. */
      uint32_t start_time, wait_counter;

      nrf_config_bootload_rx(NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
      nrf_ce_high(NRF_CE_BASE, NRF_CE_PIN);
      /*
        nRF24L01+ datasheet says that there must be at least 4 microseconds
        from a positive edge on CE to CSN being taken low.
      */
      delay_us(4);

      start_time = get_time();
      /*
        For STM32F4, we have big sectors (up to 128MB). According to the data
        sheet, they can take up to 4s to erase maximum, plus another up to
        100us maximum per word to write. So the worst case latency is actually
        up to around 8 seconds, though the common case should be much faster.
      */
      wait_counter = 80;  /* 8.0 seconds */
      while (wait_counter > 0)
      {
        uint32_t now_time;
        uint32_t status = nrf_get_status(NRF_SSI_BASE,
                                         NRF_CSN_BASE, NRF_CSN_PIN);
        if (status & nRF_RX_DR)
          break;                                    /* Data ready. */
        now_time = get_time();
        if (calc_time_from_val(start_time, now_time) > MCU_HZ/10)
        {
          --wait_counter;
          start_time = dec_time(start_time, MCU_HZ/10);
        }
      }
      nrf_ce_low(NRF_CE_BASE, NRF_CE_PIN);

      if (!wait_counter)
        errmsg = "E: Timeout waiting for reply from bootloader\r\n";
      else
      {
        nrf_rx(packet, NRF_PACKET_SIZE,
               NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
        if (packet[0] != POV_CMD_DEBUG || packet[1] != POV_SUBCMD_STATUS_REPLY)
          errmsg = "E: Unexpected reply packet from bootloader\r\n";
        else if (packet[2])
          errmsg = "E: Error status reply from bootloader\r\n";
      }
      nrf_config_bootload_tx(NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
    }

    if (errmsg)
      usb_data_put((const unsigned char *)errmsg, my_strlen(errmsg));
    else
      usb_data_put((const unsigned char *)"OK\r\n", 4);

    /* Get the next command over USB from the programmer. */
    usb_get_packet(packet, usb_timeout_seconds*5, &usb_timeout, 0);
  }
  if (usb_timeout)
  {
    static const char inactivity_error[] =
      "E: No activity on USB, leaving debug mode\r\n";
    usb_data_put((const unsigned char *)&inactivity_error[0],
                 sizeof(inactivity_error)-1);
    delay_us(200000);
  }

  nrf_config_normal_tx(NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
}


static void
handle_cmd_set_config(uint8_t *packet)
{
  /* First we need to wait for any on-going transmit to complete. */
  while (transmit_running)
    ;
  delay_us(40000);

  /*
    As we are transmitting without acks from the receiver, transmit the
    command 3 times to maximise the changes that it will be received.
  */
  nrf_transmit_packet_nack(packet);
  delay_us(40000);
  nrf_transmit_packet_nack(packet);
  delay_us(40000);
  nrf_transmit_packet_nack(packet);
  delay_us(40000);
}


int main()
{
  uint32_t read_idx;

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
  /* Playstation DualShock interrupt priority. */
  ROM_IntPrioritySet(INT_SSI0, 3 << 5);
  ROM_IntPrioritySet(INT_TIMER1B, 5 << 5);
  ROM_IntPrioritySet(INT_USB0, 5 << 5);

  setup_systick();
  setup_controlpanel();
  setup_ps2();
  setup_nrf();
  config_usb();

  setup_adc_basic();
  setup_manual_l6234();
  motor_idle = 1;
  motor_set_damper(0.0f);

  measure_voltage_divider_bias();

  setup_adc_timer_interrupts();
  setup_timer_pwm();

  start_ps2_interrupts();

  nrf_startup();

  /* Small delay to allow USB to get up before starting motor. */
  ROM_SysCtlDelay(MCU_HZ/3);

  serial_output_str("Motor controller init done\r\n");

  usb_sofar = 0;
  read_idx = 0;
  nrf_write_idx = 1;
  for (;;)
  {
    uint8_t val;
    uint8_t usb_packet[NRF_PACKET_SIZE];

    /* Wait for a packet on the USB. */
    if (usb_get_packet(usb_packet, 0, NULL, 1))
    {
      /*
        If we had a resync on the USB, then reset the frame packet count.
        We do not want to transmit a half-received frame to the POV.
      */
      usb_sofar = 0;
    }
    val = usb_packet[0];

    if (val == POV_CMD_DEBUG)
    {
      handle_cmd_debug(usb_packet);
      continue;
    }
    else if (val == POV_CMD_CONFIG)
    {
      if (usb_packet[1] == POV_SUBCMD_SET_CONFIG)
        handle_cmd_set_config(usb_packet);
      else if (usb_packet[1] == POV_SUBCMD_KEYPRESSES)
      {
        if (usb_sofar == 0)
        {
          if (keydata_sofar + NRF_PACKET_SIZE <= BUF_SIZE_PAD)
          {
            memcpy(&nrf_frame_buffers[read_idx][keydata_sofar],
                   usb_packet, NRF_PACKET_SIZE);
            keydata_sofar += NRF_PACKET_SIZE;
          }
          if (!transmit_running)
          {
            nrf_write_idx = read_idx;
            read_idx = (1 - read_idx);
            transmit_start_keypresses(keydata_sofar);
            keydata_sofar = 0;
          }
        }
        else
          serial_output_str("Skip key due to framedata\r\n");
      }
      continue;
    }
    else
    {
      serial_output_str("2D pov framebuffer transmit not implemnted in this "
                        "version of pov_sender!\r\n");
    }
  }
}
