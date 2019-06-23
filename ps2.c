#include "bldc_backemf.h"

#include <string.h>
#include <stdlib.h>

#include "inc/hw_ssi.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "driverlib/udma.h"
#include "driverlib/ssi.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"


static void
config_spi_ps2(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
  ROM_GPIOPinConfigure(GPIO_PA4_SSI0RX);
  ROM_GPIOPinConfigure(GPIO_PA5_SSI0TX);
  ROM_GPIOPinTypeSSI(GPIO_PORTA_AHB_BASE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);
  /* Pull-up is needed on "data". */
  ROM_GPIOPadConfigSet(GPIO_PORTA_AHB_BASE, GPIO_PIN_4,
                       GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

  /* Configure the "attention" pins PC6 and PA3. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  ROM_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOC);
  ROM_GPIODirModeSet(GPIO_PORTC_AHB_BASE, GPIO_PIN_6, GPIO_DIR_MODE_OUT);
  ROM_GPIOPadConfigSet(GPIO_PORTC_AHB_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA,
                       GPIO_PIN_TYPE_STD);
  ROM_GPIOPinWrite(GPIO_PORTC_AHB_BASE, GPIO_PIN_6, GPIO_PIN_6);
  ROM_GPIODirModeSet(GPIO_PORTA_AHB_BASE, GPIO_PIN_3, GPIO_DIR_MODE_OUT);
  ROM_GPIOPadConfigSet(GPIO_PORTA_AHB_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA,
                       GPIO_PIN_TYPE_STD);
  ROM_GPIOPinWrite(GPIO_PORTA_AHB_BASE, GPIO_PIN_3, GPIO_PIN_3);

  /*
    Configure the SPI for correct mode to talk to PS2 DualShock2 controller.

    We need CLK active low, so SPO=1.
    We need to setup on leading, sample on trailing CLK edge, so SPH=1.

    The speed is 500 kHz.
    Source: http://store.curiousinventor.com/guides/PS2/
  */

  ROM_SSIDisable(SSI0_BASE);
  ROM_SSIConfigSetExpClk(SSI0_BASE, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_3,
                         SSI_MODE_MASTER, 500000, 8);
  ROM_SSIEnable(SSI0_BASE);
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


/*
  The Tiva SSI peripheral uses MSB first, but DualShock controller needs LSB
  first. This compile-time macro allows to bitflip a constant value as needed.
  The RBIT() function is a more efficient way to do this at runtime.
*/
#define R_(x) ((((x)&0x1) << 7) | (((x)&0x2) << 5) | (((x)&0x4) << 3) | \
               (((x)&0x8) << 1) | (((x)&0x10) >> 1) | (((x)&0x20) >> 3) \
               | (((x)&0x50) >> 5) | (((x)&0x80) >> 7))
struct { uint32_t len; uint8_t data[21]; } ps2_cmds[] = {
  /* Do an initial poll to check response and detect missing controller. */
  { 5, { R_(0x01), R_(0x42), 0, 0, 0}},
  /* Enter config/escape mode. */
  { 5, { R_(0x01), R_(0x43), 0, R_(0x01) /* Enter config/escape mode */, 0}},
  /* Enable and lock analog mode. */
  { 9, { R_(0x01), R_(0x44), 0, R_(0x01) /* Enable analog */,
         R_(0x03) /* Lock analog mode */, 0, 0, 0, 0}},
#if 0
  /* Set vibration motor mapping. */
  { 9, { R_(0x01), R_(0x4d), 0, 0, R_(0x01), 0xff, 0xff, 0xff, 0xff}},
#endif
  /* Configure to return key pressure sensitivity. */
  { 9, { R_(0x01), R_(0x4f), 0,
         0xff, 0xff, R_(0x03) /* 18 set bits enable 18 bytes of response */,
         0, 0, 0}},
  /* Exit config mode. */
  { 9, { R_(0x01), R_(0x43), 0, 0 /* exit config/escape mode */,
         R_(0x5a), R_(0x5a), R_(0x5a), R_(0x5a), R_(0x5a)}},
  /* Poll for 18 bytes of extended status (all pressure-sensitive buttons). */
  { 21, { R_(0x01), R_(0x42), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
};
#undef R_


static volatile uint32_t ps2_current_cmd_idx = 0;
uint8_t ps2_button_state[18] = { 0xff, 0xff, 0x7f, 0x7f, 0x7f, 0x7f };
static volatile uint8_t ps2_dma_running = 0;
static uint8_t ps2_recvbuf[21];

void
IntHandlerSSI0(void)
{
  uint32_t i;

  my_disable_timera(TIMER1_BASE);

  /*
    Check if a controller is connected.
    Could refine this check, but perhaps this is sufficient.
  */
  if (unlikely(ps2_recvbuf[1] == 0xff))
  {
    /* Clear any currently marked buttons. */
    if (unlikely(ps2_current_cmd_idx == sizeof(ps2_cmds)/sizeof(ps2_cmds[0])-1))
    {
      ps2_button_state[0] = 0xff;
      ps2_button_state[1] = 0xff;
      ps2_button_state[2] = 0x7f;
      ps2_button_state[3] = 0x7f;
      ps2_button_state[4] = 0x7f;
      ps2_button_state[5] = 0x7f;
      memset(ps2_button_state+6, 0, sizeof(ps2_button_state)-6);
    }
    /* Reset and start over config if a controller is connected later. */
    ps2_current_cmd_idx = 0;
  }
  else if (likely(ps2_current_cmd_idx == sizeof(ps2_cmds)/sizeof(ps2_cmds[0])-1))
  {
    /* Got a read, save it. */
    for (i = 0; i < 18; ++i)
      ps2_button_state[i] = RBIT(ps2_recvbuf[i+3]);
  }
  else
  {
    /* Config step successful, move to next step. */
    ++ps2_current_cmd_idx;
  }

  /* Take "attention" high to complete transfer. */
  my_gpio_write(GPIO_PORTC_AHB_BASE, GPIO_PIN_6, GPIO_PIN_6);

  ps2_dma_running = 0;
}


void
IntHandlerTimer1B(void)
{
  uint8_t *ps2_data;
  uint32_t ps2_data_len;
  uint32_t idx;

  my_clear_timerb_timeout_int(TIMER1_BASE);
  if (ps2_dma_running)
    return;

  /* Take "attention" low to initiate transfer. */
  my_gpio_write(GPIO_PORTC_AHB_BASE, GPIO_PIN_6, 0);

  idx = ps2_current_cmd_idx;
  ps2_data = ps2_cmds[idx].data;
  ps2_data_len = ps2_cmds[idx].len;

  /*
    Start a timer to trigger uDMA transfers to the SSI Tx register.
    This way, we can get delays between individual bytes, as required by
    the DualShock2 controller, and still use uDMA for the transfer.
  */
  ps2_dma_running = 1;
  ROM_uDMAChannelTransferSet(UDMA_CH10_SSI0RX | UDMA_PRI_SELECT,
                             UDMA_MODE_BASIC, (void *)(SSI0_BASE + SSI_O_DR),
                             ps2_recvbuf, ps2_data_len);
  ROM_uDMAChannelEnable(UDMA_CH10_SSI0RX);
  ROM_uDMAChannelTransferSet(UDMA_CH20_TIMER1A | UDMA_PRI_SELECT,
                             UDMA_MODE_BASIC, ps2_data,
                             (void *)(SSI0_BASE + SSI_O_DR), ps2_data_len);
  ROM_uDMAChannelEnable(UDMA_CH20_TIMER1A);
  my_enable_timera(TIMER1_BASE);
}


#define SSI_PS2_TXDMA_CHAN_ASSIGN UDMA_CH11_SSI0TX
#define SSI_PS2_TXDMA (SSI_PS2_TXDMA_CHAN_ASSIGN & 0xff)
#define SSI_PS2_RXDMA_CHAN_ASSIGN UDMA_CH10_SSI0RX
#define SSI_PS2_RXDMA (SSI_PS2_RXDMA_CHAN_ASSIGN & 0xff)

static void
config_udma_for_ps2(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
  ROM_uDMAEnable();
  ROM_uDMAControlBaseSet(udma_control_block);

  ROM_SSIDMAEnable(SSI0_BASE, SSI_DMA_TX);
  ROM_SSIDMAEnable(SSI0_BASE, SSI_DMA_RX);
  ROM_IntEnable(INT_SSI0);

  /*
    Setup uDMA channel for SSI0 Rx.
    (The Tx part is handled by TIMER1 uDMA, not SSI0 uDMA).
  */
  ROM_uDMAChannelAttributeDisable(SSI_PS2_RXDMA, UDMA_ATTR_ALTSELECT |
                                  UDMA_ATTR_REQMASK | UDMA_ATTR_HIGH_PRIORITY);
  ROM_uDMAChannelAssign(SSI_PS2_RXDMA_CHAN_ASSIGN);
  ROM_uDMAChannelAttributeEnable(SSI_PS2_RXDMA, UDMA_ATTR_USEBURST);
  ROM_uDMAChannelControlSet(SSI_PS2_RXDMA | UDMA_PRI_SELECT,
                            UDMA_SIZE_8 | UDMA_DST_INC_8 | UDMA_SRC_INC_NONE |
                            UDMA_ARB_4);

  /* Set up TIMER1A to trigger a DMA request every 32 us. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
  /* Timer 1B is used to start a new DualShock controller read every 10 ms. */
  ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR|
                     TIMER_CFG_A_PERIODIC|TIMER_CFG_B_PERIODIC);
  /* 32 us * 80e6 cycles/s -> 2560. */
  ROM_TimerPrescaleSet(TIMER1_BASE, TIMER_A, 0);
  ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, MCU_HZ/1000000*32);
  /* 10 ms * 80e6 cycles/s / 100 cycles/tick -> 8000 ticks. */
  ROM_TimerPrescaleSet(TIMER1_BASE, TIMER_B, 100);
  ROM_TimerLoadSet(TIMER1_BASE, TIMER_B, MCU_HZ*10/1000/100);
  ROM_uDMAChannelAssign(UDMA_CH20_TIMER1A);
  ROM_uDMAChannelAttributeDisable(UDMA_CH20_TIMER1A, UDMA_ATTR_ALTSELECT |
                                  UDMA_ATTR_REQMASK | UDMA_ATTR_HIGH_PRIORITY);
  ROM_uDMAChannelControlSet(UDMA_CH20_TIMER1A | UDMA_PRI_SELECT,
                        UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
                        UDMA_ARB_1);
  ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMB_TIMEOUT);
}

void
setup_ps2(void)
{
  config_spi_ps2();
  config_udma_for_ps2();
}

void
start_ps2_interrupts(void)
{
  /* Enable interrrupts. */
  ROM_IntEnable(INT_TIMER1B);
  /* Start timer to poll PlayStation 2 DualShock controller periodically. */
  ROM_TimerEnable(TIMER1_BASE, TIMER_B);
}
