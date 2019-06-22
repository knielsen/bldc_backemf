#include "bldc_backemf.h"
#include "dbg.h"
#include "ps2.h"

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
setup_controlpanel(void)
{

  /* Switch 1, 2, and 3 on PA6, PC4, and PC7. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOA);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  ROM_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOC);
  ROM_GPIODirModeSet(GPIO_PORTA_AHB_BASE, GPIO_PIN_6, GPIO_DIR_MODE_IN);
  ROM_GPIOPadConfigSet(GPIO_PORTA_AHB_BASE, GPIO_PIN_6,
                       GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  ROM_GPIODirModeSet(GPIO_PORTC_AHB_BASE, GPIO_PIN_4|GPIO_PIN_7, GPIO_DIR_MODE_IN);
  ROM_GPIOPadConfigSet(GPIO_PORTC_AHB_BASE, GPIO_PIN_4|GPIO_PIN_7,
                       GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}


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


/*
  Status of all buttons. There's some legacy here, there used to be a custom
  PCB with a few buttons, then PS2 controller was added, and now the custom
  PCB is gone and only 3 dip-switches remain from there.

  All bits are active high.

  Byte 0:
   bit 0  button 1 (or "Left")
   bit 1  button 2 (or "Right")
   bit 2  button 2 (or "Down")
   bit 3  button 2 (or "Up")
   bit 4  button 2 (or SELECT)
   bit 5  switch 1 (also on DualShock controller board)
   bit 6  switch 2
   bit 7  switch 3 (also on DualShock controller board)

  Byte 1:
   bit 0: SELECT
   bit 1: L3
   bit 2  R3
   bit 3  START
   bit 4  Up
   bit 5  Right
   bit 6  Down
   bit 7  Left

  Byte 2:
   bit 0: L2
   bit 1: R2
   bit 2  L1
   bit 3  R1
   bit 4  Triangle
   bit 5  Circle
   bit 6  Cross
   bit 7  Square

  Byte 3: Joystick R left (0x00) -> right (0xff)
  Byte 4: Joystick R up (0x00) -> down (0xff)
  Byte 5: Joystick L left (0x00) -> right (0xff)
  Byte 6: Joystick L up (0x00) -> down (0xff)

  Byte 7: Button "right" pressure sensitivity
  Byte 8: Button "left" pressure sensitivity
  Byte 9: Button "up" pressure sensitivity
  Byte 10: Button "down" pressure sensitivity
  Byte 11: Button "triangle" pressure sensitivity
  Byte 12: Button "circle" pressure sensitivity
  Byte 13: Button "cross" pressure sensitivity
  Byte 14: Button "square" pressure sensitivity
  Byte 15: Button "L1" pressure sensitivity
  Byte 16: Button "R1" pressure sensitivity
  Byte 17: Button "L2" pressure sensitivity
  Byte 18: Button "R2" pressure sensitivity
*/
uint8_t button_status[19];
uint8_t prev_button_status[19];

void
check_buttons(void)
{
  uint32_t ps2_but_status1;
  long pa = my_gpio_read(GPIO_PORTA_AHB_BASE, 0x40);
  long pc = my_gpio_read(GPIO_PORTC_AHB_BASE, 0x90);
  uint8_t status =
    ~( ((pc << 1) & 0x20) | ((pc >> 1) & 0x40) | ((pa << 1) & 0x80) | 0x1f);
  button_status[1] = ~ps2_button_state[0];
  button_status[2] = ~ps2_button_state[1];
  memcpy(button_status+3, ps2_button_state+2, 16);
  ps2_but_status1 = button_status[1];
  /* Let some DualShock buttons mirror the small panel buttons. */
  status |= ((ps2_but_status1 >> 7) & 0x01);   /* Left */
  status |= ((ps2_but_status1 >> 4) & 0x06);   /* Right/Down */
  status |= ((ps2_but_status1 >> 1) & 0x08);   /* Up */
  status |= ((ps2_but_status1 << 4) & 0x10);   /* Select */
  button_status[0] = status;

 /* ToDo: Some anti-prell handling here? */
  if (status & (1<<7))
  {
    if (motor_idle)
      start_motor();
  }
  else
  {
    if (!motor_idle)
      stop_motor();
  }
}


void
setup_ps2(void)
{
  setup_controlpanel();
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
