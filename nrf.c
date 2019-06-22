#include "bldc_backemf.h"

/*
  Note that to change these, may require additional changes in
  config_ssi_gpio() and in IRQ handler setup.
*/
#define NRF_SSI_BASE SSI1_BASE
#define NRF_CSN_BASE GPIO_PORTF_AHB_BASE
#define NRF_CSN_PIN GPIO_PIN_3
#define NRF_CE_BASE GPIO_PORTB_AHB_BASE
#define NRF_CE_PIN GPIO_PIN_3
#define NRF_IRQ_BASE GPIO_PORTB_AHB_BASE
#define NRF_IRQ_PIN GPIO_PIN_0
#define NRF_DMA_CH_RX UDMA_CHANNEL_SSI1RX
#define NRF_DMA_CH_TX UDMA_CHANNEL_SSI1TX

/* Communications protocol. */
#define NRF_PACKET_SIZE 32

#define POV_CMD_CONFIG 255
#define POV_SUBCMD_SET_CONFIG 1
#define POV_SUBCMD_KEYPRESSES 2

#define POV_CMD_DEBUG 254
#define POV_SUBCMD_RESET_TO_BOOTLOADER 255
#define POV_SUBCMD_ENTER_BOOTLOADER 254
#define POV_SUBCMD_RESET_TO_APP 253
#define POV_SUBCMD_FLASH_BUFFER 252
#define POV_SUBCMD_EXIT_DEBUG   251
#define POV_SUBCMD_STATUS_REPLY 240

/* Total number of times a key is transmittet (for redundancy). */
#define KEY_RETRANSMITS 8


void
config_ssi_gpio(void)
{
  /* Config Tx on SSI1, PF0-PF3 + PB0/PB3. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  ROM_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOF);

  /* PF0 is special (NMI), needs unlock to be re-assigned to SSI1. */
  HWREG(GPIO_PORTF_AHB_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
  HWREG(GPIO_PORTF_AHB_BASE + GPIO_O_CR) |= 0x01;
  HWREG(GPIO_PORTF_AHB_BASE + GPIO_O_LOCK) = 0;

  ROM_GPIOPinConfigure(GPIO_PF2_SSI1CLK);
  ROM_GPIOPinConfigure(GPIO_PF0_SSI1RX);
  ROM_GPIOPinConfigure(GPIO_PF1_SSI1TX);
  ROM_GPIOPinTypeSSI(GPIO_PORTF_AHB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
  /* CSN pin, high initially */
  ROM_GPIOPinTypeGPIOOutput(NRF_CSN_BASE, NRF_CSN_PIN);
  ROM_GPIOPinWrite(NRF_CSN_BASE, NRF_CSN_PIN, NRF_CSN_PIN);
  /* CE pin, low initially */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOB);
  ROM_GPIOPinTypeGPIOOutput(NRF_CE_BASE, NRF_CE_PIN);
  ROM_GPIOPinWrite(NRF_CE_BASE, NRF_CE_PIN, 0);
  /* IRQ pin as input. */
  ROM_GPIOPinTypeGPIOInput(NRF_IRQ_BASE, NRF_IRQ_PIN);
}
