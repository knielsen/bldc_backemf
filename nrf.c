#include "bldc_backemf.h"

#include "inc/hw_ssi.h"
#include "inc/hw_ints.h"
#include "driverlib/udma.h"
#include "driverlib/ssi.h"


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


static void
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

static void
config_spi(void)
{
  /*
    Configure the SPI for correct mode to read from nRF24L01+.

    We need CLK inactive low, so SPO=0.
    We need to setup and sample on the leading, rising CLK edge, so SPH=0.

    The datasheet says up to 10MHz SPI is possible, depending on load
    capacitance. Let's go with a slightly cautious 8MHz, which should be
    aplenty.
  */

  uint32_t base = NRF_SSI_BASE;
  ROM_SSIDisable(base);
  ROM_SSIConfigSetExpClk(base, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                         SSI_MODE_MASTER, 8000000, 8);
  ROM_SSIEnable(base);
}


static void
config_udma_for_spi(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
  ROM_uDMAEnable();
  ROM_uDMAControlBaseSet(udma_control_block);

  ROM_SSIDMAEnable(NRF_SSI_BASE, SSI_DMA_TX);
  ROM_SSIDMAEnable(NRF_SSI_BASE, SSI_DMA_RX);
  ROM_IntEnable(INT_SSI1);

  ROM_uDMAChannelAttributeDisable(NRF_DMA_CH_TX, UDMA_ATTR_ALTSELECT |
                                  UDMA_ATTR_REQMASK | UDMA_ATTR_HIGH_PRIORITY);
  ROM_uDMAChannelAssign(UDMA_CH25_SSI1TX);
  ROM_uDMAChannelAttributeEnable(NRF_DMA_CH_TX, UDMA_ATTR_USEBURST);
  ROM_uDMAChannelControlSet(NRF_DMA_CH_TX | UDMA_PRI_SELECT,
                            UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
                            UDMA_ARB_4);

  ROM_uDMAChannelAttributeDisable(NRF_DMA_CH_RX, UDMA_ATTR_ALTSELECT |
                                  UDMA_ATTR_REQMASK | UDMA_ATTR_HIGH_PRIORITY);
  ROM_uDMAChannelAssign(UDMA_CH24_SSI1RX);
  ROM_uDMAChannelAttributeEnable(NRF_DMA_CH_RX, UDMA_ATTR_USEBURST);
  ROM_uDMAChannelControlSet(NRF_DMA_CH_RX | UDMA_PRI_SELECT,
                            UDMA_SIZE_8 | UDMA_DST_INC_8 | UDMA_SRC_INC_NONE |
                            UDMA_ARB_4);
}


void
setup_nrf(void)
{
  config_ssi_gpio();
  config_spi();
  config_udma_for_spi();
}
