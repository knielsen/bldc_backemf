#include "nrf24l01p.h"

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

/*
  In legacy 2D POV, we have dual-buffering of 65x65 pixels of framebuffer data.
  But this version is just for sending keypresses and wireless-flashing data.
  So we can use significantly smaller buffer here.
*/
#define BUF_SIZE 512
#define BUF_SIZE_PAD (32*((BUF_SIZE+30)/31))
#define BURSTSIZE 24

/* Total number of times a key is transmittet (for redundancy). */
#define KEY_RETRANSMITS 8

#define TRANSMIT_RUNNING_NO 0
#define TRANSMIT_RUNNING_FRAMEDATA 1
#define TRANSMIT_RUNNING_KEYPRESSES 2
extern volatile uint8_t transmit_running;

extern uint8_t nrf_frame_buffers[2][BUF_SIZE_PAD];
extern volatile uint32_t nrf_write_idx;

static inline void
nrf_csn_low(uint32_t csn_base, uint32_t csn_pin)
{
  my_gpio_write(csn_base, csn_pin, 0);
}


static inline void
nrf_csn_high(uint32_t csn_base, uint32_t csn_pin)
{
  my_gpio_write(csn_base, csn_pin, csn_pin);
}


static inline void
nrf_ce_low(uint32_t ce_base, uint32_t ce_pin)
{
  my_gpio_write(ce_base, ce_pin, 0);
}


static inline void
nrf_ce_high(uint32_t ce_base, uint32_t ce_pin)
{
  my_gpio_write(ce_base, ce_pin, ce_pin);
}


extern const char * nrf_transmit_packet(uint8_t *packet);
extern void nrf_transmit_packet_nack(uint8_t *packet);
extern void nrf_config_normal_tx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin);
extern void nrf_config_bootload_tx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin);
extern void nrf_config_bootload_rx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin);
extern uint32_t nrf_get_status(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin);
extern void nrf_rx(uint8_t *data, uint32_t len,
                   uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin);
extern void nrf_startup(void);
extern void transmit_start_keypresses(uint32_t howmuch);
extern void transmit_start_framedata(void);
