#include <string.h>

#include "bldc_backemf.h"
#include "dbg.h"
#include "nrf.h"

#include "inc/hw_ssi.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "driverlib/udma.h"
#include "driverlib/ssi.h"
#include "driverlib/timer.h"


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


static void
config_timer(void)
{
  /*
    Configure timer interrupt, used to put a small delay between transmit
    bursts.
  */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
  /* Configure 2 * 16-bit timer, A periodic. */
  ROM_TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR |
                     TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_ONE_SHOT);
}


static void
bzero(uint8_t *buf, uint32_t len)
{
  while (len > 0)
  {
    *buf++ = 0;
    --len;
  }
}


static void
ssi_cmd(uint8_t *recvbuf, const uint8_t *sendbuf, uint32_t len,
        uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint32_t i;
  uint32_t data;

  /* Take CSN low to initiate transfer. */
  nrf_csn_low(csn_base, csn_pin);

  for (i = 0; i < len; ++i)
  {
    ROM_SSIDataPut(ssi_base, sendbuf[i]);
    while (ROM_SSIBusy(ssi_base))
      ;
    ROM_SSIDataGet(ssi_base, &data);
    recvbuf[i] = data;
  }

  /* Take CSN high to complete transfer. */
  nrf_csn_high(csn_base, csn_pin);
}


void
nrf_rx(uint8_t *data, uint32_t len,
       uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[33], recvbuf[33];

  if (len > NRF_PACKET_SIZE)
    len = NRF_PACKET_SIZE;
  sendbuf[0] = nRF_R_RX_PAYLOAD;
  bzero(&sendbuf[1], len);
  ssi_cmd(recvbuf, sendbuf, len+1, ssi_base, csn_base, csn_pin);
  memcpy(data, &recvbuf[1], len);
}


static void
nrf_tx(uint8_t *data, uint32_t len,
       uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[33], recvbuf[33];

  if (len > NRF_PACKET_SIZE)
    len = NRF_PACKET_SIZE;
  sendbuf[0] = nRF_W_TX_PAYLOAD;
  memcpy(&sendbuf[1], data, len);
  ssi_cmd(recvbuf, sendbuf, len+1, ssi_base, csn_base, csn_pin);
}


static void
nrf_tx_nack(uint8_t *data, uint32_t len,
            uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[33], recvbuf[33];

  if (len > NRF_PACKET_SIZE)
    len = NRF_PACKET_SIZE;
  sendbuf[0] = nRF_W_TX_PAYLOAD_NOACK;
  memcpy(&sendbuf[1], data, len);
  ssi_cmd(recvbuf, sendbuf, len+1, ssi_base, csn_base, csn_pin);
}


/*
  Asynchronous SSI transfer to nRF24L01+ using uDMA.
  Performs a transaction over SPI.
  First call nrf_async_start(). While nrf_async_start() or nrf_async_cont()
  returns zero, call nrf_async_cont() for each SSI interrupt event (either
  signifying DMA or transfer done).
*/
struct nrf_async_dma {
  uint32_t ssi_base;
  uint32_t dma_rx_chan;
  uint32_t dma_tx_chan;
  uint32_t csn_base;
  uint32_t csn_pin;
  uint8_t dma_rx_running;
  uint8_t dma_tx_running;
  uint8_t ssi_active;
};


static int32_t
nrf_async_dma_start(struct nrf_async_dma *a, uint8_t *recvbuf, uint8_t *sendbuf,
                    uint32_t len, uint32_t ssi_base, uint32_t dma_rx_chan,
                    uint32_t dma_tx_chan, uint32_t csn_base, uint32_t csn_pin)
{
  a->ssi_base = ssi_base;
  a->dma_rx_chan = dma_rx_chan;
  a->dma_tx_chan = dma_tx_chan;
  a->csn_base = csn_base;
  a->csn_pin = csn_pin;
  ROM_uDMAChannelTransferSet(dma_rx_chan | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
                             (void *)(ssi_base + SSI_O_DR), recvbuf, len);
  ROM_uDMAChannelTransferSet(dma_tx_chan | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
                             sendbuf, (void *)(ssi_base + SSI_O_DR), len);
  a->dma_rx_running = 1;
  a->dma_tx_running = 1;
  a->ssi_active = 1;
  /* Take CSN low to initiate transfer. */
  nrf_csn_low(csn_base, csn_pin);
  /* Empty any left-over stuff in the RX FIFO. */
  while (HWREG(ssi_base + SSI_O_SR) & SSI_SR_RNE)
    (void)(HWREG(ssi_base + SSI_O_DR));
  ROM_uDMAChannelEnable(dma_rx_chan);
  ROM_uDMAChannelEnable(dma_tx_chan);
  return 0;
}


static int32_t
nrf_async_dma_cont(struct nrf_async_dma *a)
{
  /*
    There are no pending interrupt requests to clear here.
    The only interrupt we handle is SSI_TXFF (TX FIFO empty), and that
    cannot be cleared (except by putting stuff in the FIFO). Rather, we
    unmask and mask it as appropriate.
  */
  if (a->dma_tx_running && !ROM_uDMAChannelIsEnabled(a->dma_tx_chan))
  {
    a->dma_tx_running = 0;
    /*
      Enable interrupt at end of transfer.

      Things did not work for me unless I delayed setting EOT to here (rather
      that doing it up-front). My guess is that setting EOT prevents not only
      the interrupt at half-full fifo, but also the dma request, causing send
      to stall, but I did not investigate fully.

      Also, let's clear the TX interrupt, if not I seemed to get a spurious
      interrupt, probably due to the fifo being half-full at this point.

      Note that then I also need to check for EOT already now in this
      interrupt activation, to avoid a race where EOT could have occured
      already due to delayed interrupt execution.
    */
    HWREG(a->ssi_base + SSI_O_CR1) |= SSI_CR1_EOT;
    HWREG(a->ssi_base + SSI_O_IM) |= SSI_TXFF;
  }
  if (!ROM_SSIBusy(a->ssi_base))
  {
    a->ssi_active = 0;
    /*
      Now that the transfer is completely done, set the TX interrupt back
      to disabled and trigger at 1/2 full fifo.
    */
    HWREG(a->ssi_base + SSI_O_IM) &= ~SSI_TXFF;
    HWREG(a->ssi_base + SSI_O_CR1) &= ~SSI_CR1_EOT;
  }
  if (a->dma_rx_running && !ROM_uDMAChannelIsEnabled(a->dma_rx_chan))
    a->dma_rx_running = 0;
  if (a->dma_tx_running || a->ssi_active || a->dma_rx_running)
    return 0;
  else
  {
    /* Take CSN high to complete transfer. */
    nrf_csn_high(a->csn_base, a->csn_pin);
    return 1;
  }
}


/*
  Asynchronous SSI transfer to nRF24L01+ using only fifo (no dma).
  Performs a transaction over SPI <= 8 bytes.
  First call nrf_async_start(). While nrf_async_start() or nrf_async_cont()
  returns zero, call nrf_async_cont() for each SSI interrupt event (only
  transfer done can occur).
*/
struct nrf_async_cmd {
  uint32_t ssi_base;
  uint32_t csn_base;
  uint32_t csn_pin;
  uint8_t *recvbuf;
  uint8_t ssi_active;
};


/*
  Start a command, max 8 bytes.
  recvbuf must remain valid for the duration of the operation, or it can be
  NULL to ignore anything received.
  sendbuf can be discarded once nrf_async_cmd_start() returns.
*/
static int32_t
nrf_async_cmd_start(struct nrf_async_cmd *a,
                    uint8_t *recvbuf, const uint8_t *sendbuf, uint32_t len,
                    uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  if (len > 8)
    return -1;
  a->ssi_base = ssi_base;
  a->csn_base = csn_base;
  a->csn_pin = csn_pin;
  a->recvbuf = recvbuf;
  a->ssi_active = 1;
  /* Take CSN low to initiate transfer. */
  nrf_csn_low(csn_base, csn_pin);
  /* Empty any left-over stuff in the RX FIFO. */
  while (HWREG(ssi_base + SSI_O_SR) & SSI_SR_RNE)
    (void)(HWREG(ssi_base + SSI_O_DR));

  /* Set up so we get an interrupt when last bit has been transmitted. */
  HWREG(ssi_base + SSI_O_CR1) |= SSI_CR1_EOT;
  HWREG(ssi_base + SSI_O_IM) |= SSI_TXFF;

  while (len > 0)
  {
    HWREG(ssi_base + SSI_O_DR) = *sendbuf++;
    --len;
  }

  return 0;
}


static int32_t
nrf_async_cmd_cont(struct nrf_async_cmd *a)
{
  uint8_t *p;

  if (!ROM_SSIBusy(a->ssi_base))
  {
    a->ssi_active = 0;
    /*
      Now that the transfer is completely done, set the TX interrupt back
      to disabled and trigger at 1/2 full fifo.
    */
    HWREG(a->ssi_base + SSI_O_IM) &= ~SSI_TXFF;
    HWREG(a->ssi_base + SSI_O_CR1) &= ~SSI_CR1_EOT;

    /* Take CSN high to complete transfer. */
    nrf_csn_high(a->csn_base, a->csn_pin);

    /* Empty the receive fifo (and return the data if so requested. */
    p = a->recvbuf;
    while (HWREG(a->ssi_base + SSI_O_SR) & SSI_SR_RNE)
    {
      uint8_t v = HWREG(a->ssi_base + SSI_O_DR);
      if (p)
        *p++ = v;
    }
    return 1;
  }
  else
    return 0;
}


static void
nrf_flush_tx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t cmd = nRF_FLUSH_TX;
  uint8_t status;
  ssi_cmd(&status, &cmd, 1, ssi_base, csn_base, csn_pin);
}


static void
nrf_flush_rx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t cmd = nRF_FLUSH_RX;
  uint8_t status;
  ssi_cmd(&status, &cmd, 1, ssi_base, csn_base, csn_pin);
}


static void
nrf_write_reg_n(uint8_t reg, const uint8_t *data, uint32_t len,
                uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[6], recvbuf[6];
  if (len > 5)
    len = 5;
  sendbuf[0] = nRF_W_REGISTER | reg;
  memcpy(&sendbuf[1], data, len);
  ssi_cmd(recvbuf, sendbuf, len+1, ssi_base, csn_base, csn_pin);
}


static void
nrf_write_reg(uint8_t reg, uint8_t val,
              uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  nrf_write_reg_n(reg, &val, 1, ssi_base, csn_base, csn_pin);
}


static int32_t
nrf_write_reg_n_start(struct nrf_async_cmd *a, uint8_t reg, const uint8_t *data,
                      uint8_t *recvbuf, uint32_t len, uint32_t ssi_base,
                      uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[8];
  if (len > 7)
    len = 7;
  sendbuf[0] = nRF_W_REGISTER | reg;
  memcpy(&sendbuf[1], data, len);
  return nrf_async_cmd_start(a, recvbuf, sendbuf, len+1, ssi_base,
                             csn_base, csn_pin);
}


static int32_t
nrf_write_reg_start(struct nrf_async_cmd *a, uint8_t reg, uint8_t val,
                    uint8_t *recvbuf,
                    uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  return nrf_write_reg_n_start(a, reg, &val, recvbuf, 1,
                               ssi_base, csn_base, csn_pin);
}


static void
nrf_read_reg_n(uint8_t reg, uint8_t *out, uint32_t len,
               uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[6];
  if (len > 5)
    len = 5;
  sendbuf[0] = nRF_R_REGISTER | reg;
  bzero(&sendbuf[1], len);
  ssi_cmd(out, sendbuf, len+1, ssi_base, csn_base, csn_pin);
}


static uint8_t
nrf_read_reg(uint8_t reg, uint8_t *status_ptr,
             uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t recvbuf[2];
  nrf_read_reg_n(reg, recvbuf, 2, ssi_base, csn_base, csn_pin);
  if (status_ptr)
    *status_ptr = recvbuf[0];
  return recvbuf[1];
}


static int32_t
nrf_read_reg_n_start(struct nrf_async_cmd *a, uint8_t reg, uint8_t *recvbuf,
                     uint32_t len, uint32_t ssi_base,
                     uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[8];
  if (len > 7)
    len = 7;
  sendbuf[0] = nRF_R_REGISTER | reg;
  bzero(&sendbuf[1], len);
  return nrf_async_cmd_start(a, recvbuf, sendbuf, len+1, ssi_base,
                             csn_base, csn_pin);
}


static int32_t
nrf_read_reg_start(struct nrf_async_cmd *a, uint8_t reg, uint8_t *recvbuf,
                   uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  return nrf_read_reg_n_start(a, reg, recvbuf, 1,
                              ssi_base, csn_base, csn_pin);
}


/*
  Transmit a number of packets back-to-back with the nRF24L01+.

  The packets are filled in by a call-back function. This callback is invoked
  from interrupt context, so it should ideally be fairly quick and has to be
  aware of the general interrupt caveats. A zero return from the callback
  causes termination of the burst after that packet (irrespectively of count).
*/
struct nrf_async_transmit_multi {
  int32_t (*fillpacket)(uint8_t *, void *);
  void *cb_data;
  uint32_t ssi_base;
  uint32_t dma_rx_chan, dma_tx_chan;
  uint32_t csn_base, csn_pin;
  uint32_t ce_base, ce_pin;
  uint32_t irq_base, irq_pin;
  uint32_t remain;
  union {
    struct nrf_async_dma dma;
    struct nrf_async_cmd cmd;
  } substate;
  uint8_t sendbuf[33];
  uint8_t recvbuf[33];
  uint8_t transmit_packet_running;
  uint8_t nrf_cmd_running;
  uint8_t ce_asserted;
  uint8_t state;
};
enum nrf_async_transmit_multi_states {
  ST_NRF_ATM_WRITE_TO_FIFO,
  ST_NRF_ATM_CLEAR_DS,
  ST_NRF_ATM_CHECK_FIFO_ROOM,
  ST_NRF_ATM_WAIT_FOR_DONE,
  ST_NRF_ATM_CHECK_FIFO_EMPTY,
  ST_NRF_ATM_CHECK_IF_DONE,
};

static int32_t
nrf_async_transmit_multi_cont(struct nrf_async_transmit_multi *a,
                              uint32_t is_ssi_event);
static int32_t
nrf_async_transmit_multi_start(struct nrf_async_transmit_multi *a,
                               int32_t (*fillpacket)(uint8_t *, void *),
                               void *cb_data, uint32_t count, uint32_t ssi_base,
                               uint32_t dma_rx_chan, uint32_t dma_tx_chan,
                               uint32_t csn_base, uint32_t csn_pin,
                               uint32_t ce_base, uint32_t ce_pin,
                               uint32_t irq_base, uint32_t irq_pin)
{
  a->fillpacket = fillpacket;
  a->cb_data = cb_data;
  a->ssi_base = ssi_base;
  a->dma_rx_chan = dma_rx_chan;
  a->dma_tx_chan = dma_tx_chan;
  a->csn_base = csn_base;
  a->csn_pin = csn_pin;
  a->ce_base = ce_base;
  a->ce_pin = ce_pin;
  a->irq_base = irq_base;
  a->irq_pin = irq_pin;
  a->remain = count;
  a->transmit_packet_running = 0;
  a->nrf_cmd_running = 0;
  a->ce_asserted = 0;
  a->state = ST_NRF_ATM_WRITE_TO_FIFO;
  return nrf_async_transmit_multi_cont(a, 0);
}


/*
  This is called to continue after a 4 microseconds delay needed from taking
  CE high to starting SPI operations.
*/
static void
nrf_async_pece_delay_cb(void *data)
{
  struct nrf_async_transmit_multi *a = data;
  nrf_async_transmit_multi_cont(a, 0);
}


static void timer2a_set_delay(uint32_t cycles, void (*cb)(void *), void *cb_data);

/*
  Called to continue a multi-packet back-to-back write session.
  This should be called when an event occurs, either in the form of
  an SSI interrupt or in the form of a GPIO interrupt on the nRF24L01+
  IRQ pin (the is_ssi_event flag tells which).

  The two interrupts should be configured to have the same priority, so that
  one of them does not attempt to pre-empt the other; that would lead to
  nasty races.
*/
static int32_t
nrf_async_transmit_multi_cont(struct nrf_async_transmit_multi *a,
                              uint32_t is_ssi_event)
{
  if (is_ssi_event && a->transmit_packet_running)
  {
    if (nrf_async_dma_cont(&a->substate.dma))
      a->transmit_packet_running = 0;
    else
      return 0;
  }
  else if (is_ssi_event && a->nrf_cmd_running)
  {
    if (nrf_async_cmd_cont(&a->substate.cmd))
      a->nrf_cmd_running = 0;
    else
      return 0;
  }

resched:
  switch (a->state)
  {
  case ST_NRF_ATM_WRITE_TO_FIFO:
    /*
      In this state, there is room in the transmit fifo of the nRF24L01+.
      So start an SPI transaction to inject another packet (or prepare to
      finish if all packets sent).
    */
    if (a->remain == 0)
    {
      /* All sent, go wait for FIFO to empty. */
      a->state = ST_NRF_ATM_WAIT_FOR_DONE;
      goto resched;
    }
    --a->remain;
    a->sendbuf[0] = nRF_W_TX_PAYLOAD_NOACK;
    if (!(*(a->fillpacket))(&(a->sendbuf[1]), a->cb_data))
      a->remain = 0;                            /* Callback signalled stop. */
    a->transmit_packet_running = 1;
    nrf_async_dma_start(&a->substate.dma, a->recvbuf, a->sendbuf, 33,
                        a->ssi_base, a->dma_rx_chan, a->dma_tx_chan,
                        a->csn_base, a->csn_pin);
    a->state = ST_NRF_ATM_CLEAR_DS;
    return 0;

  case ST_NRF_ATM_CLEAR_DS:
    /*
      In this state, we clear any TX_DS flag, and in the process get back
      the STATUS register so we can check if there is room in the TX FIFO
      of the nRF24L01+.
    */
    ROM_GPIOPinIntDisable(a->irq_base, a->irq_pin);
    /* Once we have put stuff into the FIFO, assert CE to start sending. */
    if (!a->ce_asserted)
    {
      nrf_ce_high(a->ce_base, a->ce_pin);
      a->ce_asserted = 1;
      /*
        nRF24L01+ datasheet says that there must be at least 4 microseconds
        from a positive edge on CE to CSN being taken low.
      */
      timer2a_set_delay(MCU_HZ*4/1000000, nrf_async_pece_delay_cb, a);
      return 0;
    }
    /*
      Now clear the TX_DS flag, and at the same time get the STATUS register
      to see if there is room for more packets in the TX FIFO.
    */
    a->nrf_cmd_running = 1;
    nrf_write_reg_start(&a->substate.cmd, nRF_STATUS, nRF_TX_DS, a->recvbuf,
                        a->ssi_base, a->csn_base, a->csn_pin);
    a->state = ST_NRF_ATM_CHECK_FIFO_ROOM;
    return 0;

  case ST_NRF_ATM_CHECK_FIFO_ROOM:
    /*
      In this state, we have received the value on STATUS in recvbuf[0].
      Checking the TX_FULL flag, we decide whether to inject another
      packet or wait for one to be sent first.
    */
    if (a->recvbuf[0] & nRF_TX_FULL)
    {
      /*
        We have managed to fill up the TX FIFO.
        Now enable interrupt on the IRQ pin. This will trigger when one more
        packet gets sent (TX_DS) so that there is more room in the FIFO.

        Once we get that interrupt, we will again clear the TX_DS flag and
        fill in as many packets in the TX FIFO as will fit.
      */
      a->state = ST_NRF_ATM_CLEAR_DS;
      ROM_GPIOPinIntEnable(a->irq_base, a->irq_pin);
      return 0;
      /*
        I suppose there is a small race here, if the DS flag got asserted
        just before we clear it. It does not really matter, we still have two
        packets in the TX fifo, so we have time to inject more once one of
        them gets sent and re-assert the DS flag.
      */
    }

    /* There is room for (at least) one more packet in the FIFO. */
    a->state = ST_NRF_ATM_WRITE_TO_FIFO;
    goto resched;

  case ST_NRF_ATM_WAIT_FOR_DONE:
    /*
      Clear any TX_DS flag. After that we will check FIFO_STATUS
      and either stop if it is empty, or wait for another TX_DS if it
      is not.
    */
    ROM_GPIOPinIntDisable(a->irq_base, a->irq_pin);
    a->nrf_cmd_running = 1;
    nrf_write_reg_start(&a->substate.cmd, nRF_STATUS, nRF_TX_DS, NULL,
                        a->ssi_base, a->csn_base, a->csn_pin);
    a->state = ST_NRF_ATM_CHECK_FIFO_EMPTY;
    return 0;

  case ST_NRF_ATM_CHECK_FIFO_EMPTY:
    /*
      We have cleared TX_DS. Now send a FIFO_STATUS. Either the FIFO_STATUS
      will show that the TX FIFO is now empty, or we will get an IRQ on a
      new TX_DS when a packet has been sent from the FIFO.
    */
    a->nrf_cmd_running = 1;
    nrf_read_reg_start(&a->substate.cmd, nRF_FIFO_STATUS, a->recvbuf,
                       a->ssi_base, a->csn_base, a->csn_pin);
    a->state = ST_NRF_ATM_CHECK_IF_DONE;
    return 0;

  case ST_NRF_ATM_CHECK_IF_DONE:
    if (!(a->recvbuf[1] & nRF_TX_EMPTY))
    {
      /*
        There is still data in the FIFO. Wait for IRQ line to be asserted
        marking another transmit completed, and then check again.
      */
      a->state = ST_NRF_ATM_WAIT_FOR_DONE;
      ROM_GPIOPinIntEnable(a->irq_base, a->irq_pin);
      return 0;
    }

    /*
      Now the TX FIFO is empty, we can de-assert CE, then the nRF24L01+ will
      finish transmitting any last packet, and then go to standby mode.
    */
    nrf_ce_low(a->ce_base, a->ce_pin);
    a->ce_asserted = 0;
    return 1;

  default:
    /* This shouldn't really happen ... */
    return 0;
  }
}


static const uint8_t nrf_addr[3] = { 0x7e, 0xc8, 0x33 };

/*
  Configure nRF24L01+ as Rx or Tx.
    channel - radio frequency channel to use, 0 <= channel <= 127.
    power - nRF_RF_PWR_<X>DBM, <X> is 0, 6, 12, 18 dBm.
*/
void
nrf_init_config(uint8_t is_rx, uint32_t channel, uint32_t power,
                uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  if (is_rx)
    nrf_write_reg(nRF_CONFIG, nRF_PRIM_RX | nRF_MASK_TX_DS |
                  nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP,
                  ssi_base, csn_base, csn_pin);
  else
    nrf_write_reg(nRF_CONFIG, nRF_MASK_RX_DR |
                  nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP,
                  ssi_base, csn_base, csn_pin);
  /* Disable auto-ack, saving 9 bits/packet. Else 0x3f. */
  nrf_write_reg(nRF_EN_AA, 0, ssi_base, csn_base, csn_pin);
  /* Enable only pipe 0. */
  nrf_write_reg(nRF_EN_RXADDR, nRF_ERX_P0, ssi_base, csn_base, csn_pin);
  /* 3 byte adresses. */
  nrf_write_reg(nRF_SETUP_AW, nRF_AW_3BYTES, ssi_base, csn_base, csn_pin);
  /* Disable auto retransmit. */
  nrf_write_reg(nRF_SETUP_RETR, 0, ssi_base, csn_base, csn_pin);
  nrf_write_reg(nRF_RF_CH, channel, ssi_base, csn_base, csn_pin);
  /* Use 2Mbps, and set transmit power. */
  nrf_write_reg(nRF_RF_SETUP, nRF_RF_DR_LOW | power,
                ssi_base, csn_base, csn_pin);
  nrf_write_reg_n(nRF_RX_ADDR_P0, nrf_addr, 3, ssi_base, csn_base, csn_pin);
  nrf_write_reg_n(nRF_TX_ADDR, nrf_addr, 3, ssi_base, csn_base, csn_pin);
  /* Set payload size for pipe 0. */
  nrf_write_reg(nRF_RX_PW_P0, NRF_PACKET_SIZE, ssi_base, csn_base, csn_pin);
  /* Disable pipe 1-5. */
  nrf_write_reg(nRF_RX_PW_P1, 0, ssi_base, csn_base, csn_pin);
  /* Disable dynamic payload length. */
  nrf_write_reg(nRF_DYNDP, 0, ssi_base, csn_base, csn_pin);
  /* Allow disabling acks. */
  nrf_write_reg(nRF_FEATURE, nRF_EN_DYN_ACK, ssi_base, csn_base, csn_pin);

  /* Clear out all FIFOs. */
  nrf_flush_tx(ssi_base, csn_base, csn_pin);
  nrf_flush_rx(ssi_base, csn_base, csn_pin);
  /* Clear the IRQ bits in STATUS register. */
  nrf_write_reg(nRF_STATUS, nRF_RX_DR|nRF_TX_DS|nRF_MAX_RT,
                ssi_base, csn_base, csn_pin);
}


/*
  Configure nRF24L01+ as Tx for streaming frame buffer data back-to-back
  as fast as possible.

  The nRF24L01+ is configured as transmitter, with auto-ack and
  retransmission disabled.
*/
void
nrf_config_normal_tx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  nrf_write_reg(nRF_CONFIG, nRF_MASK_RX_DR |
                  nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP,
                  ssi_base, csn_base, csn_pin);
  /* Disable auto-ack, saving 9 bits/packet. Else 0x3f. */
  nrf_write_reg(nRF_EN_AA, 0, ssi_base, csn_base, csn_pin);
  /* Disable auto retransmit. */
  nrf_write_reg(nRF_SETUP_RETR, 0, ssi_base, csn_base, csn_pin);

  /* Clear out all FIFOs. */
  nrf_flush_tx(ssi_base, csn_base, csn_pin);
  nrf_flush_rx(ssi_base, csn_base, csn_pin);
  /* Clear the IRQ bits in STATUS register. */
  nrf_write_reg(nRF_STATUS, nRF_RX_DR|nRF_TX_DS|nRF_MAX_RT,
                ssi_base, csn_base, csn_pin);
}


/*
  Configure nRF24L01+ as Tx for talking to the bootloader.

  The nRF24L01+ is configured as transmitter, with auto-ack and
  retransmission enabled.
*/
void
nrf_config_bootload_tx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  nrf_write_reg(nRF_CONFIG, nRF_MASK_RX_DR |
                  nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP,
                  ssi_base, csn_base, csn_pin);
  /* Enable auto-ack. */
  nrf_write_reg(nRF_EN_AA, nRF_ENAA_P0|nRF_ENAA_P1|nRF_ENAA_P2|
                           nRF_ENAA_P3|nRF_ENAA_P4|nRF_ENAA_P5,
                ssi_base, csn_base, csn_pin);
  /* Enable auto retransmit. */
  nrf_write_reg(nRF_SETUP_RETR, (1 << nRF_ARD_SHIFT) | 15,
                ssi_base, csn_base, csn_pin);

  /* Clear out all FIFOs. */
  nrf_flush_tx(ssi_base, csn_base, csn_pin);
  nrf_flush_rx(ssi_base, csn_base, csn_pin);
  /* Clear the IRQ bits in STATUS register. */
  nrf_write_reg(nRF_STATUS, nRF_RX_DR|nRF_TX_DS|nRF_MAX_RT,
                ssi_base, csn_base, csn_pin);
}


/*
  Configure nRF24L01+ as Rx for getting reply packet from the bootloader.

  The nRF24L01+ is configured as receiver, with auto-ack and
  retransmission enabled.
*/
void
nrf_config_bootload_rx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  nrf_write_reg(nRF_CONFIG, nRF_PRIM_RX | nRF_MASK_TX_DS |
                  nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP,
                  ssi_base, csn_base, csn_pin);
  /* Enable auto-ack. */
  nrf_write_reg(nRF_EN_AA, nRF_ENAA_P0|nRF_ENAA_P1|nRF_ENAA_P2|
                           nRF_ENAA_P3|nRF_ENAA_P4|nRF_ENAA_P5,
                ssi_base, csn_base, csn_pin);
  /* Enable auto retransmit. */
  nrf_write_reg(nRF_SETUP_RETR, (1 << nRF_ARD_SHIFT) | 15,
                ssi_base, csn_base, csn_pin);

  /* Clear out all FIFOs. */
  nrf_flush_tx(ssi_base, csn_base, csn_pin);
  nrf_flush_rx(ssi_base, csn_base, csn_pin);
  /* Clear the IRQ bits in STATUS register. */
  nrf_write_reg(nRF_STATUS, nRF_RX_DR|nRF_TX_DS|nRF_MAX_RT,
                ssi_base, csn_base, csn_pin);
}


static void
config_interrupts(void)
{
  ROM_GPIOIntTypeSet(NRF_IRQ_BASE, NRF_IRQ_PIN, GPIO_LOW_LEVEL);
  ROM_IntMasterEnable();
  ROM_IntEnable(INT_GPIOB);
  ROM_IntEnable(INT_TIMER2A);
}


/* nRF24L01+ communications. */

static uint32_t write_sofar;
static uint32_t write_howmuch;
static struct nrf_async_transmit_multi transmit_multi_state;
static volatile uint8_t transmit_multi_running = 0;
volatile uint8_t transmit_running = TRANSMIT_RUNNING_NO;


static void (* volatile timer2a_cb)(void *);
static void * volatile timer2a_cb_data;

/* Request a callback to be called after a certain number of cycles passed. */
static void
timer2a_set_delay(uint32_t cycles, void (*cb)(void *), void *cb_data)
{
  timer2a_cb = cb;
  timer2a_cb_data = cb_data;
  ROM_TimerLoadSet(TIMER2_BASE, TIMER_A, cycles);
  ROM_TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
  my_enable_timera(TIMER2_BASE);
}


void
IntHandlerTimer2A(void)
{
  void (*cb)(void *) = timer2a_cb;

  my_clear_timera_timeout_int(TIMER2_BASE);
  my_disable_timera(TIMER2_BASE);
  ROM_TimerIntDisable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
  if (cb)
  {
    (*cb)(timer2a_cb_data);
    timer2a_cb = NULL;
  }
}


static void start_next_burst(void *);

static void
handle_end_of_transmit_burst(void)
{
  transmit_multi_running = 0;
  if (write_sofar < write_howmuch)
  {
    /*
      Wait 20 microseconds, then start another write burst.
      The nRF24L01+ datasheet says that we must not stay in transmit mode
      for more than 4 milliseconds (which is just a bit more than the time
      to send 24 full packets back-to-back).
    */
    timer2a_set_delay(MCU_HZ/1000000*20, start_next_burst, NULL);
  }
  else
    transmit_running = TRANSMIT_RUNNING_NO;
}


void
IntHandlerGPIOb(void)
{
  uint32_t irq_status = HWREG(GPIO_PORTB_BASE + GPIO_O_MIS) & 0xff;
  if (irq_status & NRF_IRQ_PIN)
  {
    if (transmit_multi_running)
    {
      if (nrf_async_transmit_multi_cont(&transmit_multi_state, 0))
        handle_end_of_transmit_burst();
    }
    else
    {
      /*
        Clear the interrupt request and disable further interrupts until we can
        clear the request from the device over SPI.
      */
      HWREG(GPIO_PORTB_BASE + GPIO_O_IM) &= ~NRF_IRQ_PIN & 0xff;
      HWREG(GPIO_PORTB_BASE + GPIO_O_ICR) = NRF_IRQ_PIN;

      serial_output_str("Tx: IRQ: TX_DS (spurious)\r\n");
    }
  }
}


void
IntHandlerSSI1(void)
{
  if (transmit_multi_running &&
      nrf_async_transmit_multi_cont(&transmit_multi_state, 1))
    handle_end_of_transmit_burst();
}


uint8_t nrf_frame_buffers[2][BUF_SIZE_PAD];
volatile uint32_t nrf_write_idx;


static int32_t
my_fill_packet(uint8_t *buf, void *d)
{
  uint32_t i;
  uint8_t *src = &nrf_frame_buffers[nrf_write_idx][write_sofar];

  for (i = 0; i < NRF_PACKET_SIZE; ++i)
    *buf++ = *src++;
  write_sofar += NRF_PACKET_SIZE;
  return (write_sofar < write_howmuch);
}


/* Start the next write burst (unless we are done). */
static void start_next_burst(void *dummy)
{
  if (write_sofar < write_howmuch)
  {
    transmit_multi_running = 1;
    nrf_async_transmit_multi_start(&transmit_multi_state, my_fill_packet,
                                   NULL, BURSTSIZE, NRF_SSI_BASE,
                                   NRF_DMA_CH_RX, NRF_DMA_CH_TX,
                                   NRF_CSN_BASE, NRF_CSN_PIN,
                                   NRF_CE_BASE, NRF_CE_PIN,
                                   NRF_IRQ_BASE, NRF_IRQ_PIN);
  }
  else
    transmit_running = TRANSMIT_RUNNING_NO;
}


void
transmit_start_framedata(void)
{
  transmit_running = TRANSMIT_RUNNING_FRAMEDATA;
  write_howmuch = BUF_SIZE_PAD;
  write_sofar = 0;
  transmit_multi_running = 1;
  nrf_async_transmit_multi_start(&transmit_multi_state, my_fill_packet,
                                 NULL, BURSTSIZE, NRF_SSI_BASE,
                                 NRF_DMA_CH_RX, NRF_DMA_CH_TX,
                                 NRF_CSN_BASE, NRF_CSN_PIN,
                                 NRF_CE_BASE, NRF_CE_PIN,
                                 NRF_IRQ_BASE, NRF_IRQ_PIN);
}


void
transmit_start_keypresses(uint32_t howmuch)
{
  transmit_running = TRANSMIT_RUNNING_KEYPRESSES;
  write_howmuch = howmuch;
  write_sofar = 0;
  transmit_multi_running = 1;
  nrf_async_transmit_multi_start(&transmit_multi_state, my_fill_packet,
                                 NULL, BURSTSIZE, NRF_SSI_BASE,
                                 NRF_DMA_CH_RX, NRF_DMA_CH_TX,
                                 NRF_CSN_BASE, NRF_CSN_PIN,
                                 NRF_CE_BASE, NRF_CE_PIN,
                                 NRF_IRQ_BASE, NRF_IRQ_PIN);
}


/*
  Read both the normal and FIFO status registers.
  Returns normal status or'ed with (fifo status left-shifted 8).
*/
uint32_t
nrf_get_status(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t status;
  uint32_t fifo_status;

  fifo_status =
    nrf_read_reg(nRF_FIFO_STATUS, &status, ssi_base, csn_base, csn_pin);
  return (fifo_status << 8) | status;
}


void
nrf_transmit_packet_nack(uint8_t *packet)
{
  uint32_t start_time = get_time();

  nrf_tx_nack(packet, NRF_PACKET_SIZE, NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
  nrf_ce_high(NRF_CE_BASE, NRF_CE_PIN);
  delay_us(10);
  nrf_ce_low(NRF_CE_BASE, NRF_CE_PIN);

  for (;;)
  {
    uint32_t status = nrf_get_status(NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
    if (status & (nRF_TX_DS|nRF_MAX_RT))
      break;
    if (calc_time(start_time) > MCU_HZ/20)
      break;
  }
  /* Clear the data sent / max retries flags. */
  nrf_write_reg(nRF_STATUS, nRF_TX_DS|nRF_MAX_RT,
                NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
}


/*
  Transmit a packet with auto-ack and retry.
  Wait for it to be successfully transmitted, or for a retry timeout.
  Return NULL for ok or else an error message.
*/
const char *
nrf_transmit_packet(uint8_t *packet)
{
  uint32_t start_time = get_time();
  const char *errmsg;

  nrf_tx(packet, NRF_PACKET_SIZE, NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
  nrf_ce_high(NRF_CE_BASE, NRF_CE_PIN);
  delay_us(10);
  nrf_ce_low(NRF_CE_BASE, NRF_CE_PIN);

  for (;;)
  {
    uint32_t status = nrf_get_status(NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
    if (status & nRF_MAX_RT)
    {
      errmsg = "E: No ack from receiver\r\n";
      break;
    }
    if (status & nRF_TX_DS)
    {
      errmsg = NULL;
      break;
    }
    if (calc_time(start_time) > MCU_HZ/5)
    {
      errmsg = "E: Timeout from nRF24L01+ waiting for transmit\r\n";
      break;
    }
  }
  /* Clear the data sent / max retries flags. */
  nrf_write_reg(nRF_STATUS, nRF_TX_DS|nRF_MAX_RT,
                NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
  return errmsg;
}


void
setup_nrf(void)
{
  config_ssi_gpio();
  config_spi();
  config_timer();
  config_interrupts();
  config_udma_for_spi();
}


void
nrf_startup(void)
{
  uint8_t status;
  uint8_t val;

  serial_output_str("Tx: Setting up...\r\n");

  /* nRF24L01+ datasheet says to wait 100msec for bootup. */
  ROM_SysCtlDelay(MCU_HZ/3/10);
  nrf_init_config(0 /* Tx */, 81, nRF_RF_PWR_0DBM,
                  NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);

  serial_output_str("Tx: Read CONFIG=0x");
  val = nrf_read_reg(nRF_CONFIG, &status,
                     NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
  serial_output_hexbyte(val);
  serial_output_str(" status=0x");
  serial_output_hexbyte(status);
  serial_output_str("\r\n");
  serial_output_str("Done!\r\n");
}
