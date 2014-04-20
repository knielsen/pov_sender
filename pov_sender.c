#include <inttypes.h>
#include <string.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ssi.h"
#include "inc/hw_uart.h"
#include "inc/hw_timer.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/ssi.h"
#include "driverlib/udma.h"
#include "driverlib/systick.h"

#include "nrf24l01p.h"


/*
  nRF24L01 pinout:

  Tx:
    PF2  SCK        GND *1 2. VCC
    PF3  CSN        PB3 .3 4. PF3
    PF0  MISO       PF2 .5 6. PF1
    PF1  MOSI       PF0 .7 8. PB0
    PB0  CE
    PB3  IRQ
*/

#define SIZEX 65
#define SIZEY 65
#define BUF_SIZE ((SIZEX*SIZEY*3+1)/2)
#define BUF_SIZE_PAD (32*((BUF_SIZE+30)/31))
#define BURSTSIZE 24


/* To change this, must fix clock setup in the code. */
#define MCU_HZ 80000000


static void
serial_output_hexdig(uint32_t dig)
{
  ROM_UARTCharPut(UART0_BASE, (dig >= 10 ? 'A' - 10 + dig : '0' + dig));
}


static void
serial_output_hexbyte(uint8_t byte)
{
  serial_output_hexdig(byte >> 4);
  serial_output_hexdig(byte & 0xf);
}


static void
serial_output_str(const char *str)
{
  char c;

  while ((c = *str++))
    ROM_UARTCharPut(UART0_BASE, c);
}


 __attribute__ ((unused))
static void
println_uint32(uint32_t val)
{
  char buf[12];
  char *p = buf;
  uint32_t l, d;

  l = 1000000000UL;
  while (l > val && l > 1)
    l /= 10;

  do
  {
    d = val / l;
    *p++ = '0' + d;
    val -= d*l;
    l /= 10;
  } while (l > 0);

  *p++ = '\n';
  *p = '\0';
  serial_output_str(buf);
}


static void
config_ssi_gpio(void)
{
  /* Config Tx on SSI1, PF0-PF3 + PB0/PB3. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

  /* PF0 is special (NMI), needs unlock to be re-assigned to SSI1. */
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
  HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

  ROM_GPIOPinConfigure(GPIO_PF2_SSI1CLK);
  ROM_GPIOPinConfigure(GPIO_PF0_SSI1RX);
  ROM_GPIOPinConfigure(GPIO_PF1_SSI1TX);
  ROM_GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
  /* CSN pin, high initially */
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
  ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
  /* CE pin, low initially */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);
  ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0);
  /* IRQ pin as input. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_3);
}


static void
config_spi(uint32_t base)
{
  /*
    Configure the SPI for correct mode to read from nRF24L01+.

    We need CLK inactive low, so SPO=0.
    We need to setup and sample on the leading, rising CLK edge, so SPH=0.

    The datasheet says up to 10MHz SPI is possible, depending on load
    capacitance. Let's go with a slightly cautious 8MHz, which should be
    aplenty.
  */

  ROM_SSIDisable(base);
  ROM_SSIConfigSetExpClk(base, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                         SSI_MODE_MASTER, 8000000, 8);
  ROM_SSIEnable(base);
}


static uint32_t udma_control_block[256] __attribute__ ((aligned(1024)));
static void
config_udma_for_spi(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
  ROM_uDMAEnable();
  ROM_uDMAControlBaseSet(udma_control_block);

  ROM_SSIDMAEnable(SSI1_BASE, SSI_DMA_TX);
  ROM_SSIDMAEnable(SSI1_BASE, SSI_DMA_RX);
  ROM_IntEnable(INT_SSI1);

  ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_SSI1TX, UDMA_ATTR_ALTSELECT |
                                  UDMA_ATTR_REQMASK | UDMA_ATTR_HIGH_PRIORITY);
  ROM_uDMAChannelAssign(UDMA_CH25_SSI1TX);
  ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_SSI1TX, UDMA_ATTR_USEBURST);
  ROM_uDMAChannelControlSet(UDMA_CHANNEL_SSI1TX | UDMA_PRI_SELECT,
                            UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
                            UDMA_ARB_4);

  ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_SSI1RX, UDMA_ATTR_ALTSELECT |
                                  UDMA_ATTR_REQMASK | UDMA_ATTR_HIGH_PRIORITY);
  ROM_uDMAChannelAssign(UDMA_CH24_SSI1RX);
  ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_SSI1RX, UDMA_ATTR_USEBURST);
  ROM_uDMAChannelControlSet(UDMA_CHANNEL_SSI1RX | UDMA_PRI_SELECT,
                            UDMA_SIZE_8 | UDMA_DST_INC_8 | UDMA_SRC_INC_NONE |
                            UDMA_ARB_4);
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


static inline void
csn_low(uint32_t csn_base, uint32_t csn_pin)
{
  ROM_GPIOPinWrite(csn_base, csn_pin, 0);
}


static inline void
csn_high(uint32_t csn_base, uint32_t csn_pin)
{
  ROM_GPIOPinWrite(csn_base, csn_pin, csn_pin);
}


static inline void
ce_low(uint32_t ce_base, uint32_t ce_pin)
{
  ROM_GPIOPinWrite(ce_base, ce_pin, 0);
}


static inline void
ce_high(uint32_t ce_base, uint32_t ce_pin)
{
  ROM_GPIOPinWrite(ce_base, ce_pin, ce_pin);
}


static void
ssi_cmd(uint8_t *recvbuf, const uint8_t *sendbuf, uint32_t len,
        uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint32_t i;
  uint32_t data;

  /* Take CSN low to initiate transfer. */
  csn_low(csn_base, csn_pin);

  for (i = 0; i < len; ++i)
  {
    ROM_SSIDataPut(ssi_base, sendbuf[i]);
    while (ROM_SSIBusy(ssi_base))
      ;
    ROM_SSIDataGet(ssi_base, &data);
    recvbuf[i] = data;
  }

  /* Take CSN high to complete transfer. */
  csn_high(csn_base, csn_pin);
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
  csn_low(csn_base, csn_pin);
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
    csn_high(a->csn_base, a->csn_pin);
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
  csn_low(csn_base, csn_pin);
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
    csn_high(a->csn_base, a->csn_pin);

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
      ce_high(a->ce_base, a->ce_pin);
      a->ce_asserted = 1;
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
    ce_low(a->ce_base, a->ce_pin);
    a->ce_asserted = 0;
    return 1;

  default:
    /* This shouldn't really happen ... */
    return 0;
  }
}


/*
  Configure nRF24L01+ as Rx or Tx.
    channel - radio frequency channel to use, 0 <= channel <= 127.
    power - nRF_RF_PWR_<X>DBM, <X> is 0, 6, 12, 18 dBm.
*/
static void
nrf_init_config(uint8_t is_rx, uint32_t channel, uint32_t power,
                uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  static const uint8_t addr[3] = { 0xe7, 0xe7, 0xe7 };

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
  nrf_write_reg(nRF_RF_SETUP, nRF_RF_DR_HIGH | power,
                ssi_base, csn_base, csn_pin);
  nrf_write_reg_n(nRF_RX_ADDR_P0, addr, 3, ssi_base, csn_base, csn_pin);
  nrf_write_reg_n(nRF_TX_ADDR, addr, 3, ssi_base, csn_base, csn_pin);
  /* Set payload size for pipe 0. */
  if (is_rx)
    nrf_write_reg(nRF_RX_PW_P0, 32, ssi_base, csn_base, csn_pin);
  else
    nrf_write_reg(nRF_RX_PW_P0, 8, ssi_base, csn_base, csn_pin);
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


static void
config_interrupts(void)
{
  ROM_GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_LOW_LEVEL);
  ROM_IntMasterEnable();
  ROM_IntEnable(INT_GPIOB);
}


static uint32_t write_sofar;
static struct nrf_async_transmit_multi transmit_multi_state;
static volatile uint8_t transmit_multi_running = 0;
static uint8_t transmit_running = 0;

static void
handle_end_of_transmit_burst(void)
{
  transmit_multi_running = 0;
  if (write_sofar < BUF_SIZE_PAD)
  {
    /*
      Wait 20 microseconds, then start another write burst.
      The nRF24L01+ datasheet says that we must not stay in transmit mode
      for more than 4 milliseconds (which is just a bit more than the time
      to send 24 full packets back-to-back).
    */
    ROM_TimerLoadSet(TIMER2_BASE, TIMER_A, 20*MCU_HZ/1000000);
    ROM_TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    ROM_TimerEnable(TIMER2_BASE, TIMER_A);
  }
  else
    transmit_running = 0;
}


void
IntHandlerGPIOb(void)
{
  uint32_t irq_status = HWREG(GPIO_PORTB_BASE + GPIO_O_MIS) & 0xff;
  if (irq_status & GPIO_PIN_3)
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
      HWREG(GPIO_PORTB_BASE + GPIO_O_IM) &= ~GPIO_PIN_3 & 0xff;
      HWREG(GPIO_PORTB_BASE + GPIO_O_ICR) = GPIO_PIN_3;

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


static uint8_t frame_buffers[2][BUF_SIZE_PAD];
static volatile uint32_t write_idx;

static int32_t
my_fill_packet(uint8_t *buf, void *d)
{
  uint32_t i;
  uint8_t *src = &frame_buffers[write_idx][write_sofar];

  for (i = 0; i < 32; ++i)
    *buf++ = *src++;
  write_sofar += 32;
  return (write_sofar < BUF_SIZE_PAD);
}


static void
setup_systick(void)
{
  ROM_SysTickPeriodSet(0xffffff+1);
  /* Force reload. */
  HWREG(NVIC_ST_CURRENT) = 0;
  ROM_SysTickEnable();
}


static inline uint32_t
get_time(void)
{
  return HWREG(NVIC_ST_CURRENT);
}


static inline uint32_t
calc_time(uint32_t start)
{
  uint32_t stop = HWREG(NVIC_ST_CURRENT);
  return (start - stop) & 0xffffff;
}


void
IntHandlerTimer2A(void)
{
  ROM_TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
  ROM_TimerDisable(TIMER2_BASE, TIMER_A);
  ROM_TimerIntDisable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

  /* Start the next write burst (unless we are done). */
  if (write_sofar < BUF_SIZE_PAD)
  {
    transmit_multi_running = 1;
    nrf_async_transmit_multi_start(&transmit_multi_state, my_fill_packet,
                                   NULL, BURSTSIZE, SSI1_BASE,
                                   UDMA_CHANNEL_SSI1RX, UDMA_CHANNEL_SSI1TX,
                                   GPIO_PORTF_BASE, GPIO_PIN_3,
                                   GPIO_PORTB_BASE, GPIO_PIN_0,
                                   GPIO_PORTB_BASE, GPIO_PIN_3);
  }
  else
    transmit_running = 0;
}


static void
transmit_start(void)
{
  transmit_running = 1;
  write_sofar = 0;
  transmit_multi_running = 1;
  nrf_async_transmit_multi_start(&transmit_multi_state, my_fill_packet,
                                 NULL, BURSTSIZE, SSI1_BASE,
                                 UDMA_CHANNEL_SSI1RX, UDMA_CHANNEL_SSI1TX,
                                 GPIO_PORTF_BASE, GPIO_PIN_3,
                                 GPIO_PORTB_BASE, GPIO_PIN_0,
                                 GPIO_PORTB_BASE, GPIO_PIN_3);
}


int main()
{
  uint8_t status;
  uint8_t val;
  uint32_t sofar;
  uint8_t next_run_num;
  uint32_t read_idx;

  /* Use the full 80MHz system clock. */
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                     SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  ROM_FPULazyStackingEnable();
  setup_systick();

  /* Configure serial. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
  ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  ROM_UARTConfigSetExpClk(UART0_BASE, (ROM_SysCtlClockGet()), 2000000,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

  config_ssi_gpio();
  config_spi(SSI1_BASE);

  /*
    Configure timer interrupt, used to put a small delay between transmit
    bursts.
  */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
  /* Configure 2 * 16-bit timer, A periodic. */
  ROM_TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR |
                     TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_ONE_SHOT);

  /* Enable interrrupts. */
  ROM_IntMasterEnable();
  ROM_IntEnable(INT_TIMER2A);

  /* nRF24L01+ datasheet says to wait 100msec for bootup. */
  ROM_SysCtlDelay(MCU_HZ/3/10);

  serial_output_str("Tx: Setting up...\r\n");
  config_interrupts();
  config_udma_for_spi();
  nrf_init_config(0 /* Tx */, 2, nRF_RF_PWR_0DBM,
                  SSI1_BASE, GPIO_PORTF_BASE, GPIO_PIN_3);
  serial_output_str("Tx: Read CONFIG=0x");
  val = nrf_read_reg(nRF_CONFIG, &status,
                     SSI1_BASE, GPIO_PORTF_BASE, GPIO_PIN_3);
  serial_output_hexbyte(val);
  serial_output_str(" status=0x");
  serial_output_hexbyte(status);
  serial_output_str("\r\n");
  serial_output_str("Done!\r\n");

  sofar = 0;
  next_run_num = 0;
  read_idx = 0;
  write_idx = 1;
  for (;;)
  {
    uint32_t start_time = get_time();
    uint8_t val;

ROM_UARTCharPut(UART0_BASE, '.');
    /* Wait for data, then read from the RX fifo. */
    while(HWREG(UART0_BASE + UART_O_FR) & UART_FR_RXFE)
    {
      /* Simple sync method: if data stream pauses, then reset state. */
      if (calc_time(start_time) > MCU_HZ/5)
      {
        sofar = 0;
        next_run_num = 0;
        start_time = get_time();
      }
    }
    if (HWREG(UART0_BASE + UART_O_RSR) & 0x8)
    {
      serial_output_str("\r\nRx overrun!\r\n");
      HWREG(UART0_BASE + UART_O_RSR) = 0;       /* Clear the overrun status */
    }

    val = HWREG(UART0_BASE + UART_O_DR);

    if ((sofar % 32) == 0)
    {
      if (val != next_run_num)
      {
        serial_output_str("Corruption, expected: ");
        println_uint32(next_run_num);
        serial_output_str("Got: ");
        println_uint32(val);
      }
      ++next_run_num;
    }
    frame_buffers[read_idx][sofar++] = val;
    if (sofar >= BUF_SIZE_PAD)
    {
      sofar = 0;
      next_run_num = 0;
      if (!transmit_running)
      {
        write_idx = read_idx;
        read_idx = (1 - read_idx);
        transmit_start();
      }
      else
      {
        /* If wireless transmit is too slow, we have to skip the frame. */
        serial_output_str("Skip frame due to previous frame "
                          "not transmitted.\r\n");
      }
    }
  }
}
