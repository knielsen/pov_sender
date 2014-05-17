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
#include "driverlib/usb.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"

#include "nrf24l01p.h"
#include "usb_serial_structs.h"


/*
  nRF24L01 pinout:

  Tx:
    PF2  SCK        GND *1 2. VCC
    PF3  CSN        PB3 .3 4. PF3
    PF0  MISO       PF2 .5 6. PF1
    PF1  MOSI       PF0 .7 8. PB0
    PB0  IRQ
    PB3  CE
*/

#define SIZEX 65
#define SIZEY 65
#define BUF_SIZE ((SIZEX*SIZEY*3+1)/2)
#define BUF_SIZE_PAD (32*((BUF_SIZE+30)/31))
#define BURSTSIZE 24


/*
  Note that to change these, may require additional changes in
  config_ssi_gpio() and in IRQ handler setup.
*/
#define NRF_SSI_BASE SSI1_BASE
#define NRF_CSN_BASE GPIO_PORTF_BASE
#define NRF_CSN_PIN GPIO_PIN_3
#define NRF_CE_BASE GPIO_PORTB_BASE
#define NRF_CE_PIN GPIO_PIN_3
#define NRF_IRQ_BASE GPIO_PORTB_BASE
#define NRF_IRQ_PIN GPIO_PIN_0
#define NRF_DMA_CH_RX UDMA_CHANNEL_SSI1RX
#define NRF_DMA_CH_TX UDMA_CHANNEL_SSI1TX


/* Communications protocol. */
#define POV_CMD_DEBUG 254
#define POV_SUBCMD_RESET_TO_BOOTLOADER 255
#define POV_SUBCMD_ENTER_BOOTLOADER 254
#define POV_SUBCMD_RESET_TO_APP 253
#define POV_SUBCMD_FLASH_BUFFER 252
#define POV_SUBCMD_STATUS_REPLY 240


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


/*
  USB stuff.
  Mostly copied and simplified from the Stellaris/Tiva USB virtual COM port
  example.
*/

/*
  Define my own memcpy(). The usblib gets to access gcc builtin memcpy() for
  struct assignment, and somehow it seems to get linked with a version that
  uses not thumb2 instructions but basic full-length ARM instructions, which
  does not work ...
*/
void *
memcpy(void *dest, const void *src, unsigned n)
{
  unsigned char *d = dest;
  const unsigned char *s = src;
  while(n--)
    *d++ = *s++;
  return dest;
}


__attribute__ ((unused))
static void
usb_data_put(unsigned char *buf, uint32_t size)
{
  while (size > 0)
  {
    uint32_t actual = USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, buf, size);
    if (size >= actual)
      size -= actual;
    else
      size = 0;
    buf += actual;
  }
}


#define RECV_BUF_SIZE 8192
static volatile
struct recv_buf_struct {
  uint32_t head;
  uint32_t tail;
  unsigned char buf[RECV_BUF_SIZE];
} recvbuf;

static void
usb_data_get(void)
{
  unsigned long h, t, size, actual;

  h = recvbuf.head;
  t = recvbuf.tail;
  if (h >= t)
  {
    size = RECV_BUF_SIZE - h;
    /* Do not overrun the FIFO. */
    if (t == 0)
      --size;
  }
  else
    size = t - h - 1;
  if (size == 0)
    return;

  actual = USBBufferRead((tUSBBuffer *)&g_sRxBuffer,
                         (unsigned char *)&recvbuf.buf[h], size);
  h += actual;
  if (h >= RECV_BUF_SIZE)
    h = 0;
  if (actual >= size && h + 1 < t)
  {
    size = t - h - 1;
    actual = USBBufferRead((tUSBBuffer *)&g_sRxBuffer,
                           (unsigned char *)&recvbuf.buf[h], size);
    h += actual;
  }
  recvbuf.head = h;
}


//*****************************************************************************
//
// Get the communication parameters in use on the UART.
//
//*****************************************************************************
static void
GetLineCoding(tLineCoding *psLineCoding)
{
    psLineCoding->ulRate = 2000000;
    psLineCoding->ucDatabits = 8;
    psLineCoding->ucParity = USB_CDC_PARITY_NONE;
    psLineCoding->ucStop = USB_CDC_STOP_BITS_1;
}

//*****************************************************************************
//
// Handles CDC driver notifications related to control and setup of the device.
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host.  These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
ControlHandler(void *pvCBData, unsigned long ulEvent,
               unsigned long ulMsgValue, void *pvMsgData)
{
    //
    // Which event are we being asked to process?
    //
    switch(ulEvent)
    {
        //
        // We are connected to a host and communication is now possible.
        //
        case USB_EVENT_CONNECTED:
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);
            break;

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
            break;

        //
        // Return the current serial communication parameters.
        //
        case USBD_CDC_EVENT_GET_LINE_CODING:
            GetLineCoding(pvMsgData);
            break;

        /* Ignored stuff. */
        case USBD_CDC_EVENT_SET_LINE_CODING:
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
        case USBD_CDC_EVENT_SEND_BREAK:
        case USBD_CDC_EVENT_CLEAR_BREAK:
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
            break;
    }

    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the transmit channel (data to
// the USB host).
//
// \param ulCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
TxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
          void *pvMsgData)
{
    //
    // Which event have we been sent?
    //
    switch(ulEvent)
    {
        case USB_EVENT_TX_COMPLETE:
            //
            // Since we are using the USBBuffer, we don't need to do anything
            // here.
            //
            break;

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
            break;
    }
    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the receive channel (data from
// the USB host).
//
// \param ulCBData is the client-supplied callback data value for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
RxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
          void *pvMsgData)
{
    //
    // Which event are we being sent?
    //
    switch(ulEvent)
    {
        //
        // A new packet has been received.
        //
        case USB_EVENT_RX_AVAILABLE:
        {
            usb_data_get();
            break;
        }

        case USB_EVENT_DATA_REMAINING:
        {
          uint32_t h, t;

          h = recvbuf.head;
          t = recvbuf.tail;
          if (t > h)
            return (RECV_BUF_SIZE - t) + h;
          else
            return h - t;
        }

        //
        // We are being asked to provide a buffer into which the next packet
        // can be read. We do not support this mode of receiving data so let
        // the driver know by returning 0. The CDC driver should not be sending
        // this message but this is included just for illustration and
        // completeness.
        //
        case USB_EVENT_REQUEST_BUFFER:
        {
            return(0);
        }

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
            break;
    }

    return(0);
}


static void
config_usb(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  ROM_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_5 | GPIO_PIN_4);

  USBBufferInit((tUSBBuffer *)&g_sTxBuffer);
  USBBufferInit((tUSBBuffer *)&g_sRxBuffer);
  USBStackModeSet(0, USB_MODE_DEVICE, 0);
  USBDCDCInit(0, (tUSBDCDCDevice *)&g_sCDCDevice);
}

/* End of USB stuff. */


static void
config_led(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
}


__attribute__ ((unused))
static void
led_on(void)
{
  GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
}


__attribute__ ((unused))
static void
led_off(void)
{
  GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
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
  ROM_GPIOPinTypeGPIOOutput(NRF_CSN_BASE, NRF_CSN_PIN);
  ROM_GPIOPinWrite(NRF_CSN_BASE, NRF_CSN_PIN, NRF_CSN_PIN);
  /* CE pin, low initially */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_GPIOPinTypeGPIOOutput(NRF_CE_BASE, NRF_CE_PIN);
  ROM_GPIOPinWrite(NRF_CE_BASE, NRF_CE_PIN, 0);
  /* IRQ pin as input. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_GPIOPinTypeGPIOInput(NRF_IRQ_BASE, NRF_IRQ_PIN);
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


static void
nrf_rx(uint8_t *data, uint32_t len,
       uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[33], recvbuf[33];

  if (len > 32)
    len = 32;
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

  if (len > 32)
    len = 32;
  sendbuf[0] = nRF_W_TX_PAYLOAD;
  memcpy(&sendbuf[1], data, len);
  ssi_cmd(recvbuf, sendbuf, len+1, ssi_base, csn_base, csn_pin);
}


static void
nrf_tx_nack(uint8_t *data, uint32_t len,
            uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[33], recvbuf[33];

  if (len > 32)
    len = 32;
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


static const uint8_t nrf_addr[3] = { 0xe7, 0xe7, 0xe7 };

/*
  Configure nRF24L01+ as Rx or Tx.
    channel - radio frequency channel to use, 0 <= channel <= 127.
    power - nRF_RF_PWR_<X>DBM, <X> is 0, 6, 12, 18 dBm.
*/
static void
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
  nrf_write_reg(nRF_RF_SETUP, nRF_RF_DR_HIGH | power,
                ssi_base, csn_base, csn_pin);
  nrf_write_reg_n(nRF_RX_ADDR_P0, nrf_addr, 3, ssi_base, csn_base, csn_pin);
  nrf_write_reg_n(nRF_TX_ADDR, nrf_addr, 3, ssi_base, csn_base, csn_pin);
  /* Set payload size for pipe 0. */
  nrf_write_reg(nRF_RX_PW_P0, 32, ssi_base, csn_base, csn_pin);
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
static void
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
static void
nrf_config_bootload_tx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  nrf_write_reg(nRF_CONFIG, nRF_MASK_RX_DR |
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


/*
  Configure nRF24L01+ as Rx for getting reply packet from the bootloader.

  The nRF24L01+ is configured as receiver, with auto-ack and
  retransmission enabled.
*/
static void
nrf_config_bootload_rx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  nrf_write_reg(nRF_CONFIG, nRF_PRIM_RX | nRF_MASK_RX_DR |
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
}


static uint32_t write_sofar;
static struct nrf_async_transmit_multi transmit_multi_state;
static volatile uint8_t transmit_multi_running = 0;
static volatile uint8_t transmit_running = 0;

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
                                   NULL, BURSTSIZE, NRF_SSI_BASE,
                                   NRF_DMA_CH_RX, NRF_DMA_CH_TX,
                                   NRF_CSN_BASE, NRF_CSN_PIN,
                                   NRF_CE_BASE, NRF_CE_PIN,
                                   NRF_IRQ_BASE, NRF_IRQ_PIN);
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
                                 NULL, BURSTSIZE, NRF_SSI_BASE,
                                 NRF_DMA_CH_RX, NRF_DMA_CH_TX,
                                 NRF_CSN_BASE, NRF_CSN_PIN,
                                 NRF_CE_BASE, NRF_CE_PIN,
                                 NRF_IRQ_BASE, NRF_IRQ_PIN);
}


/*
  Read a packet from USB. A packet is 32 bytes of data.

  Handles sync-up by resetting the state (and resetting the packet buffer to
  empty) whenever more than 0.2 seconds pass without any data received.

  Returns a true value if a reset was done, false if not.
*/
static uint32_t
usb_get_packet(uint8_t *packet_buf)
{
  uint32_t reset = 0;
  uint32_t sofar = 0;
  uint32_t start_time;
  uint8_t val;
  uint32_t h, t;

  while (sofar < 32)
  {
    t = recvbuf.tail;
    start_time = get_time();
    while ((h = recvbuf.head) == t)
    {
      /* Simple sync method: if data stream pauses, then reset state. */
      if (calc_time(start_time) > MCU_HZ/5)
      {
        sofar = 0;
        reset = 1;
        start_time = get_time();
      }
    }
    val = recvbuf.buf[t];
    ++t;
    if (t >= RECV_BUF_SIZE)
      t = 0;
    recvbuf.tail = t;
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


static void
delay_us(uint32_t us)
{
  /* This assumes that MCU_HZ is divisible by 1000000. */
  uint32_t cycles = (MCU_HZ/1000000)*us;
#if (MCU_HZ % 1000000)
#error delay_us() computes delay incorrectly if MCU_HZ is not a multiple of 1000000
#endif

  while (cycles > 0xffffff)
  {
    delay_systicks(0xffffff);
    cycles -= 0xffffff;
  }
  delay_systicks(cycles);
}


static void
nrf_transmit_packet_nack(uint8_t *packet)
{
  nrf_tx_nack(packet, 32, NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
}


static void
handle_cmd_debug(uint8_t *packet)
{
  /*
    Hm. At some point I need to implement that I can send the DEBUG START
    command to lm4f_pov.

    And I need to wait for existing transfer to stop before doing more with
    nRF24L01+. Not sure if I should disable interrupts, or if I should use
    existing interrupt/dma routines.
  */
  uint8_t subcmd= packet[1];

  /* First we need to wait for any on-going transmit to complete. */
  while (transmit_running)
    ;

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
    usb_get_packet(packet);
  }

  nrf_config_bootload_tx(NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);


  nrf_config_normal_tx(NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
}


int main()
{
  uint8_t status;
  uint8_t val;
  uint32_t sofar;
  uint8_t next_run_num;
  uint32_t read_idx;
  uint32_t corruption_flag;

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
  ROM_UARTConfigSetExpClk(UART0_BASE, (ROM_SysCtlClockGet()), 500000,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

  config_ssi_gpio();
  config_spi(NRF_SSI_BASE);
  config_led();

  /*
    Configure timer interrupt, used to put a small delay between transmit
    bursts.
  */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
  /* Configure 2 * 16-bit timer, A periodic. */
  ROM_TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR |
                     TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_ONE_SHOT);

  ROM_IntPriorityGroupingSet(7);
  ROM_IntPrioritySet(INT_SSI1, 0 << 5);
  ROM_IntPrioritySet(INT_GPIOB, 1 << 5);
  ROM_IntPrioritySet(INT_TIMER2A, 2 << 5);
  ROM_IntPrioritySet(INT_USB0, 4 << 5);

  config_usb();

  /* Enable interrrupts. */
  ROM_IntMasterEnable();
  ROM_IntEnable(INT_TIMER2A);

  /* nRF24L01+ datasheet says to wait 100msec for bootup. */
  ROM_SysCtlDelay(MCU_HZ/3/10);

  serial_output_str("Tx: Setting up...\r\n");
  config_interrupts();
  config_udma_for_spi();
  nrf_init_config(0 /* Tx */, 2, nRF_RF_PWR_0DBM,
                  NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
  serial_output_str("Tx: Read CONFIG=0x");
  val = nrf_read_reg(nRF_CONFIG, &status,
                     NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
  serial_output_hexbyte(val);
  serial_output_str(" status=0x");
  serial_output_hexbyte(status);
  serial_output_str("\r\n");
  serial_output_str("Done!\r\n");

  sofar = 0;
  next_run_num = 0;
  corruption_flag = 1;
  read_idx = 0;
  write_idx = 1;
  for (;;)
  {
    uint32_t i;
    uint8_t val;
    uint8_t usb_packet[32];

    /* Wait for a packet on the USB. */
    if (usb_get_packet(usb_packet))
    {
      /*
        If we had a resync on the USB, then reset the frame packet count.
        We do not want to transmit a half-received frame to the POV.
      */
      next_run_num = 0;
    }
    val = usb_packet[0];

    if (val == POV_CMD_DEBUG)
    {
      handle_cmd_debug(usb_packet);
      continue;
    }

    if (val != next_run_num)
    {
      if (!corruption_flag)
      {
        serial_output_str("Corruption, expected: ");
        println_uint32(next_run_num);
        serial_output_str("Got: ");
        println_uint32(val);
        /* Let's silently skip everything until start of next frame. */
        corruption_flag = 1;
        next_run_num = 0;
      }
      continue;
    }
    else
      corruption_flag = 0;
    ++next_run_num;

    /* Copy the packet into the frame buffer for later bulk sending. */
    for (i = 0; i < 32; ++i)
      frame_buffers[read_idx][sofar++] = usb_packet[i];
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
