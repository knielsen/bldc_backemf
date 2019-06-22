#include "bldc_backemf.h"
#include "usb.h"

#include "driverlib/usb.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"

#include "usb_serial_structs.h"

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


void
usb_data_put(const unsigned char *buf, uint32_t size)
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

volatile struct usb_recv_buf_struct usb_recvbuf;

static void
usb_data_get(void)
{
  unsigned long h, t, size, actual;

  h = usb_recvbuf.head;
  t = usb_recvbuf.tail;
  if (h >= t)
  {
    size = USB_RECV_BUF_SIZE - h;
    /* Do not overrun the FIFO. */
    if (t == 0)
      --size;
  }
  else
    size = t - h - 1;
  if (size == 0)
    return;

  actual = USBBufferRead((tUSBBuffer *)&g_sRxBuffer,
                         (unsigned char *)&usb_recvbuf.buf[h], size);
  h += actual;
  if (h >= USB_RECV_BUF_SIZE)
    h = 0;
  if (actual >= size && h + 1 < t)
  {
    size = t - h - 1;
    actual = USBBufferRead((tUSBBuffer *)&g_sRxBuffer,
                           (unsigned char *)&usb_recvbuf.buf[h], size);
    h += actual;
  }
  usb_recvbuf.head = h;
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

          h = usb_recvbuf.head;
          t = usb_recvbuf.tail;
          if (t > h)
            return (USB_RECV_BUF_SIZE - t) + h;
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


void
config_usb(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  ROM_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOD);
  ROM_GPIOPinTypeUSBAnalog(GPIO_PORTD_AHB_BASE, GPIO_PIN_5 | GPIO_PIN_4);

  USBBufferInit((tUSBBuffer *)&g_sTxBuffer);
  USBBufferInit((tUSBBuffer *)&g_sRxBuffer);
  USBStackModeSet(0, USB_MODE_FORCE_DEVICE, 0);
  USBDCDCInit(0, (tUSBDCDCDevice *)&g_sCDCDevice);
}
