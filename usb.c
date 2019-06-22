#include "bldc_backemf.h"

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


static uint32_t
my_strlen(const char *s)
{
  uint32_t len = 0;
  while (*s++)
    ++len;
  return len;
}


__attribute__ ((unused))
static void
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
#define USB_DBG(x) usb_data_put((const unsigned char *)("!" x), sizeof(x))
