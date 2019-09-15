#define USB_RECV_BUF_SIZE 8192

struct usb_recv_buf_struct {
  uint32_t head;
  uint32_t tail;
  unsigned char buf[USB_RECV_BUF_SIZE];
};

extern volatile struct usb_recv_buf_struct usb_recvbuf;
