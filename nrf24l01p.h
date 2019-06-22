/* nRF24L01+ commands. */
#define nRF_R_REGISTER 0
#define nRF_W_REGISTER 0x20
#define nRF_R_RX_PAYLOAD 0x61
#define nRF_W_TX_PAYLOAD 0xa0
#define nRF_FLUSH_TX 0xe1
#define nRF_FLUSH_RX 0xe2
#define nRF_REUSE_TX_PL 0xe3
#define nRF_R_RX_PL_WID 0x60
#define nRF_W_ACK_PAYLOAD 0xa8
#define nRF_W_TX_PAYLOAD_NOACK 0xb0
#define nRF_NOP 0xff


/* nRF24L01+ registers. */
#define nRF_CONFIG 0
#define nRF_MASK_RX_DR (1<<6)
#define nRF_MASK_TX_DS (1<<5)
#define nRF_MASK_MAX_RT (1<<4)
#define nRF_EN_CRC (1<<3)
#define nRF_CRCO (1<<2)
#define nRF_PWR_UP (1<<1)
#define nRF_PRIM_RX (1<<0)

#define nRF_EN_AA 1
#define nRF_ENAA_P0 (1<<0)
#define nRF_ENAA_P1 (1<<1)
#define nRF_ENAA_P2 (1<<2)
#define nRF_ENAA_P3 (1<<3)
#define nRF_ENAA_P4 (1<<4)
#define nRF_ENAA_P5 (1<<5)

#define nRF_EN_RXADDR 2
#define nRF_ERX_P0 (1<<0)
#define nRF_ERX_P1 (1<<1)
#define nRF_ERX_P2 (1<<2)
#define nRF_ERX_P3 (1<<3)
#define nRF_ERX_P4 (1<<4)
#define nRF_ERX_P5 (1<<5)

#define nRF_SETUP_AW 3
#define nRF_AW_3BYTES 1
#define nRF_AW_4BYTES 2
#define nRF_AW_5BYTES 3

#define nRF_SETUP_RETR 4
#define nRF_ARD_SHIFT 4
#define nRF_ARC_MASK 0xf

#define nRF_RF_CH 5
#define nRF_RF_SETUP 6
#define nRF_CONT_WAVE (1<<7)
#define nRF_RF_DR_LOW (1<<5)
#define nRF_PLL_LOCK (1<<4)
#define nRF_RF_DR_HIGH (1<<3)
#define nRF_RF_PWR_0DBM (3<<1)
#define nRF_RF_PWR_6DBM (2<<1)
#define nRF_RF_PWR_12DBM (1<<1)
#define nRF_RF_PWR_18DBM (0<<1)

#define nRF_STATUS 7
#define nRF_RX_DR (1<<6)
#define nRF_TX_DS (1<<5)
#define nRF_MAX_RT (1<<4)
#define nRF_RX_P_NO_SHIFT 1
#define nRF_RX_P_NO_MASK 0x7
#define nRF_RX_P_NO_EMPTY 0x7
#define nRF_TX_FULL (1<<0)

#define nRF_OBSERVE_TX 8
#define nRF_PLOS_CNT_SHIFT 4
#define nRF_ARC_CNT_MASK 0xf

#define nRF_RPD 9
#define nRF_RX_ADDR_P0 0xa
#define nRF_RX_ADDR_P1 0xb
#define nRF_RX_ADDR_P2 0xc
#define nRF_RX_ADDR_P3 0xd
#define nRF_RX_ADDR_P4 0xe
#define nRF_RX_ADDR_P5 0xf
#define nRF_TX_ADDR 0x10
#define nRF_RX_PW_P0 0x11
#define nRF_RX_PW_P1 0x12
#define nRF_RX_PW_P2 0x13
#define nRF_RX_PW_P3 0x14
#define nRF_RX_PW_P4 0x15
#define nRF_RX_PW_P5 0x16
#define nRF_FIFO_STATUS 0x17
#define nRF_TX_REUSE (1<<6)
#define nRF_FIFO_STATUS_TX_FULL (1<<5)
#define nRF_TX_EMPTY (1<<4)
#define nRF_RX_FULL (1<<1)
#define nRF_RX_EMPTY (1<<0)

#define nRF_DYNDP 0x1c
#define nRF_DPL_P5 (1<<5)
#define nRF_DPL_P4 (1<<4)
#define nRF_DPL_P3 (1<<3)
#define nRF_DPL_P2 (1<<2)
#define nRF_DPL_P1 (1<<1)
#define nRF_DPL_P0 (1<<0)

#define nRF_FEATURE 0x1d
#define nRF_EN_DPL (1<<2)
#define nRF_EN_ACK_PAY (1<<1)
#define nRF_EN_DYN_ACK (1<<0)
