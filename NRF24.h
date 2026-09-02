#ifndef NRF_24_H
#define NRF_24_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// Register Addresses
#define CONFIG          0x00
#define EN_AA           0x01
#define EN_RXADDR       0x02
#define SETUP_AW        0x03
#define SETUP_RETR      0x04
#define RF_CH           0x05
#define RF_SETUP        0x06
#define STATUS          0x07
#define RX_ADDR_P0      0x0A
#define RX_ADDR_P1      0x0B
#define TX_ADDR         0x10
#define RX_PW_P0        0x11
#define RX_PW_P1        0x12
#define FIFO_STATUS     0x17
#define DYNPD           0x1C
#define FEATURE         0x1D

// Commands
#define R_REGISTER      0x00
#define W_REGISTER      0x20
#define R_RX_PAYLOAD    0x61
#define W_TX_PAYLOAD    0xA0
#define W_ACK_PAYLOAD   0xA8
#define FLUSH_TX        0xE1
#define FLUSH_RX        0xE2
#define NOP_CMD         0xFF

// Bit positions
#define PWR_UP      1
#define PRIM_RX     0
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define ERX_P0      0
#define ERX_P1      1
#define RX_EMPTY    0
#define EN_DPL      2
#define EN_ACK_PAY  1

// Pin control
void ce(int level);
void csn(int level);

// Register operations
uint8_t nrf_spi_transfer(uint8_t tx);
void write_register(uint8_t reg, uint8_t value);
uint8_t read_register(uint8_t reg);
void write_register_buf(uint8_t reg, const uint8_t *data, uint8_t len);
void read_register_buf(uint8_t reg, uint8_t *data, uint8_t len);

// FIFO operations
void flush_tx(void);
void flush_rx(void);
uint8_t get_status(void);

// Utility
void delay_us(uint16_t del_time);

#endif
