/*
 * STM32 HAL nRF24L01+ support
 * Stable full-duplex SPI version.
 */

#include "stm32f1xx_hal.h"
#include "NRF24_conf.h"
#include "NRF24_reg_addresses.h"
#include "NRF24.h"
#include <stdbool.h>

extern SPI_HandleTypeDef hspiX;

void ce(int level)
{
    HAL_GPIO_WritePin(ce_gpio_port, ce_gpio_pin,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void csn(int level)
{
    HAL_GPIO_WritePin(csn_gpio_port, csn_gpio_pin,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t nrf_spi_transfer(uint8_t tx)
{
    uint8_t rx = 0U;

    if (HAL_SPI_TransmitReceive(&hspiX, &tx, &rx, 1U, 10U) != HAL_OK) {
        return 0xFFU;
    }

    return rx;
}

void write_register(uint8_t reg, uint8_t value)
{
    csn(0);
    nrf_spi_transfer((uint8_t)(W_REGISTER | (reg & 0x1FU)));
    nrf_spi_transfer(value);
    csn(1);
}

uint8_t read_register(uint8_t reg)
{
    uint8_t value;

    csn(0);
    nrf_spi_transfer((uint8_t)(R_REGISTER | (reg & 0x1FU)));
    value = nrf_spi_transfer(NOP_CMD);
    csn(1);

    return value;
}

void write_register_buf(uint8_t reg, const uint8_t *data, uint8_t len)
{
    csn(0);
    nrf_spi_transfer((uint8_t)(W_REGISTER | (reg & 0x1FU)));

    for (uint8_t i = 0U; i < len; i++) {
        nrf_spi_transfer(data[i]);
    }

    csn(1);
}

void read_register_buf(uint8_t reg, uint8_t *data, uint8_t len)
{
    csn(0);
    nrf_spi_transfer((uint8_t)(R_REGISTER | (reg & 0x1FU)));

    for (uint8_t i = 0U; i < len; i++) {
        data[i] = nrf_spi_transfer(NOP_CMD);
    }

    csn(1);
}

void flush_tx(void)
{
    csn(0);
    nrf_spi_transfer(FLUSH_TX);
    csn(1);
}

void flush_rx(void)
{
    csn(0);
    nrf_spi_transfer(FLUSH_RX);
    csn(1);
}

uint8_t get_status(void)
{
    uint8_t status;

    csn(0);
    status = nrf_spi_transfer(NOP_CMD);
    csn(1);

    return status;
}

void delay_us(uint16_t del_time)
{
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    uint32_t cycles_per_us = SystemCoreClock / 1000000UL;
    uint32_t start = DWT->CYCCNT;
    uint32_t wait_cycles = (uint32_t)del_time * cycles_per_us;

    while ((uint32_t)(DWT->CYCCNT - start) < wait_cycles) {
        __NOP();
    }
}
