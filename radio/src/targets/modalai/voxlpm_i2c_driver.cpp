#pragma once

#include "voxlpm_i2c_driver.h"

#include "stm32_hal_ll.h"
#include "stm32_gpio.h"
#include "stm32_ws2812.h"

#include "board.h"
#include "boards/generic_stm32/module_ports.h"
#include "boards/generic_stm32/analog_inputs.h"
#include "boards/generic_stm32/rgb_leds.h"

// TODO: Make this more portable using the edgetx stm32_i2c libs

#if defined(POWER_I2C)

// static int count = 0;
static uint8_t current_reg = 0;

static volatile uint32_t status = 0;
static volatile uint32_t cr1_stat = 0;
static volatile uint32_t cr2_stat = 0;

void i2c_set_read_reg(uint8_t reg)
{
    uint8_t n_bytes = 1;
    volatile uint8_t tx_buf[1] = {0};

    tx_buf[0] = reg;

    POWER_I2C->CR1 |= I2C_CR1_PE; // Turn on the peripheral

    POWER_I2C->CR2 &= ~I2C_CR2_START; // Clear START

    while(POWER_I2C->CR2 & I2C_CR2_START) {};

    POWER_I2C->ISR |= I2C_ISR_TXE;
    POWER_I2C->ICR |= I2C_ICR_NACKCF;
    POWER_I2C->ICR |= I2C_ICR_STOPCF;

    POWER_I2C->CR2 &= ~I2C_CR2_RD_WRN; // Set Write mode

    POWER_I2C->CR2 &=  ~(I2C_CR2_NBYTES_Msk);;
    POWER_I2C->CR2 |= (n_bytes << I2C_CR2_NBYTES_Pos);; // Set number of bytes to transfer

    POWER_I2C->CR2 |= I2C_CR2_START; // START

    for(volatile int i = 0; i < n_bytes; i++) // transmit each byte when register available
    {
        while(!(POWER_I2C->ISR & I2C_ISR_TXE)) { status = POWER_I2C->ISR; cr1_stat = POWER_I2C->CR1; cr2_stat = POWER_I2C->CR2; }
        if(i < n_bytes) POWER_I2C->TXDR = tx_buf[i];
    }

    while(!(POWER_I2C->ISR & I2C_ISR_STOPF)) { status = POWER_I2C->ISR; cr1_stat = POWER_I2C->CR1; cr2_stat = POWER_I2C->CR2; } // Wait until transaction finished

    POWER_I2C->CR1 &= ~I2C_CR1_PE; // Turn off the peripheral

    current_reg = reg;
}



void i2c_transmit(uint8_t reg, uint16_t val)
{
    uint8_t n_bytes = 3;
    volatile uint8_t rx_buf[3] = {0};
    volatile uint8_t tx_buf[3] = {0};

    tx_buf[0] = reg;
    tx_buf[1] = (val >> 8) & 0xFF; // MSB first
    tx_buf[2] = (val) & 0xFF;

    POWER_I2C->CR1 |= I2C_CR1_PE; // Turn on the peripheral

    POWER_I2C->CR2 &= ~I2C_CR2_START; // Clear START

    while(POWER_I2C->CR2 & I2C_CR2_START) {};

    POWER_I2C->ISR |= I2C_ISR_TXE;
    POWER_I2C->ICR |= I2C_ICR_NACKCF;
    POWER_I2C->ICR |= I2C_ICR_STOPCF;

    POWER_I2C->CR2 &= ~I2C_CR2_RD_WRN; // Set Read mode

    POWER_I2C->CR2 &= ~(I2C_CR2_NBYTES_Msk);
    POWER_I2C->CR2 |= (n_bytes << I2C_CR2_NBYTES_Pos); // Set number of bytes to transfer

    POWER_I2C->CR2 |= I2C_CR2_START; // START

    for(volatile int i = 0; i < n_bytes; i++) // transmit each byte when register available
    {
        while(!(POWER_I2C->ISR & I2C_ISR_TXE)) { status = POWER_I2C->ISR; cr1_stat = POWER_I2C->CR1; cr2_stat = POWER_I2C->CR2; }
        if(i < n_bytes) POWER_I2C->TXDR = tx_buf[i];
    }

    while(!(POWER_I2C->ISR & I2C_ISR_STOPF)) { status = POWER_I2C->ISR; cr1_stat = POWER_I2C->CR1; cr2_stat = POWER_I2C->CR2; } // Wait until transaction finished

    POWER_I2C->CR1 &= ~I2C_CR1_PE; // Turn off the peripheral

    current_reg = reg;
    return;
}

uint16_t i2c_receive(uint8_t reg)
{
    // if(current_reg != reg);
    // {
    i2c_set_read_reg(reg);
    // }
    uint8_t n_bytes = 2;
    volatile uint8_t rx_buf[3] = {0};

    POWER_I2C->CR1 |= I2C_CR1_PE; // Turn on the peripheral

    POWER_I2C->CR2 &= ~I2C_CR2_START; // Clear START

    while(POWER_I2C->CR2 & I2C_CR2_START) {};

    POWER_I2C->ISR |= I2C_ISR_TXE;
    POWER_I2C->ICR |= I2C_ICR_NACKCF;
    POWER_I2C->ICR |= I2C_ICR_STOPCF;

    POWER_I2C->CR2 |= I2C_CR2_RD_WRN; // Set Read mode

    POWER_I2C->CR2 &= ~(I2C_CR2_NBYTES_Msk);
    POWER_I2C->CR2 |= (n_bytes << I2C_CR2_NBYTES_Pos); // Set number of bytes to transfer

    POWER_I2C->CR2 |= I2C_CR2_START; // START

    for(volatile int i = 0; i < n_bytes; i++)
    {
        while(!(POWER_I2C->ISR & I2C_ISR_RXNE)) { status = POWER_I2C->ISR; cr1_stat = POWER_I2C->CR1; cr2_stat = POWER_I2C->CR2; } // Wait for data
        rx_buf[i] = POWER_I2C->RXDR;
    }

    while(!(POWER_I2C->ISR & I2C_ISR_STOPF)) { status = POWER_I2C->ISR; cr1_stat = POWER_I2C->CR1; cr2_stat = POWER_I2C->CR2; } // Wait until transaction finished

    POWER_I2C->CR1 &= ~I2C_CR1_PE; // Turn off the peripheral

    uint16_t data_out = rx_buf[1]; // LSB
    data_out |= rx_buf[0] << 8;
    return data_out;
}

void voxl_pm_init()
{
    gpio_init(POWER_I2C_PIN_SDA, GPIO_OD, GPIO_PIN_SPEED_HIGH);
    gpio_init(POWER_I2C_PIN_SCL, GPIO_OD, GPIO_PIN_SPEED_HIGH);

    gpio_init_af(POWER_I2C_PIN_SDA, POWER_I2C_SDA_AF, GPIO_PIN_SPEED_HIGH);
    gpio_init_af(POWER_I2C_PIN_SCL, POWER_I2C_SCL_AF, GPIO_PIN_SPEED_HIGH);

    // RCC->APB1LENR |= 0x800000; // Turn on I2C3 Peripheral Clock
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

    POWER_I2C->TIMINGR = POWER_I2C_TIMING_REGISTER; // Set timing values
    // POWER_I2C->CR1 |= I2C_CR1_ANFOFF; // Turn off analog filter
    uint32_t dnf = 0x3U;
    POWER_I2C->CR1 |= dnf << I2C_CR1_DNF_Pos;


    POWER_I2C->CR2 |= I2C_CR2_RD_WRN; // Set Read mode

    POWER_I2C->CR2 |= I2C_CR2_AUTOEND; // Automatically send stop condition

    // Empty receive register
    while(POWER_I2C->ISR & I2C_ISR_RXNE)
    {
        uint8_t data = POWER_I2C->RXDR; // Read one data byte out
    }

    POWER_I2C->CR2 &= ~(I2C_CR2_SADD_Msk); // Clear slave address
    POWER_I2C->CR2 |= (uint32_t) POWER_I2C_SLAVE_ADDRESS; // Set slave address

    POWER_I2C->CR2 &= ~I2C_CR2_START; // Clear START

    while(POWER_I2C->CR2 & I2C_CR2_START) {}; // Wait for start to be 0

    i2c_transmit(INA231_REG_CONFIG, (INA231_CONFIG | 0x4000));

    // while(i2c_receive(INA231_REG_CONFIG) != (INA231_CONFIG | 0x4000)) { count++; };
    return;
}

uint16_t voxl_pm_get_voltage()
{
    uint16_t voltage = i2c_receive(INA231_REG_BUSVOLTAGE) * (INA231_VBUSSCALE * 100);
    return voltage;
}


#endif