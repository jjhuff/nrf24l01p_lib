/*
 * radio.c
 *
 *  Created on: 24-Jan-2009
 *      Author: Neil MacMillan
 */
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "nRF24L01.h"
#include "spi.h"

#include "radio.h"

// non-public constants and macros

#define ADDRESS_LENGTH 5

// Pin definitions
#define CE_DDR          DDRC
#define CE_PORT         PORTC
#define CE_PIN          PORTC0

#define IRQ_DDR         DDRC
#define IRQ_PIN         PORTC1
#define IRQ_VECTOR      PCINT1_vect

// Definitions for selecting and enabling the radio
#define CSN_HIGH()    PORTB |=  1<<SPI_SS_PIN;
#define CSN_LOW()     PORTB &= ~(1<<SPI_SS_PIN);
#define CE_HIGH()     CE_PORT |=  1<<CE_PIN;
#define CE_LOW()      CE_PORT &= ~(1<<CE_PIN);

// Flag which denotes that the radio is currently transmitting
static volatile uint8_t transmit_lock;

// holds the transmit address (Rx pipe 0 is set to this address when transmitting with auto-ack enabled).
static volatile uint8_t tx_address[5] = { 0xe7, 0xe7, 0xe7, 0xe7, 0xe7 };

// holds the receiver address for Rx pipe 0 (the address is overwritten when transmitting with auto-ack enabled).
static volatile uint8_t rx_pipe0_address[5] = { 0xe7, 0xe7, 0xe7, 0xe7, 0xe7 };

static volatile RADIO_TX_STATUS tx_last_status = RADIO_TX_SUCCESS;

extern void radio_rxhandler(uint8_t pipenumber);

/**
 * Retrieve the status register.
 */
static uint8_t get_status(void) {
    CSN_LOW();
    uint8_t status = send_spi(NOP);
    CSN_HIGH();
    return status;
}


/**
 * Set a register in the radio
 * \param reg The register value defined in nRF24L01.h (e.g. CONFIG, EN_AA, &c.).
 * \param value The value to write to the given register (the whole register is overwritten).
 * \return The status register.
 */
static uint8_t set_register(radio_register_t reg, const uint8_t* value, uint8_t len) {
    CSN_LOW();
    uint8_t status = send_spi(W_REGISTER | (REGISTER_MASK & reg));
    for (uint8_t i = 0; i < len; i++) {
        send_spi(value[i]);
    }
    CSN_HIGH();
    return status;
}

/**
 * Retrieve a register value from the radio.
 * \param reg The register value defined in nRF24L01.h (e.g. CONFIG, EN_AA, &c.).
 * \param buffer A contiguous memory block into which the register contents will be copied.  If the buffer is too long for the
 *         register contents, then the remaining bytes will be overwritten with 0xFF.
 * \param len The length of the buffer.
 */
static uint8_t get_register(radio_register_t reg, uint8_t* buffer, uint8_t len) {
    CSN_LOW();
    uint8_t status = send_spi(R_REGISTER | (REGISTER_MASK & reg));
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = send_spi(0);
    }
    CSN_HIGH();
    return status;
}

/**
 * Send an instruction to the nRF24L01.
 * \param instruction The instruction to send (see the bottom of nRF24L01.h)
 */
static void send_instruction_simple(uint8_t instruction) {
    CSN_LOW();
    send_spi(instruction);
    CSN_HIGH();
}

/**
 * Switch the radio to receive mode.  If the radio is already in receive mode, this does nothing.
 */
static void set_rx_mode(void) {
    uint8_t config;
    get_register(CONFIG, &config, 1);
    if ((config & (1<<PRIM_RX)) == 0) {
        config |= (1<<PRIM_RX);
        set_register(CONFIG, &config, 1);
        // the radio takes 130 us to power up the receiver.
        _delay_us(130);
    }
}

/**
 * Switch the radio to transmit mode.  If the radio is already in transmit mode, this does nothing.
 */
static void set_tx_mode(void) {
    uint8_t config;
    get_register(CONFIG, &config, 1);
    if ((config & (1<<PRIM_RX)) != 0) {
        config &= ~(1<<PRIM_RX);
        set_register(CONFIG, &config, 1);
        // The radio takes 130 us to power up the transmitter
        _delay_us(130);
    }
}

/**
 * Reset the pipe 0 address if pipe 0 is enabled.  This is necessary when the radio is using Enhanced Shockburst, because
 * the pipe 0 address is set to the transmit address while the radio is transmitting (this is how the radio receives
 * auto-ack packets).
 */
static void reset_pipe0_address(void) {
    set_register(RX_ADDR_P0, (uint8_t*)rx_pipe0_address, ADDRESS_LENGTH);
}

/**
 * Configure radio defaults and turn on the radio in receive mode.
 */
void Radio_Init() {
    transmit_lock = 0;

    //Setup SPI
    setup_spi(SPI_MODE_0, SPI_MSB, SPI_NO_INTERRUPT, SPI_MSTR_CLK16);

    // disable radio during config
    CE_LOW();

    // set as output
    CE_DDR |= 1<<CE_PIN;

    // A 10.3 ms delay is required between power off and power on states (controlled by 3.3 V supply).
    _delay_ms(11);

    uint8_t value;

    // set address width to 5 bytes.
    value = ADDRESS_LENGTH - 2;            // 0b11 for 5 bytes, 0b10 for 4 bytes, 0b01 for 3 bytes
    set_register(SETUP_AW, &value, 1);

    // Enable dynamic packet length support
    value = 1<<EN_DPL;
    set_register(FEATURE, &value, 1);
    value = 0x3F; // Enable for all pipes
    set_register(DYNPD, &value, 1);

    // set Enhanced Shockburst retry to every 586 us, up to 5 times.
    value = 0xFF;
    set_register(SETUP_RETR, &value, 1);

    // Enable 2-byte CRC and power up in receive mode.
    value = (1<<EN_CRC) | (1<<CRCO) | (1<<PWR_UP) | (1<<PRIM_RX);
    set_register(CONFIG, &value, 1);

    // flush the FIFOs in case there are old data in them.
    Radio_Flush();

    // clear the interrupt flags in case the radio's still asserting an old unhandled interrupt
    value = (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT);
    set_register(STATUS, &value, 1);

    // Enable radio interrupt.  This interrupt is triggered when data are received and when a transmission completes.
    IRQ_DDR &= ~(1<<IRQ_PIN);
    PCMSK1 |= (1<<IRQ_PIN);
    PCICR  |= (1<<PCIE1);

    // A 1.5 ms delay is required between power down and power up states (controlled by PWR_UP bit in CONFIG)
    _delay_ms(2);

    // enable radio as a receiver
    CE_HIGH();
}

void Radio_Configure(uint8_t channel, RADIO_DATA_RATE dr, RADIO_TX_POWER power) {
    uint8_t value;

    set_register(RF_CH, &channel, 1);

    // set the data rate and power bits in the RF_SETUP register
    get_register(RF_SETUP, &value, 1);

    value |= 3 << RF_PWR;            // set the power bits so that the & will mask the power value in properly.
    value &= power << RF_PWR;        // mask the power value into the RF status byte.

    switch(dr) {
        case RADIO_250KBPS:
            value |= 1<<RF_DR_LOW;
            value &= ~(1<<RF_DR_HIGH);
            break;
        case RADIO_1MBPS:
            value &= ~(1<<RF_DR_LOW);
            value &= ~(1<<RF_DR_HIGH);
            break;
        case RADIO_2MBPS:
            value &= ~(1<<RF_DR_LOW);
            value |= 1<<RF_DR_HIGH;
            break;
    }

    set_register(RF_SETUP, &value, 1);
}

// default address for pipe 0 is 0xe7e7e7e7e7
// default address for pipe 1 is 0xc2c2c2c2c2
// default address for pipe 2 is 0xc2c2c2c2c3 (disabled)
// default address for pipe 3 is 0xc2c2c2c2c4 (disabled)
// default address for pipe 4 is 0xc2c2c2c2c5 (disabled)
// default address for pipe 5 is 0xc2c2c2c2c6 (disabled)
void Radio_Configure_Rx(uint8_t pipe, const uint8_t* address, uint8_t enable, uint8_t enable_aa) {
    uint8_t value;

    // store the pipe 0 address so that it can be overwritten when transmitting with auto-ack enabled.
    if (pipe == 0) {
        rx_pipe0_address[0] = address[0];
        rx_pipe0_address[1] = address[1];
        rx_pipe0_address[2] = address[2];
        rx_pipe0_address[3] = address[3];
        rx_pipe0_address[4] = address[4];
    }

    // Set the address.  We set this stuff even if the pipe is being disabled, because for example the transmitter
    // needs pipe 0 to have the same address as the Tx address for auto-ack to work, even if pipe 0 is disabled.
    set_register(RX_ADDR_P0 + pipe, address, pipe > 1 ? 1 : ADDRESS_LENGTH);

    // Set auto-ack.
    get_register(EN_AA, &value, 1);
    if (enable_aa) {
        value |= (1<<pipe);
    } else {
        value &= ~(1<<pipe);
    }
    set_register(EN_AA, &value, 1);

    // Enable or disable the pipe.
    get_register(EN_RXADDR, &value, 1);
    if (enable) {
        value |= (1<<pipe);
    } else {
        value &= ~(1<<pipe);
    }
    set_register(EN_RXADDR, &value, 1);

}

// default transmitter address is 0xe7e7e7e7e7.
void Radio_Set_Tx_Addr(const uint8_t* address) {
    tx_address[0] = address[0];
    tx_address[1] = address[1];
    tx_address[2] = address[2];
    tx_address[3] = address[3];
    tx_address[4] = address[4];
    set_register(TX_ADDR, address, ADDRESS_LENGTH);
}

void Radio_Transmit(const void* payload, uint8_t len) {
    Radio_Transmit_Wait();

    // indicate that the driver is transmitting.
    transmit_lock = 1;

    // disable the radio while writing to the Tx FIFO.
    CE_LOW();

    set_tx_mode();

    // for auto-ack to work, the pipe0 address must be set to the Tx address while the radio is transmitting.
    // The register will be set back to the original pipe 0 address when the TX_DS or MAX_RT interrupt is asserted.
    set_register(RX_ADDR_P0, (uint8_t*)tx_address, ADDRESS_LENGTH);

    // transfer the packet to the radio's Tx FIFO for transmission
    CSN_LOW();
    send_spi(W_TX_PAYLOAD);
    for (uint8_t i = 0; i < len; i++) {
        send_spi(((uint8_t*)payload)[i]);
    }
    CSN_HIGH();

    // start the transmission.
    CE_HIGH();

}

uint8_t Radio_Transmit_Wait() {
    while (transmit_lock) {}
    return tx_last_status;
}


uint8_t Radio_Receive(const void* buffer, uint8_t buffer_len) {
    CE_LOW();

    uint8_t len = 0;
    uint8_t status = get_status();
    uint8_t pipe_number =  (status & 0xE) >> 1;

    if (pipe_number != PIPE_EMPTY) {
        // Read the payload length
        CSN_LOW();
        send_spi(R_RX_PL_WID);
        len = send_spi(0);
        CSN_HIGH();

        // Clamp the packet length to the buffer length
        if(len>buffer_len) {
            len = buffer_len;
        }

        // Move the data payload to the buffer
        CSN_LOW();
        send_spi(R_RX_PAYLOAD);
        for (uint8_t i = 0; i < len; i++) {
            ((uint8_t*)buffer)[i] = send_spi(0);
        }
        CSN_HIGH();
    }

    CE_HIGH();
    return len;
}

void Radio_Flush() {
    send_instruction_simple(FLUSH_TX);
    send_instruction_simple(FLUSH_RX);
}

void Radio_DumpStatus() {
    for(uint8_t reg=0;reg<0x18;reg++) {
        printf("%02x ", reg);
    }
    puts("");
    uint8_t x;
    for(uint8_t reg=0;reg<0x18;reg++) {
        get_register(reg, &x, 1);
        printf("%02x ", x);
    }
    puts("");
}

// Interrupt handler
ISR(IRQ_VECTOR) {
    CE_LOW();

    uint8_t status = get_status();

    // Do we have new RX darta?
    if (status & (1<<RX_DR)) {
        uint8_t pipe_number =  (status & 0xE) >> 1;
        radio_rxhandler(pipe_number);
        send_instruction_simple(FLUSH_RX);
    }

    // We can get the TX_DS or the MAX_RT interrupt, but not both.
    if (status & (1<<TX_DS)) {
        reset_pipe0_address();
        set_rx_mode();
        tx_last_status = RADIO_TX_SUCCESS;
        transmit_lock = 0;
    } else if (status & (1<<MAX_RT)) {
        puts("MAX_RT");
        send_instruction_simple(FLUSH_TX);
        reset_pipe0_address();
        set_rx_mode();
        tx_last_status = RADIO_TX_MAX_RT;
        transmit_lock = 0;
    }

    // clear the interrupt flags.
    status = (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT);
    set_register(STATUS, &status, 1);

    CE_HIGH();
}

