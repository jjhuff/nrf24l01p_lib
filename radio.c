/*
 * radio.c
 *
 *  Created on: 24-Jan-2009
 *      Author: Neil MacMillan
 */
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "nRF24L01.h"
#include "packet.h"
#include "spi.h"

#include "radio.h"

// non-public constants and macros

#define CHANNEL 112
#define ADDRESS_LENGTH 5

// Pin definitions for chip select and chip enable on the radio module
#define CE_DDR        DDRB
#define CE_PORT       PORTB
#define CE_PIN        PORTB0

#define CSN_DDR       DDRB
#define CSN_PORT      PORTB
#define CSN_PIN       PORTB1

#define IRQ_DDR       DDRD
#define IRQ_PORT      PORTD
#define IRQ_PIN       PORTD6

// Definitions for selecting and enabling the radio
#define CSN_HIGH()    CSN_PORT |=  1<<CSN_PIN;
#define CSN_LOW()     CSN_PORT &= ~(1<<CSN_PIN);
#define CE_HIGH()     CE_PORT |=  1<<CE_PIN;
#define CE_LOW()      CE_PORT &= ~(1<<CE_PIN);

// Flag which denotes that the radio is currently transmitting
static volatile uint8_t transmit_lock;

// tracks the payload widths of the Rx pipes
static volatile uint8_t rx_pipe_widths[6] = {32, 32, 0, 0, 0, 0};

// holds the transmit address (Rx pipe 0 is set to this address when transmitting with auto-ack enabled).
static volatile uint8_t tx_address[5] = { 0xe7, 0xe7, 0xe7, 0xe7, 0xe7 };

// holds the receiver address for Rx pipe 0 (the address is overwritten when transmitting with auto-ack enabled).
static volatile uint8_t rx_pipe0_address[5] = { 0xe7, 0xe7, 0xe7, 0xe7, 0xe7 };

// the driver keeps track of the success status for the last 16 transmissions
static volatile uint16_t tx_history = 0xFF;

static volatile RADIO_TX_STATUS tx_last_status = RADIO_TX_SUCCESS;

extern void radio_rxhandler(uint8_t pipenumber);

/**
 * Retrieve the status register.
 */
static uint8_t get_status(void) {
    uint8_t status = 0;
    CSN_LOW();

    status = send_spi(NOP);

    CSN_HIGH();

    return status;
}

/**
 * Set a register in the radio
 * \param reg The register value defined in nRF24L01.h (e.g. CONFIG, EN_AA, &c.).
 * \param value The value to write to the given register (the whole register is overwritten).
 * \return The status register.
 */
static uint8_t set_register(radio_register_t reg, uint8_t* value, uint8_t len) {
    uint8_t status;
    CSN_LOW();

    status = send_spi(W_REGISTER | (REGISTER_MASK & reg));

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
    uint8_t status, i;
    for (i = 0; i < len; i++) {
        // If the buffer is too long for the register results, then the radio will interpret the extra bytes as instructions.
        // To remove the risk, we set the buffer elements to NOP instructions.
        buffer[i] = 0xFF;
    }
    CSN_LOW();

    status = send_spi(R_REGISTER | (REGISTER_MASK & reg));
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = send_spi(0);
    }

    CSN_HIGH();

    return status;
}

/**
 * Send an instruction to the nRF24L01.
 * \param instruction The instruction to send (see the bottom of nRF24L01.h)
 * \param data An array of argument data to the instruction.  If len is 0, then this may be NULL.
 * \param buffer An array for the instruction's return data.  This can be NULL if the instruction has no output.
 * \param len The length of the data and buffer arrays.
 */
static void send_instruction(uint8_t instruction, uint8_t* data, uint8_t* buffer, uint8_t len) {
    CSN_LOW();
    // send the instruction
    send_spi(instruction);
    // pass in args
    if (len > 0) {
        if (buffer == NULL) {   //
            for (uint8_t i = 0; i < len; i++) {
                send_spi(data[i]);
            }
        } else {
            for (uint8_t i = 0; i < len; i++) {
                data[i] = send_spi(buffer[i]);
            }
        }
    }
    // resynch SPI
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
        _delay_us(65);
        _delay_us(65);
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
        // You can delete this if you're sending large packets (I'm thinking > 25 bytes, but I'm not sure) because it
        // sending the bytes over SPI can take this long.
        _delay_us(65);
        _delay_us(65);
    }
}

/**
 * Reset the pipe 0 address if pipe 0 is enabled.  This is necessary when the radio is using Enhanced Shockburst, because
 * the pipe 0 address is set to the transmit address while the radio is transmitting (this is how the radio receives
 * auto-ack packets).
 */
static void reset_pipe0_address(void) {
    if (rx_pipe_widths[RADIO_PIPE_0] != 0) {
        // reset the pipe 0 address if pipe 0 is enabled.
        set_register(RX_ADDR_P0, (uint8_t*)rx_pipe0_address, ADDRESS_LENGTH);
    }
}

/**
 * Configure radio defaults and turn on the radio in receive mode.
 * This configures the radio to its max-power, max-packet-header-length settings.  If you want to reduce power consumption
 * or increase on-air payload bandwidth, you'll have to change the config.
 */
static void configure_registers(void) {
    uint8_t value;

    setup_spi(SPI_MODE_0, SPI_MSB, SPI_NO_INTERRUPT, SPI_MSTR_CLK16);

    // set address width to 5 bytes.
    value = ADDRESS_LENGTH - 2;            // 0b11 for 5 bytes, 0b10 for 4 bytes, 0b01 for 3 bytes
    set_register(SETUP_AW, &value, 1);

    // set Enhanced Shockburst retry to every 586 us, up to 5 times.  If packet collisions are a problem even with AA enabled,
    // then consider changing the retry delay to be different on the different stations so that they do not keep colliding on each retry.
    value = 0xFF;
    set_register(SETUP_RETR, &value, 1);

    // Set to use 2.4 GHz channel 110.
    value = CHANNEL;
    set_register(RF_CH, &value, 1);

    // Enable 2-byte CRC and power up in receive mode.
    value = (1<<EN_CRC) | (1<<CRCO) | (1<<PWR_UP) | (1<<PRIM_RX);
    set_register(CONFIG, &value, 1);

    // clear the interrupt flags in case the radio's still asserting an old unhandled interrupt
    value = (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT);
    set_register(STATUS, &value, 1);

    // flush the FIFOs in case there are old data in them.
    send_instruction(FLUSH_TX, NULL, NULL, 0);
    send_instruction(FLUSH_RX, NULL, NULL, 0);
}

void Radio_Init() {
    transmit_lock = 0;

    // disable radio during config
    CE_LOW();

    // set as output
    CE_DDR |= 1<<CE_PIN;
    CSN_DDR |= 1<<CSN_PIN;

    // Enable radio interrupt.  This interrupt is triggered when data are received and when a transmission completes.
    IRQ_DDR &= ~(1<<IRQ_PIN);
    PCMSK2 |= (1<<PCINT22);
    PCICR  |= (1<<PCIE2);

    // A 10.3 ms delay is required between power off and power on states (controlled by 3.3 V supply).
    _delay_ms(11);

    // Configure the radio registers that are not application-dependent.
    configure_registers();

    // A 1.5 ms delay is required between power down and power up states (controlled by PWR_UP bit in CONFIG)
    _delay_ms(2);

    // enable radio as a receiver
    CE_HIGH();
}

void Radio_Configure(RADIO_DATA_RATE dr, RADIO_TX_POWER power) {
    uint8_t value;

    if (power < RADIO_LOWEST_POWER || power > RADIO_HIGHEST_POWER || dr < RADIO_250KBPS || dr > RADIO_2MBPS) return;

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
void Radio_Configure_Rx(RADIO_PIPE pipe, uint8_t* address, uint8_t enable) {
    uint8_t value;
    uint8_t use_aa = 1;
    uint8_t payload_width = 32;
    if (payload_width < 1 || payload_width > 32 || pipe < RADIO_PIPE_0 || pipe > RADIO_PIPE_5) return;

    // store the pipe 0 address so that it can be overwritten when transmitting with auto-ack enabled.
    if (pipe == RADIO_PIPE_0) {
        rx_pipe0_address[0] = address[0];
        rx_pipe0_address[1] = address[1];
        rx_pipe0_address[2] = address[2];
        rx_pipe0_address[3] = address[3];
        rx_pipe0_address[4] = address[4];
    }

    // Set the address.  We set this stuff even if the pipe is being disabled, because for example the transmitter
    // needs pipe 0 to have the same address as the Tx address for auto-ack to work, even if pipe 0 is disabled.
    set_register(RX_ADDR_P0 + pipe, address, pipe > RADIO_PIPE_1 ? 1 : ADDRESS_LENGTH);

    // Set auto-ack.
    get_register(EN_AA, &value, 1);
    if (use_aa) {
        value |= (1<<pipe);
    } else {
        value &= ~(1<<pipe);
    }
    set_register(EN_AA, &value, 1);

    // Set the pipe's payload width.  If the pipe is being disabled, then the payload width is set to 0.
    value = enable ? payload_width : 0;
    set_register(RX_PW_P0 + pipe, &value, 1);
    rx_pipe_widths[pipe] = value;

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
void Radio_Set_Tx_Addr(uint8_t* address) {
    tx_address[0] = address[0];
    tx_address[1] = address[1];
    tx_address[2] = address[2];
    tx_address[3] = address[3];
    tx_address[4] = address[4];
    set_register(TX_ADDR, address, ADDRESS_LENGTH);
}

uint8_t Radio_Transmit(radiopacket_t* payload, RADIO_TX_WAIT wait) {
    //if (block && transmit_lock) while (transmit_lock);
    //if (!block && transmit_lock) return 0;
    uint8_t len = 32;

    // indicate that the driver is transmitting.
    transmit_lock = 1;

    // disable the radio while writing to the Tx FIFO.
    CE_LOW();

    set_tx_mode();

    // for auto-ack to work, the pipe0 address must be set to the Tx address while the radio is transmitting.
    // The register will be set back to the original pipe 0 address when the TX_DS or MAX_RT interrupt is asserted.
    set_register(RX_ADDR_P0, (uint8_t*)tx_address, ADDRESS_LENGTH);

    // transfer the packet to the radio's Tx FIFO for transmission
    send_instruction(W_TX_PAYLOAD, (uint8_t*)payload, NULL, len);

    // start the transmission.
    CE_HIGH();

    if (wait == RADIO_WAIT_FOR_TX) {
        while (transmit_lock);
        return tx_last_status;
    }

    return RADIO_TX_SUCCESS;
}

RADIO_RX_STATUS Radio_Receive(radiopacket_t* buffer) {
    uint8_t len = 32;
    uint8_t status;
    uint8_t pipe_number;
    uint8_t doMove = 1;
    RADIO_RX_STATUS result;

    transmit_lock = 0;

    CE_LOW();

    status = get_status();
    pipe_number =  (status & 0xE) >> 1;

    if (pipe_number == RADIO_PIPE_EMPTY) {
        result = RADIO_RX_FIFO_EMPTY;
        doMove = 0;
    }

    if (rx_pipe_widths[pipe_number] > len) {
        // the buffer isn't big enough, so don't copy the data.
        result = RADIO_RX_INVALID_ARGS;
        doMove = 0;
    }

    if (doMove) {
        // Move the data payload into the local
        send_instruction(R_RX_PAYLOAD, (uint8_t*)buffer, (uint8_t*)buffer, rx_pipe_widths[pipe_number]);

        status = get_status();
        pipe_number =  (status & 0xE) >> 1;

        if (pipe_number != RADIO_PIPE_EMPTY)
            result = RADIO_RX_MORE_PACKETS;
        else
            result = RADIO_RX_SUCCESS;
    }

    CE_HIGH();

    transmit_lock = 0;

    return result;
}

// This is only accurate if all the failed packets were sent using auto-ack.
uint8_t Radio_Success_Rate() {
    uint16_t wh = tx_history;
    uint8_t weight = 0;
    while (wh != 0) {
        if ((wh & 1) != 0) weight++;
        wh >>= 1;
    }
    wh = (16 - weight) * 100;
    wh /= 16;
    return wh;
}

void Radio_Flush() {
    send_instruction(FLUSH_TX, NULL, NULL, 0);
    send_instruction(FLUSH_RX, NULL, NULL, 0);
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
ISR(PCINT2_vect) {
    uint8_t status;
    uint8_t pipe_number;

    CE_LOW();

    status = get_status();

    // Do we have new RX darta?
    if (status & (1<<RX_DR)) {
        pipe_number =  (status & 0xE) >> 1;
        radio_rxhandler(pipe_number);
    }

    // We can get the TX_DS or the MAX_RT interrupt, but not both.
    if (status & (1<<TX_DS)) {
        // if there's nothing left to transmit, switch back to receive mode.
        transmit_lock = 0;
        reset_pipe0_address();
        set_rx_mode();

        // indicate in the history that a packet was transmitted successfully by appending a 1.
        tx_history <<= 1;
        tx_history |= 1;

        tx_last_status = RADIO_TX_SUCCESS;
    } else if (status & (1<<MAX_RT)) {
        send_instruction(FLUSH_TX, NULL, NULL, 0);

        transmit_lock = 0;
        reset_pipe0_address();
        set_rx_mode();
        // indicate in the history that a packet was dropped by appending a 0.
        tx_history <<= 1;

        tx_last_status = RADIO_TX_MAX_RT;
    }

    // clear the interrupt flags.
    status = (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT);
    set_register(STATUS, &status, 1);

    CE_HIGH();
}

