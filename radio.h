/*
 * radio.h
 *
 *  Created on: 24-Jan-2009
 *      Author: Neil MacMillan
 *
 *  References:
 *    Ball, Brennen.  DIYEmbedded Tutorials.  http://www.diyembedded.com/
 *    Engelke, Stefan.  AVR-Lib/nRF24L01.  http://www.tinkerer.eu/AVRLib/nRF24L01
 *    Nordic Semiconductor.  nRF24L01 Product Specification 2.0.  http://www.nordicsemi.no/files/Product/data_sheet/nRF24L01_Product_Specification_v2_0.pdf
 *
 *    Most of the hard work for this was done by [Engelke].
 */

#ifndef RADIO_H_
#define RADIO_H_

#define RADIO_ADDRESS_LENGTH 5

typedef enum _radio_tx_power {
    RADIO_LOWEST_POWER = 0,        // -18 dBm (about 16 uW)
    RADIO_LOW_POWER = 1,        // -12 dBm (about 63 uW)
    RADIO_HIGH_POWER = 2,        // -6 dBm (about 251 uW)
    RADIO_HIGHEST_POWER = 3,    // 0 dBm (1 mW)
} RADIO_TX_POWER;

typedef enum _radio_dr {
    RADIO_250KBPS = 0,
    RADIO_1MBPS = 1,
    RADIO_2MBPS = 2,
} RADIO_DATA_RATE;

typedef enum _radio_transmit {
    RADIO_TX_MAX_RT,
    RADIO_TX_SUCCESS,
} RADIO_TX_STATUS;

void Radio_Init(void);


/**
 * Configure one of the radio's six Rx pipes.
 * This configures a pipe's address and enables or disables the pipe.  Pipes 0 and 1 are enabled by default, and pipes 2-5 are
 *         disabled by default.  The configuration for pipe 0 will be changed while the radio is transmitting, to facilitate auto-
 *         ack.  It will be changed back when the transmission is completed.
 * \param pipe The pipe to configure.
 * \param address The 1- or 5-byte address to give the pipe.  For pipes 0 and 1 all five bytes can be different, but
 *         pipes 2-5 share the four most significant bytes of pipe 1's address.  The LSB of each pipe's address must be unique.
 *         For example:
 *                 Pipe 0: 0x0123456789
 *                 Pipe 1: 0x9876543210
 *                 Pipe 2: 0x98765432AB
 *                 Pipe 3: 0x98765432BC
 *                 Pipe 4: 0x98765432CD
 *                 Pipe 5: 0x98765432DE
 *         If pipe 0 or 1 is being configured, then address must be a 5-byte array.  If the other four pipes are being configured,
 *         then the first byte of address is used as the LSB of the pipe's address (i.e. you only pass a 1-byte address, with the
 *         four MSBytes of the pipe's address left implied).  For example, this will set the first four pipe addresses above:
 *                 uint8_t address[5] = {0x01, 0x23, 0x45, 0x67, 0x89};
 *                 Radio_Configure_Rx(RADIO_PIPE_0, address, ENABLE);
 *                 address = {0x98, 0x76, 0x54, 0x32, 0x10};
 *                 Radio_Configure_Rx(RADIO_PIPE_1, address, ENABLE);
 *                 address[0] = 0xAB;
 *                 Radio_Configure_Rx(RADIO_PIPE_2, address, ENABLE);
 *                 address[0] = 0xBC;
 *                 Radio_Configure_Rx(RADIO_PIPE_3, address, ENABLE);
 *                 ...
 * \param enable Enable or disable the pipe.
 */
void Radio_Configure_Rx(uint8_t pipe, const uint8_t* address, uint8_t enable);

/**
 * Configure the radio transceiver.
 * \param channel The RF channel number (0-125)
 * \param dr The data rate at which the radio will transmit and receive data (1 Mbps or 2 Mbps).
 * \param power The transmitter's power output.
 */
void Radio_Configure(uint8_t channel, RADIO_DATA_RATE dr, RADIO_TX_POWER power);

/**
 * Set the radio transmitter's address.
 * \param The 5-byte address that packets will be sent to.
 */
void Radio_Set_Tx_Addr(const uint8_t* address);

/**
 * Transmit some data to another station.
 * \param payload The data packet to transmit.
 * \param len Length of the payload
 */
void Radio_Transmit(const void* payload, uint8_t len);

/**
 * Wait for the current TX to be complete
 */
uint8_t Radio_Transmit_Wait(void);

/**
 * Get the next packet from the Rx FIFO.
 * \param buffer If there is a packet to copy out of the Rx FIFO, then its payload will be placed in this structure.
 *         If the FIFO is empty, then this structure will be left alone.
 * \param buffer_len Length of buffer
 * \return Number of bytes read (0 means no packet was ready)
 */
uint8_t Radio_Receive(const void* buffer, uint8_t buffer_len);

/**
 * Flush the radio's Rx and Tx FIFOs.
 */
void Radio_Flush(void);

void Radio_DumpStatus(void);

#endif /* RADIO_H_ */
