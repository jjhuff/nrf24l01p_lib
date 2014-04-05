#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "radio.h"

#define BAUD 9600

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

///============Initialize Prototypes=====//////////////////
void UART_Init(void);
int uart_putchar(char c, FILE *stream);
int uart_getchar(FILE *stream);
char uart_haschar(void);

FILE uart = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

uint8_t address[3][5] = {
    {0x01, 0x23, 0x45, 0x67, 0x89},
    {0x98, 0x76, 0x54, 0x32, 0x10},
    {0x45, 0x34, 0x76, 0x23, 0x67},
};

uint16_t tx_count = 0;

#define MODE_UPGRADE           'u'
#define MODE_STATUS            's'
#define MODE_TX_0              't'
#define MODE_TX_1              'T'
#define MODE_TX_B              'b'
#define MODE_0                 '0'
#define MODE_1                 '1'

int main(void)
{
    UART_Init();
    _delay_ms(100);
    Radio_Init();
    _delay_ms(200);
    Radio_Configure(RADIO_1MBPS, RADIO_LOW_POWER);
    //Radio_Configure_Rx(RADIO_PIPE_0, address0, ENABLE);
    Radio_Configure_Rx(RADIO_PIPE_1, address[2], ENABLE);
    //Radio_Set_Tx_Addr(address0);

    sei();

    puts("READY\n");
    wdt_enable(WDTO_8S);

    char mode = 0;
    radiopacket_t pkt;
    pkt.type = MESSAGE;
    while(1)
    {
        wdt_reset();
        if (uart_haschar())
        {
            mode = getchar();
        }

        switch(mode)
        {
            case MODE_UPGRADE:
                wdt_enable(WDTO_1S);
                while(1); //Wait for WTD to timeout
                break;
            case MODE_STATUS:
                Radio_DumpStatus();
                mode = 0;
                break;
            case MODE_TX_0:
                pkt.timestamp = (tx_count++);
                printf("TX 0: %d\n", pkt.timestamp);
                Radio_Set_Tx_Addr(address[0]);
                Radio_Transmit(&pkt, RADIO_RETURN_ON_TX);
                mode = 0;
                break;
            case MODE_TX_1:
                pkt.timestamp = (tx_count++);
                printf("TX 1: %d\n", pkt.timestamp);
                Radio_Set_Tx_Addr(address[1]);
                Radio_Transmit(&pkt, RADIO_RETURN_ON_TX);
                mode = 0;
                break;
            case MODE_TX_B:
                pkt.timestamp = (tx_count++);
                printf("TX B: %d\n", pkt.timestamp);
                Radio_Set_Tx_Addr(address[2]);
                Radio_Transmit(&pkt, RADIO_RETURN_ON_TX);
                mode = 0;
                break;
            case MODE_0:
                Radio_Configure_Rx(RADIO_PIPE_0, address[0], ENABLE);
                mode = 0;
                break;
            case MODE_1:
                Radio_Configure_Rx(RADIO_PIPE_0, address[1], ENABLE);
                mode = 0;
                break;
            default:
                mode = 0;
        }
    }
}
void radio_rxhandler(uint8_t pipenumber) {
    radiopacket_t pkt;
    uint8_t ret = RADIO_RX_MORE_PACKETS;
    while(ret == RADIO_RX_MORE_PACKETS) {
        pkt.timestamp = 0xFF;
        ret=Radio_Receive(&pkt);
        printf("RX: %d\n", pkt.timestamp);
    }
}

/*********************
 ****Initialize****
 *********************/
void UART_Init()
{
    unsigned int ubrr = (unsigned int)(F_CPU/16/(BAUD)-1);
    // Set baud rate
    UBRR0H = ubrr>>8;
    UBRR0L = ubrr;

    // Enable receiver and transmitter
    UCSR0A = (0<<U2X0);
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);

    // Set frame format: 8 bit, no parity, 1 stop bit,
    UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);

    stdout = &uart;
    stdin = &uart;
}

int uart_putchar(char c, FILE *stream)
{
    if (c == '\n') uart_putchar('\r', stream);

    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;

    return 0;
}

int uart_getchar(FILE *stream)
{
    loop_until_bit_is_set(UCSR0A, RXC0); /* Wait until data exists. */
    return UDR0;
}

char uart_haschar(void)
{
    return (UCSR0A & 1<<RXC0) > 0;
}
