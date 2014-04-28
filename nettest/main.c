#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "net.h"

#define BAUD 9600

///============Initialize Prototypes=====//////////////////
void UART_Init(void);
int uart_putchar(char c, FILE *stream);
int uart_getchar(FILE *stream);
char uart_haschar(void);

FILE uart = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

net_address address[2] = {
    {0x11, 0x23, 0x45, 0x67, 0x89},
    {0x12, 0x76, 0x54, 0x32, 0x10},
};

void ping_resp_handler(net_address from);

net_handlers handlers = {
    ping_resp_handler
};

uint16_t tx_count = 0;

int main(void)
{
    UART_Init();
    Net_Init(&handlers);
    Net_SetNodeAddress(address[0]);

    //setup LED
    DDRD |= 1<<PORTD2;

    sei();

    puts("READY\n");
    wdt_enable(WDTO_8S);

    char mode = 0;
    while(1)
    {
        wdt_reset();
        Net_Process();
        if (uart_haschar())
        {
            mode = getchar();
        }

        switch(mode)
        {
            case 'u':
                wdt_enable(WDTO_1S);
                while(1); //Wait for WTD to timeout
                break;
            case 's':
                //Radio_DumpStatus();
                mode = 0;
                break;
            case 'b':
                mode = 0;
            case 'B':
                puts("Ping.");
                Net_SendPing(NET_BROADCAST);
                _delay_ms(250);
                break;
            case '0':
                Net_SetNodeAddress(address[0]);
                mode = 0;
                break;
            case '1':
                Net_SetNodeAddress(address[1]);
                mode = 0;
                break;
            default:
                mode = 0;
        }
    }
}
char led_toggler = 0;
void ping_resp_handler(net_address from) {
    printf("Ping Resp from %x\n", from[0]);
    led_toggler = !led_toggler;
    if (led_toggler) {
        PORTD |= 1<<PORTD2;
    } else {
        PORTD &= ~(1<<PORTD2);
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
