#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "radio.h"
#include "net.h"

typedef enum {
    EMPTY = 0,
    PING_REQ = 1,
    PING_RESP = 2
} packet_type;

typedef struct {
    uint8_t type;
    net_address from;
} packet_header;

typedef struct {
    packet_header header;
} ping_req;

typedef struct {
    packet_header header;
} ping_resp;

typedef union {
    packet_header header;
    ping_req ping_req;
    ping_resp ping_resp;
    uint8_t raw[32];
} packet;


net_address NET_BROADCAST = {0xc0, 0xc0, 0xc0, 0xc0, 0xc0};

packet _rx_packet = {{0}};

static net_address _node_address;
static net_handlers* _handlers;

void Net_Init(net_handlers* handlers) {
    Radio_Init();
    Radio_Configure(110, RADIO_250KBPS, RADIO_HIGH_POWER);
    Radio_Configure_Rx(1, NET_BROADCAST, 1, 0);
    _handlers = handlers;
}

void Net_SetNodeAddress(net_address addr) {
    //TODO: write to EEPROM
    Radio_Configure_Rx(0, addr, 1, 1);
    memcpy(_node_address, addr, sizeof(_node_address));
}

void Net_Send(net_address to_addr, void* pkt, uint8_t len) {
    packet_header* h = (packet_header*)pkt;
    memcpy(h->from, _node_address, sizeof(_node_address));
    Radio_Set_Tx_Addr(to_addr);
    Radio_Transmit(pkt, len);
}

void Net_SendPing(net_address addr) {
    ping_req pkt;
    pkt.header.type = PING_REQ;
    Net_Send(addr, &pkt, sizeof(pkt));
}

void Net_Process() {
    if(_rx_packet.header.type == EMPTY) {
        return;
    }

    // Atomically copy any received packet so we can process it while still having interrupts enabled
    packet pkt;
    cli();
    memcpy(&pkt, &_rx_packet, sizeof(packet));
    _rx_packet.header.type = EMPTY;
    sei();

    switch (pkt.header.type) {
        case PING_REQ:
            //printf("Process: PING_REQ from %0x\n", pkt.header.from[0]);
            {
                ping_resp resp;
                resp.header.type = PING_RESP;
                Net_Send(pkt.header.from, &resp, sizeof(resp));
            }
            break;
        case PING_RESP:
            //printf("Process: PING_RESP from %0x\n", pkt.header.from[0]);
            if (_handlers->ping_resp) {
                (*_handlers->ping_resp)(pkt.header.from);
            }
            break;
        default:
            printf("Process: type:%d\n", pkt.header.type);
            break;
    }
}

void radio_rxhandler(uint8_t pipenumber) {
    if (_rx_packet.header.type == EMPTY) {
        Radio_Receive(&_rx_packet, sizeof(_rx_packet));
    } else {
        puts("RX: dropping packet");
    }
}

