#ifndef __NET_H__956456265
#define __NET_H__956456265

#include <stdint.h>

typedef uint8_t net_address[5];

extern net_address NET_BROADCAST;

typedef struct {
    void (*ping_resp) (net_address from);
} net_handlers;

void Net_Init(net_handlers* handlers);
void Net_Process(void);

void Net_SetNodeAddress(net_address addr);

void Net_Send(net_address to_addr, void* pkt, uint8_t len);

void Net_SendPing(net_address addr);
#endif
