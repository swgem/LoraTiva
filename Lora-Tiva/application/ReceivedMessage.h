#ifndef __RECEIVED_MESSAGE_H___
#define __RECEIVED_MESSAGE_H___

#include <cstdint>
#include "../config.h"

typedef struct
{
    uint8_t payload[RX_PAYLOAD_SIZE];
    uint16_t payload_size;
    int16_t rssi;
    int8_t snr;
} ReceivedMessage_s;

#endif // __RECEIVED_MESSAGE_H___
