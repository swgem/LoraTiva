#ifndef __TIMESTAMP_MESSAGE_H__
#define __TIMESTAMP_MESSAGE_H__

typedef struct
{
    uint8_t device_id;
    uint8_t message_id;
    int32_t timestamp;
    int16_t rssi;
    int8_t snr;
} TimestampMessage_s;

#endif // __TIMESTAMP_MESSAGE_H__
