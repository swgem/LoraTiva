#ifndef __LORA_DEVICE_H___
#define __LORA_DEVICE_H___

#include <cstdint>
#include "Board.h"
#include "LoraStates.h"
#include "ReceivedMessage.h"
#include "TimestampMessage.h"
#include "../sx1276/sx1276-hal.h"

class LoraDevice
{
public:
    LoraDevice(Board* b);
    ~LoraDevice(void);
    void start(void);

protected:
    Board* board;
    SX1276MB1xAS* radio;
    volatile ReceivedMessage_s received_msg;
    volatile TimestampMessage_s* timestamp_msg;

    void transmit_message(uint8_t *msg, uint16_t msg_size, uint16_t delay_ms);
    void print_sent_timestamp_message(TimestampMessage_s* msg);
    virtual void reception_handle(void);
    virtual void reception_timeout_handle(void);
    virtual void reception_error_handle(void);
    virtual void transmission_handle(void);
    virtual void transmission_timeout_handle(void);
    virtual void init(void);

private:
    volatile LoraStates_e lora_state;
    RadioEvents_t radio_events;

    void loop(void);
    void config_radio(void);
};

#endif // __LORA_DEVICE_H___
