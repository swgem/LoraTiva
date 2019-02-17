#include "../config.h"

#ifdef DEVICE_MODE_TRACKER

#include <cstdint>
#include "LoraTracker.h"
#include "Board.h"

LoraTracker::LoraTracker(Board* b) : LoraDevice { b },
                                     timestamp_index { 0 }
{
    return;
}

LoraTracker::~LoraTracker(void)
{
    return;
}

void LoraTracker::reception_handle(void)
{
    this->radio->Rx(TX_RCT_TIMEOUT_VALUE_US);

    return;
}

void LoraTracker::reception_timeout_handle(void)
{
    this->radio->Rx(TX_RCT_TIMEOUT_VALUE_US);

    return;
}

void LoraTracker::reception_error_handle(void)
{
    this->radio->Rx(TX_RCT_TIMEOUT_VALUE_US);

    return;
}

void LoraTracker::transmission_handle(void)
{
    if (timestamp_index == TX_MESSAGES_PER_SEQUENCE)
    {
        this->board->UARTcout << "Transmission sequence over\r\n\r\n";

        // Wait and then restart transmission
        this->board->delay_ms(TX_SILENCE_PERIOD_MS);
        this->timestamp_index = 0;

        this->board->UARTcout << "Restarting sequence\r\n";
    }

    // Send message with sequence number
    TimestampMessage_s msg_tx { DEVICE_ID, this->timestamp_index, -1, -1, -1 };
    this->transmit_message((uint8_t *)&msg_tx, sizeof(msg_tx), TX_SEQUENCE_PERIOD_MS);
    this->timestamp_index++;

    this->print_sent_timestamp_message(&msg_tx);

    return;
}

void LoraTracker::transmission_timeout_handle(void)
{
    this->board->UARTcout << "Message not sent\r\n";

    this->board->delay_ms(TX_TIMEOUT_SILENCE_PERIOD_MS);
    
    this->board->UARTcout << "Restarting sequence\r\n";

    // Restart sequence transmission
    this->timestamp_index = 0;
    TimestampMessage_s msg_tx { DEVICE_ID, this->timestamp_index, -1, -1, -1 };
    this->transmit_message((uint8_t *)&msg_tx, sizeof(msg_tx), TX_SEQUENCE_PERIOD_MS);
    this->timestamp_index++;

    this->print_sent_timestamp_message(&msg_tx);

    return;
}

void LoraTracker::init(void)
{
    TimestampMessage_s msg_tx { DEVICE_ID, this->timestamp_index, -1, -1, -1 };
    this->transmit_message((uint8_t *)&msg_tx, sizeof(msg_tx), TX_SEQUENCE_PERIOD_MS);
    this->timestamp_index++;

    this->print_sent_timestamp_message(&msg_tx);

    return;
}

#endif // DEVICE_MODE_TRACKER
