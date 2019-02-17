#include <cstring>
#include "LoraHandlers.h"
#include "LoraStates.h"
#include "ReceivedMessage.h"
#include "../sx1276/sx1276-hal.h"

static SX1276MB1xAS* lora_radio;
static volatile LoraStates_e* lora_state;
static volatile ReceivedMessage_s* received_msg;

void init_lora_handlers(LoraStates_e* state, SX1276MB1xAS* radio, ReceivedMessage_s* received_message)
{
    lora_state = state;
    lora_radio = radio;
    received_msg = received_message;

    return;
}

void on_rx_done(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr)
{
    lora_radio->Sleep();

    received_msg->payload_size = size;
    received_msg->rssi = rssi;
    received_msg->snr = snr;
    memset((void *)(received_msg->payload), 0, size);
    memcpy((void *)(received_msg->payload), payload, size);

    *lora_state = LoraStates_e::RECEIVED;
    return;
}

void on_rx_timeout(void)
{
    lora_radio->Sleep();
    *lora_state = LoraStates_e::RECEPTION_TIMEOUT;
    return;
}

void on_rx_error(void)
{
    lora_radio->Sleep();
    // rx_error_count++;
    *lora_state = LoraStates_e::RECEPTION_ERROR;
    return;
}

void on_tx_done(void)
{
    lora_radio->Sleep();
    *lora_state = LoraStates_e::TRANSMITTED;
    return;
}

void on_tx_timeout(void)
{
    lora_radio->Sleep();
    *lora_state = LoraStates_e::TRANSMISSION_TIMEOUT;
    return;
}
