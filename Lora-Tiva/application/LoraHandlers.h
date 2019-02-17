#ifndef __LORA_HANDLERS_H___
#define __LORA_HANDLERS_H___

#include "LoraHandlers.h"
#include "LoraStates.h"
#include "ReceivedMessage.h"
#include "../sx1276/sx1276-hal.h"

extern void init_lora_handlers(LoraStates_e* state, SX1276MB1xAS* radio, ReceivedMessage_s* received_message);
extern void on_rx_done(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr);
extern void on_rx_timeout(void);
extern void on_rx_error(void);
extern void on_tx_done(void);
extern void on_tx_timeout(void);

#endif // __LORA_HANDLERS_H___
