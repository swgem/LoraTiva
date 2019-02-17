#ifndef __LORA_STATES_H___
#define __LORA_STATES_H___

enum class LoraStates_e
{
    RECEIVED,
    RECEPTION_TIMEOUT,
    RECEPTION_ERROR,
    TRANSMITTED,
    TRANSMISSION_TIMEOUT,
    IDLE
};

#endif // __LORA_STATES_H___
