#ifndef __LORA_BASE_H___
#define __LORA_BASE_H___

#include <cstdint>
#include "LoraDevice.h"
#include "Board.h"
#include "PreciseClockManager.h"
#include "BaseEvents.h"
#include "BaseStates.h"
#include "TimestampData.h"
#include "../config.h"

class LoraBase : public LoraDevice
{
public:
    LoraBase(Board* b);
    ~LoraBase(void);

protected:

private:
    TimestampData_s timestamp_buffer[RX_TIMESTAMP_BUFFER_SIZE];
#if (DEVICE_ID == 0)
    TimestampData_s timestamp_buffer_base1[RX_TIMESTAMP_BUFFER_SIZE];
    TimestampData_s timestamp_buffer_base2[RX_TIMESTAMP_BUFFER_SIZE];
#else
    uint8_t timestamp_index;
#endif
    BaseStates_e base_state;
    uint16_t wrong_reception_count;
    PreciseClockManager precise_clock_manager;

    void reception_handle(void);
    void reception_timeout_handle(void);
    void reception_error_handle(void);
    void transmission_handle(void);
    void transmission_timeout_handle(void);
    void init(void);
    void set_word_buffer(int32_t *buffer, int32_t value, uint32_t size);
    void execute_state_machine(BaseEvents_e event);
    void print_rcvd_timestamp_message(uint32_t timestamp);
};

#endif // __LORA_BASE_H___
