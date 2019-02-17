#ifndef __LORA_TRACKER_H___
#define __LORA_TRACKER_H___

#include <cstdint>
#include "LoraDevice.h"
#include "Board.h"

class LoraTracker : public LoraDevice
{
public:
    LoraTracker(Board* b);
    ~LoraTracker(void);

protected:

private:
    uint8_t timestamp_index;

    void reception_handle(void);
    void reception_timeout_handle(void);
    void reception_error_handle(void);
    void transmission_handle(void);
    void transmission_timeout_handle(void);
    void init(void);
};

#endif // __LORA_TRACKER_H___
