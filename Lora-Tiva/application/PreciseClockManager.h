#ifndef __PRECISE_CLOCK_MANAGER_H___
#define __PRECISE_CLOCK_MANAGER_H___

#include "Board.h"
#include "SPIStates.h"

class PreciseClockManager
{
public:
    PreciseClockManager(Board* b);
    ~PreciseClockManager(void);
    void init(void);
    uint32_t get_current_time_ns(void);

protected:

private:
    Board* board;

    uint8_t get_byte_spi(SPIStates_e state);
};

#endif // __PRECISE_CLOCK_MANAGER_H___
