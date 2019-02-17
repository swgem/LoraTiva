#ifndef __BOARD_H__
#define __BOARD_H__

#include <cstdint>
#include <cstdarg>
#include "UARTcout.h"
#include "LedColors.h"

class Board
{
public:
	UARTcout UARTcout;

    Board(void);
    ~Board(void);
    void init(void);
    void delay_ms(const uint32_t times_ms);
    void turn_on_led(const LedColors_e color);
    void turn_off_led(void);

protected:

private:
	void configure_uart(void);
};

#endif // __BOARD_H__
