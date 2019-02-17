#include "UARTcout.h"
#include <ostream>
#include <string>
#include "utils/uartstdio.h"

UARTcout& operator<<(UARTcout& out, const std::string &str)
{
    UARTprintf(str.c_str());

    return out;
}

UARTcout& operator<<(UARTcout& out, const int num)
{
    std::string str = std::to_string(num);

    UARTprintf(str.c_str());

    return out;
}
