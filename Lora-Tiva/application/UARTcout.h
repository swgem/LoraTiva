#ifndef __UART_COUT_H__
#define __UART_COUT_H__

#include <ostream>
#include <string>

class UARTcout
{
public:
    friend UARTcout& operator<<(UARTcout& out, const std::string &str);
    friend UARTcout& operator<<(UARTcout& out, const int num);
};

#endif // __UART_COUT_H__
