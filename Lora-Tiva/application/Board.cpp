#include <cstdint>
#include "Board.h"
#include "LedColors.h"
#include "../config.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

Board::Board(void)
{
    return;
}

Board::~Board(void)
{
    return;
}

void Board::init(void)
{
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    ROM_FPULazyStackingEnable();

    // Set the clocking to run directly from the crystal
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    // Enable the GPIO port that is used for the on-board LED
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Enable the GPIO pins for the LED (PF1, PF2 & PF3)
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    // Initialize the UART
    this->configure_uart();

    this->test_blink_led(INIT_BOARD_TEST_PERIOD, INIT_TEST_BLINK_PERIOD);

    return;
}

void Board::delay_ms(const uint32_t times_ms)
{
    SysCtlDelay((SysCtlClockGet() / (1000*6)) * (times_ms));

    return;
}

void Board::turn_on_led(const LedColors_e color)
{
    switch (color)
    {
        case LedColors_e::RED:
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
        }
        break;

        case LedColors_e::GREEN:
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
        }
        break;

        case LedColors_e::BLUE:
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        }
        break;

        case LedColors_e::YELLOW:
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
        }
        break;

        case LedColors_e::WHITE:
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
        }
        break;

        default:
        {

        }
        break;
    }

    return;
}

void Board::turn_off_led(void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);

    return;
}

void Board::test_blink_led(const uint32_t test_period_ms, const uint32_t blink_period_ms)
{
    uint8_t on = false;
    uint32_t remaining_time_ms = test_period_ms;

    while (remaining_time_ms > (blink_period_ms / 2))
    {
        if (on)
        {
            this->turn_off_led();
            on = false;
        }
        else
        {
            this->turn_on_led(LedColors_e::WHITE);
            on = true;
        }

        this->delay_ms(blink_period_ms / 2);
        remaining_time_ms -= (blink_period_ms / 2);
    }

    this->turn_off_led();

    return;
}

void Board::configure_uart(void)
{
    // Enable the GPIO Peripheral used by the UART
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O
    UARTStdioConfig(0, 115200, 16000000);
}
