#include <cstdint>
#include "PreciseClockManager.h"
#include "SPIStates.h"
#include "../config.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

PreciseClockManager::PreciseClockManager(Board* b) : board { b }
{
	return;
}

PreciseClockManager::~PreciseClockManager(void)
{
	return;
}

void PreciseClockManager::init(void)
{
    // PINs configuration for SPI

    // mosi - SSI0Tx  - PA5
    // miso - SSI0Rx  - PA4
    // sclk - SSI0Clk - PA2
    // nss  - SSI0Fss  - PA3

    // Enable the GPIO pin for SSEL
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5);
    // Baud rate : 1,125 MBits/s;
    SSIConfigSetExpClk(SSI0_BASE,SysCtlClockGet(),SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,1125000,8);
    SSIEnable(SSI0_BASE);

    board->delay_ms(100);

	return;
}

uint32_t PreciseClockManager::get_current_time_ns(void)
{
    uint32_t curr_ticks = 0;
    uint32_t curr_time_ns;

    // Get ticks byte by byte from slave
    curr_ticks = ((uint32_t)this->get_byte_spi(r_ticks_byte0)) << 0;
    curr_ticks |= ((uint32_t)this->get_byte_spi(r_ticks_byte1)) << 8;
    curr_ticks |= ((uint32_t)this->get_byte_spi(r_ticks_byte2)) << 16;
    curr_ticks |= ((uint32_t)this->get_byte_spi(r_ticks_byte3)) << 24;

    // Convert from ticks to ns
    curr_time_ns = curr_ticks * NS_PER_TICK;

    return curr_time_ns;
}

uint8_t PreciseClockManager::get_byte_spi(SPIStates_e state)
{
	uint32_t dummy = 0;
    uint32_t r_data;
    uint8_t r_byte;

    // Nss = 0;
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);

    // Send state to slave
    SSIDataPut(SSI0_BASE, state);
    while(SSIBusy(SSI0_BASE));
    SSIDataGet(SSI0_BASE, &dummy);

    board->delay_ms(1);

    // Get the byte related to the sent state
    SSIDataPut(SSI0_BASE, dummy);
    while(SSIBusy(SSI0_BASE));
    SSIDataGet(SSI0_BASE, &r_data);

    r_byte = (uint8_t)r_data;

    // Nss = 1;
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);

    board->delay_ms(1);

    return r_byte;
}
