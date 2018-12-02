//*****************************************************************************
//
// hello.cpp - Simple hello world example.
//
// Copyright (c) 2012-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.0.12573 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
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

#include "driverlib/ssi.h"

#include "main.h"

#include "lora.h"
#include "sx1276/sx1276-hal.h"

/******************************************************************
 * DECLARATION OF INTERNAL VARIABLES
 ******************************************************************/

static volatile AppStates_t State = LOWPOWER;

static RadioEvents_t RadioEvents;

static SX1276MB1xAS Radio { NULL };

static uint16_t BufferSize = BUFFER_SIZE;
static uint8_t Buffer[BUFFER_SIZE];

static int16_t RssiValue = 0.0;
static int8_t SnrValue = 0.0;

static uint16_t TimesReceived = 0;
static uint16_t TimesError = 0;

static uint32_t curr_time_ns;

// const uint8_t SendMsg1[] = "                               ";
// const uint8_t RecvMsg1[] = "                               ";

/******************************************************************
 * DECLARATION OF INTERNAL FUNCTIONS
 ******************************************************************/

static void UIntToString(int number, char * out);

static void ConfigureUART(void);

static void blue_led(uint8_t led);

static void ConfigureSpiPreciseClk(void);

static uint32_t GetCurrentTimeNs(void);

static uint8_t GetBytePreciseTimeSpi(SPIState_t state);

/******************************************************************
 * DEFINITION OF EXTERNAL FUNCTIONS
 ******************************************************************/

int main(void)
{
    int led = 0;

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF2 & PF3).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Initialize the UART.
    //
    ConfigureUART();

    //
    // Hello!
    //
    debug_msg("Hello, LoRa!\n");


    debug_msg( "\n\n\r     SX1276 TIVA Cage Trap application \n\n\r" );

    // Initialize Radio driver
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxError = OnRxError;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    Radio.Init( &RadioEvents );

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0xFF);
    // verify the connection with the board
    int32_t radio_version = Radio.Read( REG_VERSION );
    while( Radio.Read( REG_VERSION ) == 0x00  )
    {

    	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x00);
    	SysCtlDelay(SysCtlClockGet() / (2*1000));
    	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0xFF);
    	debug_msg( "Radio could not be detected!\n\r" );
        SysCtlDelay(SysCtlClockGet() / 10 );
    }

    UARTprintf( "Radio Version: %d!\n\r",radio_version);
    debug_msg_if( ( DEBUG_MESSAGE & ( Radio.DetectBoardType( ) == SX1276MB1LAS ) ) , "\n\r > Board Type: SX1276MB1LAS < \n\r" );
    debug_msg_if( ( DEBUG_MESSAGE & ( Radio.DetectBoardType( ) == SX1276MB1MAS ) ) , "\n\r > Board Type: SX1276MB1MAS < \n\r" );

    Radio.SetChannel( RF_FREQUENCY );

    ConfigureSpiPreciseClk();

#ifdef TESTE_SPI
    uint32_t curr_time_ns;

    while(1)
    {
        curr_time_ns = GetCurrentTimeNs();
    }
#endif

#if USE_MODEM_LORA == 1

    debug_msg_if( LORA_FHSS_ENABLED, "\n\n\r             > LORA FHSS Mode < \n\n\r");
    debug_msg_if( !LORA_FHSS_ENABLED, "\n\n\r             > LORA Mode < \n\n\r");

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                      LORA_IQ_INVERSION_ON, 2000000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
                      LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                      LORA_IQ_INVERSION_ON, true );

#elif USE_MODEM_FSK == 1

    debug_msg("\n\n\r              > FSK Mode < \n\n\r");
    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                      FSK_DATARATE, 0,
                      FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                      FSK_CRC_ENABLED, 0, 0, 0, 2000000 );

    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                      0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                      0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, FSK_CRC_ENABLED,
                      0, 0, false, true );

#else

#error "Please define a modem in the compiler options."

#endif

    debug_msg_if( DEBUG_MESSAGE, "Starting Infinite Loop\r\n" );

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x00);
    SysCtlDelay(SysCtlClockGet()/2000);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0xFF);
    SysCtlDelay(SysCtlClockGet()/2000);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x00);

    //
    // Turn off the BLUE LED.
    //
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

    Radio.Rx( RX_TIMEOUT_VALUE );


#ifdef TESTE_TRANSMITE
    char msg_tx[2];
    char msg_id = 0;

    while (1)
    {
        blue_led(1);

        msg_tx[0] = DEVICE_ID;
        msg_tx[1] = msg_id;

        DelayMs(10);

        // debug_msg("ID do device: %i        ID da msg: %i\n", msg_tx[0], msg_id[1]);

        Radio.Send((uint8_t *)msg_tx, sizeof(msg_tx));

        blue_led(0);

        DelayMs(1500);

        msg_id++;
    }
#else
    while( 1 )
    {
        switch( State )
        {
        case RX:
            if( BufferSize > 0 )
            {
                // if( strncmp( ( const char* )Buffer, ( const char* )RecvMsg1, 2 ) == 0 )
                // {
                    // led = !led;
                    // blue_led(led);

                    // debug_msg( "Recebido: \"OK\"\r\n" );
                // }
                // else // valid reception but neither a OK message
                // {
                //     Radio.Rx( RX_TIMEOUT_VALUE );
                // }

                Radio.Rx( RX_TIMEOUT_VALUE );
            }
            State = LOWPOWER;
            break;

        case TX:
            led = !led;
            blue_led(led);

            debug_msg( "ENVIANDO...\r\n" );

            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;

        case RX_TIMEOUT:
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;

        case RX_ERROR:
            // We have received a Packet with a CRC error, blink red led

            //RED LED

            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;

        case TX_TIMEOUT:
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;

        case LOWPOWER:
            break;

        default:
            State = LOWPOWER;
            break;
        }
    }
#endif
}

void OnTxDone(void)
{
    Radio.Sleep( );
    State = TX;
    debug_msg_if( DEBUG_MESSAGE, "> OnTxDone\n\r" );
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    Radio.Sleep( );

    curr_time_ns = GetCurrentTimeNs();

    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
    UARTprintf("size: %d, rss: %d, snr: %d, timestamp: %d, device_id: %d, msg_id: %d\n\r",size,rssi,snr,curr_time_ns,payload[0],payload[1]);
    TimesReceived++;
    // UARTprintf("Received: %d, Error: %d, Sum: %d \n\r",TimesReceived, TimesError, (TimesReceived+TimesError));
}

void OnTxTimeout(void)
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
    debug_msg_if( DEBUG_MESSAGE, "> OnTxTimeout\n\r" );
}

void OnRxTimeout(void)
{
    Radio.Sleep( );
    Buffer[ BufferSize-1 ] = 0;
    State = RX_TIMEOUT;
    debug_msg_if( DEBUG_MESSAGE, "> OnRxTimeout\n\r" );
}

void OnRxError(void)
{
    Radio.Sleep( );
    State = RX_ERROR;
    debug_msg_if( DEBUG_MESSAGE, "> OnRxError\n\r" );
    TimesError++;
    UARTprintf("Received: %d, Error: %d, Sum: %d \n\r",TimesReceived, TimesError, (TimesReceived+TimesError));
}


/******************************************************************
 * DEFINITION OF INTERNAL FUNCTIONS
 ******************************************************************/

static void UIntToString(int number, char * out)
{
    int i = 0, j = 0;
    int aux;

    // Gera numero swapped
    do
    {
        aux = number % 10;
        number = number / 10;

        out[i] = (char)(aux + 0x30);

        i++;
    } while (number != 0);

    // Deswap todos os algarismos
    while (j != i / 2)
    {
        aux = out[j];
        out[j] = out[i - j - 1];
        out[i - j - 1] = aux;

        j++;
    };

    out[i] = '\n';
    out[i + 1] = '\r';
    out[i + 2] = '\0';
}

/**
 * Configure the UART and its pins.  This must be called before UARTprintf().
 */
static void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

static void blue_led(uint8_t led)
{

    if(led)
    {
        //
        // Turn on the BLUE LED.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    }
    else
    {
        //
        // Turn off the BLUE LED.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
    }
}

static void ConfigureSpiPreciseClk(void)
{
    //
    // PINs configuration for SPI
    //

    // mosi - SSI0Tx  - PA5
    // miso - SSI0Rx  - PA4
    // sclk - SSI0Clk - PA2
    // nss  - SSI0Fss  - PA3

    //
    // Enable the GPIO pin for SSEL
    //
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

    //wait(0.1);
    SysCtlDelay(SysCtlClockGet() / 10);
}

static uint32_t GetCurrentTimeNs(void)
{
    uint32_t curr_ticks = 0;
    uint32_t curr_time_ns;

    // Get ticks byte by byte from slave
    curr_ticks = ((uint32_t)GetBytePreciseTimeSpi(r_ticks_byte0)) << 0;
    curr_ticks |= ((uint32_t)GetBytePreciseTimeSpi(r_ticks_byte1)) << 8;
    curr_ticks |= ((uint32_t)GetBytePreciseTimeSpi(r_ticks_byte2)) << 16;
    curr_ticks |= ((uint32_t)GetBytePreciseTimeSpi(r_ticks_byte3)) << 24;

    // Convert from ticks to ns
    curr_time_ns = curr_ticks * NS_PER_TICK;

    return curr_time_ns;
}

static uint8_t GetBytePreciseTimeSpi(SPIState_t state)
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

    DelayMs(1);

    // Get the byte related to the sent state
    SSIDataPut(SSI0_BASE, dummy);
    while(SSIBusy(SSI0_BASE));
    SSIDataGet(SSI0_BASE, &r_data);

    r_byte = (uint8_t)r_data;

    // Nss = 1;
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);

    DelayMs(1);

    return r_byte;
}
