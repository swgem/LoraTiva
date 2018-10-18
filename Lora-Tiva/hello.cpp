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

#include "lora.h"
#include "sx1276/sx1276-hal.h"

//início das definições do LoRa*****************




/* Set this flag to '1' to display debug messages on the console */
#define DEBUG_MESSAGE   1

#define debug_msg(X)  UARTprintf(X)
#define debug_msg_if(D, X)  if(D){UARTprintf(X);}

/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   !USE_MODEM_LORA

#define RF_FREQUENCY                                    915000000 // Hz
#define TX_OUTPUT_POWER                                 20        // 14 dBm

// #define TESTE_TRANSMITE

#if USE_MODEM_LORA == 1

    #define LORA_BANDWIDTH                              2         // [0: 125 kHz,
                                                                  //  1: 250 kHz,
                                                                  //  2: 500 kHz,
                                                                  //  3: Reserved]
    #define LORA_SPREADING_FACTOR                       12        // [SF7..SF12]
    #define LORA_CODINGRATE                             4         // [1: 4/5,
                                                                  //  2: 4/6,
                                                                  //  3: 4/7,
                                                                  //  4: 4/8]
    #define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
    #define LORA_SYMBOL_TIMEOUT                         5         // Symbols
    #define LORA_FIX_LENGTH_PAYLOAD_ON                  false
    #define LORA_FHSS_ENABLED                           false
    #define LORA_NB_SYMB_HOP                            4
    #define LORA_IQ_INVERSION_ON                        false
    #define LORA_CRC_ENABLED                            true

#elif USE_MODEM_FSK == 1

    #define FSK_FDEV                                    25000     // Hz
    #define FSK_DATARATE                                19200     // bps
    #define FSK_BANDWIDTH                               50000     // Hz
    #define FSK_AFC_BANDWIDTH                           83333     // Hz
    #define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
    #define FSK_FIX_LENGTH_PAYLOAD_ON                   false
    #define FSK_CRC_ENABLED                             true

#else
    #error "Please define a modem in the compiler options."
#endif

#define RX_TIMEOUT_VALUE                                100000000   // in us
#define BUFFER_SIZE                                     32          // Define the payload size here

/*
 *  Global variables declarations
 */
typedef enum
{
    LOWPOWER = 0,
    IDLE,

    RX,
    RX_TIMEOUT,
    RX_ERROR,

    TX,
    TX_TIMEOUT,

    CAD,
    CAD_DONE
} AppStates_t;

volatile AppStates_t State = LOWPOWER;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*
 *  Global variables declarations
 */
SX1276MB1xAS Radio(NULL);

// const uint8_t SendMsg1[] = "                               ";
// const uint8_t RecvMsg1[] = "                               ";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

int16_t RssiValue = 0.0;
int8_t SnrValue = 0.0;

static void UIntToString(int number, char * out);


//fim das definições do LoRa********************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************

void ConfigureUART(void)
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

void blue_led(uint8_t led)
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
    int i;
    int id = 0;

    while (1)
    {
        blue_led(1);

        UIntToString(id, msg_id);

        debug_msg(msg_id);

        strcpy((char*)Buffer, (char*)msg_id);

        // We fill the buffer with numbers for the payload

        for(i = sizeof(msg_id); i < BufferSize; i++){
            Buffer[i] = i - sizeof(msg_id);
        }
        SysCtlDelay(SysCtlClockGet() / 100);

        Radio.Send(Buffer, BufferSize);

        blue_led(0);

        SysCtlDelay(SysCtlClockGet() / 2);

        id++;
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

void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
    debug_msg_if( DEBUG_MESSAGE, "> OnTxDone\n\r" );
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
    debug_msg_if( DEBUG_MESSAGE, "> OnRxDone\n\r" );
    UARTprintf("size: %d, rss: %d, snr: %d, payload: %s",size,rssi,snr,payload);

}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
    debug_msg_if( DEBUG_MESSAGE, "> OnTxTimeout\n\r" );
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    Buffer[ BufferSize-1 ] = 0;
    State = RX_TIMEOUT;
    debug_msg_if( DEBUG_MESSAGE, "> OnRxTimeout\n\r" );
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
    debug_msg_if( DEBUG_MESSAGE, "> OnRxError\n\r" );
}

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
