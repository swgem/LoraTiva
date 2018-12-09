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

static volatile States_t State = IDLE;

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
 * DECLARATION OF EXTERNAL VARIABLES
 ******************************************************************/

uint8_t lora_board_connected = LORA_BOARD_CONNECTED;

/******************************************************************
 * DECLARATION OF INTERNAL FUNCTIONS
 ******************************************************************/

static void UIntToString(int number, char * out);

static void ConfigureUART(void);

static void blue_led(uint8_t led);

static void ConfigureSpiPreciseClk(void);

static uint32_t GetCurrentTimeNs(void);

static uint8_t GetBytePreciseTimeSpi(SPIState_t state);

static void TransmitMessage(uint8_t *msg, uint16_t msg_size, uint16_t delay_ms);

/******************************************************************
 * DEFINITION OF EXTERNAL FUNCTIONS
 ******************************************************************/

int main(void)
{
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
#ifdef DEVICE_MODE_TEST_SPI
    UARTprintf("SX1276 TIVA SPI Test application \n\n\r");
#endif

#ifdef DEVICE_MODE_TEST_TRANSMISSION
    UARTprintf("SX1276 TIVA Transmission Test application \n\n\r");
#endif

#ifdef DEVICE_MODE_BASE
    UARTprintf("SX1276 TIVA Base Device application \n\n\r");
#endif

#ifdef DEVICE_MODE_TRACKER
    UARTprintf("SX1276 TIVA Tracker Device application \n\n\r");
#endif

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
    	DELAY_MS(2);
    	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0xFF);
    	UARTprintf( "Radio could not be detected!\n\r" );
        DELAY_MS(100);
    }

    UARTprintf("Radio Version: %d!\n\r", radio_version);

    if (Radio.DetectBoardType( ) == SX1276MB1LAS)
    {
        UARTprintf("\n\r > Board Type: SX1276MB1LAS < \n\r");
    }
    else if (Radio.DetectBoardType( ) == SX1276MB1MAS)
    {
        UARTprintf("\n\r > Board Type: SX1276MB1MAS < \n\r");
    }

    Radio.SetChannel( RF_FREQUENCY );

    ConfigureSpiPreciseClk();

#if USE_MODEM_LORA == 1

    if (LORA_FHSS_ENABLED)
    {
        UARTprintf("\n\n\r             > LORA FHSS Mode < \n\n\r");
    }
    else
    {
        UARTprintf("\n\n\r             > LORA Mode < \n\n\r");
    }

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

    UARTprintf("\n\n\r              > FSK Mode < \n\n\r");
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

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x00);
    DELAY_MS(1);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0xFF);
    DELAY_MS(1);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x00);

#ifdef DEVICE_MODE_TEST_SPI
    uint32_t curr_time_ns;

    while(1)
    {
        curr_time_ns = GetCurrentTimeNs();
    }
#endif

#ifdef DEVICE_MODE_TEST_TRANSMISSION
    char msg_tx[2];
    char msg_id = 0;

    while (1)
    {
        blue_led(1);

        msg_tx[0] = DEVICE_ID;
        msg_tx[1] = msg_id;

        DELAY_MS(10);

        // debug_msg("ID do device: %i        ID da msg: %i\n", msg_tx[0], msg_id[1]);

        Radio.Send((uint8_t *)msg_tx, sizeof(msg_tx));

        blue_led(0);

        DELAY_MS(1500);

        msg_id++;
    }
#endif

#ifdef DEVICE_MODE_BASE
    Radio.Rx(RX_TIMEOUT_VALUE);

    while(1)
    {
        switch(State)
        {
            case RECEIVED:
            {
                curr_time_ns = GetCurrentTimeNs();

                UARTprintf("size: %d, rss: %d, snr: %d, timestamp: %d, device_id: %d, msg_id: %d\n\r",BufferSize,RssiValue,SnrValue,curr_time_ns,Buffer[0],Buffer[1]);
                // UARTprintf("Received: %d, Error: %d, Sum: %d \n\r",TimesReceived, TimesError, (TimesReceived+TimesError));
                
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
                if( BufferSize > 0 )
                {
                    Radio.Rx( RX_TIMEOUT_VALUE );
                }
                State = IDLE;
            }
            break;

            case RECEPTION_TIMEOUT:
            {
                Radio.Rx( RX_TIMEOUT_VALUE );
                State = IDLE;
            }
            break;

            case RECEPTION_ERROR:
            {
                Radio.Rx( RX_TIMEOUT_VALUE );
                State = IDLE;
            }
            break;

            case TRANSMITTED:
            {
                Radio.Rx( RX_TIMEOUT_VALUE );
                State = IDLE;
            }    
            break;

            case TRANSMISSION_TIMEOUT:
            {
                Radio.Rx( RX_TIMEOUT_VALUE );
                State = IDLE;
            }
            break;

            case IDLE:
            default:
            {
                
            }
            break;
        }
    }
#endif

#ifdef DEVICE_MODE_TRACKER
    uint8_t sent_messages_number = 0;
    char msg_tx[2];

    // Set first byte as device id
    msg_tx[0] = DEVICE_ID;

    // Send first message with sequence number
    msg_tx[1] = sent_messages_number;
    TransmitMessage((uint8_t *)msg_tx, sizeof(msg_tx), TX_SEQUENCE_PERIOD_MS);
    sent_messages_number++;

    while( 1 )
    {
        switch( State )
        {
            case RECEIVED:
            {
                UARTprintf("Message received, size: %d, rss: %d, snr: %d\n\r",BufferSize,RssiValue,SnrValue);
                // UARTprintf("Received: %d, Error: %d, Sum: %d \n\r",TimesReceived, TimesError, (TimesReceived+TimesError));

                Radio.Rx( RX_TIMEOUT_VALUE );
                State = IDLE;
            }
            break;

            case RECEPTION_TIMEOUT:
            {
                Radio.Rx( RX_TIMEOUT_VALUE );
                State = IDLE;
            }
            break;

            case RECEPTION_ERROR:
            {
                Radio.Rx( RX_TIMEOUT_VALUE );
                State = IDLE;
            }
            break;

            case TRANSMITTED:
            {
                UARTprintf("Message correctly sent\n");
                // If sequence is over
                if (sent_messages_number == TX_MESSAGES_PER_SEQUENCE)
                {
                    UARTprintf("Transmission sequence over\n\n");

                    // Wait and then restart transmission
                    DELAY_MS(TX_SILENCE_PERIOD_MS);
                    sent_messages_number = 0;

                    UARTprintf("Restarting sequence\n");
                }

                // Send message with sequence number
                msg_tx[1] = sent_messages_number;
                TransmitMessage((uint8_t *)msg_tx, sizeof(msg_tx), TX_SEQUENCE_PERIOD_MS);
                sent_messages_number++;
            }    
            break;

            case TRANSMISSION_TIMEOUT:
            {
                UARTprintf("Message not sent\n");

                DELAY_MS(TX_TIMEOUT_SILENCE_PERIOD_MS);
                
                UARTprintf("Restarting sequence\n");

                // Restart sequence transmission
                sent_messages_number = 0;

                // Send message with sequence number
                msg_tx[1] = sent_messages_number;
                TransmitMessage((uint8_t *)msg_tx, sizeof(msg_tx), TX_SEQUENCE_PERIOD_MS);
                sent_messages_number++;
            }
            break;

            case IDLE:
            default:
            {

            }
            break;
        }
    }
#endif
}

void OnTxDone(void)
{
    Radio.Sleep( );
    State = TRANSMITTED;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    Radio.Sleep( );

    BufferSize = size;
    RssiValue = rssi;
    SnrValue = snr;

    memset(Buffer, 0, BufferSize);
    memcpy(Buffer, payload, BufferSize);

    TimesReceived++;

    State = RECEIVED;
}

void OnTxTimeout(void)
{
    Radio.Sleep();

    State = TRANSMISSION_TIMEOUT;
}

void OnRxTimeout(void)
{
    Radio.Sleep();

    State = RECEPTION_TIMEOUT;
}

void OnRxError(void)
{
    Radio.Sleep( );

    TimesError++;

    State = RECEPTION_ERROR;
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
 * Configure the UART and its pins. This must be called before UARTprintf().
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

    DELAY_MS(100);
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

    DELAY_MS(1);

    // Get the byte related to the sent state
    SSIDataPut(SSI0_BASE, dummy);
    while(SSIBusy(SSI0_BASE));
    SSIDataGet(SSI0_BASE, &r_data);

    r_byte = (uint8_t)r_data;

    // Nss = 1;
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);

    DELAY_MS(1);

    return r_byte;
}

static void TransmitMessage(uint8_t *msg, uint16_t msg_size, uint16_t delay_ms)
{
    // Send message
    Radio.Send((uint8_t *)msg, msg_size);

    // Blink blue led
    blue_led(1);
    DELAY_MS(BLINK_PERIOD_MS);
    blue_led(0);

    if ((int16_t)delay_ms - BLINK_PERIOD_MS > 0)
    {
        DELAY_MS(delay_ms - BLINK_PERIOD_MS);
    }

    return;
}
