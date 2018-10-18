/*
 * main.h
 *
 *  Created on: 17 de out de 2018
 *      Author: User
 */

#ifndef MAIN_H_
#define MAIN_H_

/******************************************************************
 * DECLARATION OF MACROS
 ******************************************************************/

// #define TESTE_TRANSMITE

// LORA Macros

/* Set this flag to '1' to display debug messages on the console */
#define DEBUG_MESSAGE   1

#define debug_msg(X)  UARTprintf(X)
#define debug_msg_if(D, X)  if(D){UARTprintf(X);}

/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   !USE_MODEM_LORA

#define RF_FREQUENCY                                    915000000 // Hz
#define TX_OUTPUT_POWER                                 20        // 14 dBm

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

/******************************************************************
 * DECLARATION OF ENUMS AND STRUCTS
 ******************************************************************/

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

/******************************************************************
 * DECLARATION OF EXTERNAL FUNCTIONS
 ******************************************************************/

void OnTxDone(void);

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

void OnTxTimeout(void);

void OnRxTimeout(void);

void OnRxError(void);


#endif /* MAIN_H_ */
