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

/****** DEVICE MODES ******/

// #define DEVICE_MODE_TEST_SPI
// #define DEVICE_MODE_TEST_TRANSMISSION
#define DEVICE_MODE_BASE
// #define DEVICE_MODE_TRACKER

/****** DEFINES FOR TRACKER MODE ******/

#define TX_MESSAGES_PER_SEQUENCE            50

#define TX_SEQUENCE_PERIOD_MS               200
#define TX_SILENCE_PERIOD_MS                45000
#define TX_TIMEOUT_SILENCE_PERIOD_MS        5000

#define NS_PER_TICK                         125

#define TX_RCT_TIMEOUT_VALUE_US             100000000

/****** DEFINES FOR BASE MODE ******/

#define RX_RCT_SEQ_TIMEOUT_VALUE_US         10 * TX_SEQUENCE_PERIOD_MS * 1000
#define RX_RCT_SIL_TIMEOUT_VALUE_US         2 * TX_SILENCE_PERIOD_MS * 1000

#define RX_MAX_RCT_TIMEOUT_COUNT            3

#define RX_TIMESTAMP_BUFFER_SIZE            TX_MESSAGES_PER_SEQUENCE

/****** GENERAL DEFINES FOR DEVICE ******/

#define DEVICE_ID                           2
#define BLINK_PERIOD_MS                     10

// For this specific project, this is the configuration
#if (DEVICE_ID >= 2)
#define LORA_BOARD_LAS_INAIR9B
#else
#define LORA_BOARD_MAS_INAIR9
#endif


/****** MACRO FUNCTIONS ******/

#define DELAY_MS(timeMs)    SysCtlDelay((SysCtlClockGet() / (1000*6)) * timeMs)

/****** DEFINES FOR LORA MIDDLEWARE ******/

/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   !USE_MODEM_LORA

#define RF_FREQUENCY                                    915000000 // Hz
#define TX_OUTPUT_POWER                                 14        // 14 dBm

#if USE_MODEM_LORA == 1

    #define LORA_BANDWIDTH                              2         // [0: 125 kHz,
                                                                  //  1: 250 kHz,
                                                                  //  2: 500 kHz,
                                                                  //  3: Reserved]
    #define LORA_SPREADING_FACTOR                       9         // [SF7..SF12]
    #define LORA_CODINGRATE                             2         // [1: 4/5,
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

#define BUFFER_SIZE                                     32          // Define the payload size here

/******************************************************************
 * DECLARATION OF ENUMS AND STRUCTS
 ******************************************************************/

typedef enum
{
    RECEIVED = 0,
    RECEPTION_TIMEOUT,
    RECEPTION_ERROR,
    TRANSMITTED,
    TRANSMISSION_TIMEOUT,
    IDLE
} States_t;

typedef enum
{
    r_ticks_byte0 = 0,
    r_ticks_byte1,
    r_ticks_byte2,
    r_ticks_byte3
} SPIState_t;

typedef enum
{
    BASE_WAITING_TRACKER_SEQ = 0,
    BASE_RECEIVING_TRACKER_SEQ,

#if (DEVICE_ID == 0 || DEVICE_ID == 2)
    BASE_WAITING_BASE1_SEQ,
    BASE_RECEIVING_BASE1_SEQ,
#endif

#if (DEVICE_ID == 0 || DEVICE_ID == 1)
    BASE_WAITING_BASE2_SEQ,
    BASE_RECEIVING_BASE2_SEQ,
#endif

#if (DEVICE_ID == 1 || DEVICE_ID == 2)
    BASE_TRANSMITTING,
#endif

} BaseState_t;

typedef enum
{
    TIMEOUT = 0,               // Timeout or sequence over
    MESSAGE_RECEIVED,          // Received a message
#if (DEVICE_ID == 1 || DEVICE_ID == 2)
    MESSAGE_TRANSMITTED        // Transmitted a message
#endif
} BaseEvent_t;

typedef struct
{
    uint8_t device_id;
    uint8_t message_id;
    int32_t timestamp;
} TimestampMessage_t;

/******************************************************************
 * DECLARATION OF EXTERNAL VARIABLES
 ******************************************************************/

extern uint8_t lora_board_connected;

/******************************************************************
 * DECLARATION OF EXTERNAL FUNCTIONS
 ******************************************************************/

void OnTxDone(void);

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

void OnTxTimeout(void);

void OnRxTimeout(void);

void OnRxError(void);

#endif /* MAIN_H_ */
