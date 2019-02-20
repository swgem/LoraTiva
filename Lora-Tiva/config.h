#ifndef __CONFIG_H__
#define __CONFIG_H__

/****************************************/
/****** APPLICATION CONFIGURATIONS ******/
/****************************************/

// DEVICE ID AND CORRESPONDING OPERATION MODE

#define DEVICE_ID                           0

// TIMING CONFIGURATIONS

#define BLINK_PERIOD_MS                     10

#define TX_MESSAGES_PER_SEQUENCE            50
#define TX_SEQUENCE_PERIOD_MS               200
#define TX_SILENCE_PERIOD_MS                45000
#define TX_TIMEOUT_SILENCE_PERIOD_MS        5000
#define TX_RCT_TIMEOUT_VALUE_US             100000000
#define RX_RCT_SEQ_TIMEOUT_US               10 * TX_SEQUENCE_PERIOD_MS * 1000
#define RX_RCT_SIL_TRACKER_TIMEOUT_US       2 * TX_SILENCE_PERIOD_MS * 1000
#define RX_RCT_SIL_BASE_TIMEOUT_US          (TX_SILENCE_PERIOD_MS / 3) * 1000

// OTHER CONFIGURATIONS

#define NS_PER_TICK                         125
#define RX_MAX_RCT_WRONG_RECEPTION_COUNT    3
#define RX_PAYLOAD_SIZE                     32

// TIMING CONFIGURATIONS FOR BLINK TEST DURING INITILIZATION

#define INIT_BOARD_TEST_PERIOD              5000
#define INIT_LORA_TEST_PERIOD               5000
#define INIT_TEST_BLINK_PERIOD              1000

/****************************************/
/******** APPLICATION DEFINITIONS *******/
/****************************************/

// DEVICE MODE AND LORA BOARD TYPE

#if (DEVICE_ID >= 3)
#define DEVICE_MODE_TRACKER
#else
#define DEVICE_MODE_BASE
#endif

#if (DEVICE_ID >= 2)
#define LORA_BOARD_LAS_INAIR9B
#else
#define LORA_BOARD_MAS_INAIR9
#endif

// RECEPTION DEFINITIONS

#define RX_LAST_MESSAGE_ID                  TX_MESSAGES_PER_SEQUENCE - 1
#define RX_TIMESTAMP_BUFFER_SIZE            TX_MESSAGES_PER_SEQUENCE

/****************************************/
/******* MIDDLEWARE CONFIGURATIONS ******/
/****************************************/

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

#endif // __CONFIG_H__
