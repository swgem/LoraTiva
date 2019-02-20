#include "LoraDevice.h"
#include <cstdint>
#include <ostream>
#include <string>
#include "../config.h"
#include "Board.h"
#include "LoraStates.h"
#include "LedColors.h"
#include "ReceivedMessage.h"
#include "TimestampMessage.h"
#include "LoraHandlers.h"
#include "../sx1276/sx1276-hal.h"

LoraDevice::LoraDevice(Board* b) : board { b },
                                   lora_state { LoraStates_e::IDLE },
                                   timestamp_msg { (TimestampMessage_s *)(this->received_msg.payload) },
                                   radio_events { on_tx_done, on_tx_timeout, on_rx_done, on_rx_timeout, on_rx_error }
{
    return;
}

LoraDevice::~LoraDevice(void)
{
    return;
}

void LoraDevice::start(void)
{
    // Create radio to be used the whole execution
    SX1276MB1xAS sx1276 { nullptr };
    this->radio = &sx1276;

    this->config_radio();
    this->init();
    this->loop();

    return;
}

void LoraDevice::loop(void)
{
    while(1)
    {
        switch(this->lora_state)
        {
            case LoraStates_e::RECEIVED:
            {
                this->lora_state = LoraStates_e::IDLE;

                this->board->turn_on_led(LedColors_e::YELLOW);
                this->board->delay_ms(BLINK_PERIOD_MS);
                this->board->turn_off_led();

                this->reception_handle();
            }
            break;

            case LoraStates_e::RECEPTION_TIMEOUT:
            {
                this->lora_state = LoraStates_e::IDLE;
                this->reception_timeout_handle();
            }
            break;

            case LoraStates_e::RECEPTION_ERROR:
            {
                this->lora_state = LoraStates_e::IDLE;
                this->reception_error_handle();
            }
            break;

            case LoraStates_e::TRANSMITTED:
            {
                this->lora_state = LoraStates_e::IDLE;

                this->board->turn_on_led(LedColors_e::BLUE);
                this->board->delay_ms(BLINK_PERIOD_MS);
                this->board->turn_off_led();

                this->transmission_handle();
            }    
            break;

            case LoraStates_e::TRANSMISSION_TIMEOUT:
            {
                this->lora_state = LoraStates_e::IDLE;
                this->transmission_timeout_handle();
            }
            break;

            case LoraStates_e::IDLE:
            default:
            {

            }
            break;
        }
    }

    return;
}

void LoraDevice::config_radio(void)
{
    // Initialize Radio driver
    init_lora_handlers((LoraStates_e*)&this->lora_state, this->radio, (ReceivedMessage_s*)&this->received_msg);
    this->radio->Init(&this->radio_events);
    
    // Verify the connection with the board
    int32_t radio_version = this->radio->Read(REG_VERSION);
    while (this->radio->Read(REG_VERSION) == 0x00)
    {
        this->board->UARTcout << "Radio could not be detected!\n\r";
        this->board->delay_ms(100);
    }

    this->board->UARTcout << "Radio Version: " << radio_version << "\n\r";
    this->board->UARTcout << "Device ID: " << DEVICE_ID << "\r\n";

    if (this->radio->DetectBoardType() == SX1276MB1LAS)
    {
        this->board->UARTcout << "\n\r > Board Type: SX1276MB1LAS < \n\r";
    }
    else if (this->radio->DetectBoardType() == SX1276MB1MAS)
    {
        this->board->UARTcout << "\n\r > Board Type: SX1276MB1MAS < \n\r";
    }

    this->radio->SetChannel(RF_FREQUENCY);

#if USE_MODEM_LORA == 1
    if (LORA_FHSS_ENABLED)
    {
        this->board->UARTcout << "\n\n\r             > LORA FHSS Mode < \n\n\r";
    }
    else
    {
        this->board->UARTcout << "\n\n\r             > LORA Mode < \n\n\r";
    }

    this->radio->SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                       LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                       LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                       LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                       LORA_IQ_INVERSION_ON, 2000000);

    this->radio->SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                       LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                       LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
                       LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                       LORA_IQ_INVERSION_ON, true);
#elif USE_MODEM_FSK == 1
    this->board->UARTcout << "\n\n\r              > FSK Mode < \n\n\r";
    this->radio->SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                       FSK_DATARATE, 0,
                       FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                       FSK_CRC_ENABLED, 0, 0, 0, 2000000);

    this->radio->SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                       0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                       0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, FSK_CRC_ENABLED,
                       0, 0, false, true);
#else
#error "Please define a modem in the compiler options."
#endif

    this->board->test_blink_led(INIT_LORA_TEST_PERIOD, INIT_TEST_BLINK_PERIOD);

    return;
}

void LoraDevice::transmit_message(uint8_t *msg, uint16_t msg_size, uint16_t delay_ms)
{
    // Send message
    this->radio->Send((uint8_t *)msg, msg_size);

    // Wait for the specified transmission delay
    if ((int16_t)delay_ms - BLINK_PERIOD_MS > 0)
    {
        this->board->delay_ms(delay_ms - BLINK_PERIOD_MS);
    }

    return;
}

void LoraDevice::print_sent_timestamp_message(TimestampMessage_s* msg)
{
    this->board->UARTcout << "Sent, "
                          << "message_id: " << msg->message_id << ", "
                          << "timestamp: "  << msg->timestamp  << ", "
                          << "rssi: "       << msg->rssi       << ", "
                          << "snr: "        << msg->snr
                          << "\r\n";

    return;
}

void LoraDevice::reception_handle(void)
{
    return;
}

void LoraDevice::reception_timeout_handle(void)
{
    return;
}

void LoraDevice::reception_error_handle(void)
{
    return;
}

void LoraDevice::transmission_handle(void)
{
    return;
}

void LoraDevice::transmission_timeout_handle(void)
{
    return;
}

void LoraDevice::init(void)
{
    return;
}
