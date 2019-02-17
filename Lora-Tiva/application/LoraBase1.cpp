#include "../config.h"

#if (DEVICE_ID == 1)

#include <cstdint>
#include <string>
#include "LoraBase.h"
#include "Board.h"
#include "PreciseClockManager.h"
#include "BaseEvents.h"
#include "BaseStates.h"
#include "TimestampData.h"

LoraBase::LoraBase(Board* b) : LoraDevice { b },
                               base_state { BaseStates_e::BASE_WAITING_TRACKER_SEQ },
                               precise_clock_manager { b },
                               wrong_reception_count { 0 },
                               timestamp_index { 0 }
{
    return;
}

LoraBase::~LoraBase(void)
{
    return;
}

void LoraBase::reception_handle(void)
{
    this->execute_state_machine(BaseEvents_e::MESSAGE_RECEIVED);

    return;
}

void LoraBase::reception_timeout_handle(void)
{
    this->execute_state_machine(BaseEvents_e::TIMEOUT);

    return;
}

void LoraBase::reception_error_handle(void)
{
    this->radio->Rx(TX_RCT_TIMEOUT_VALUE_US);

    return;
}

void LoraBase::transmission_handle(void)
{
    this->execute_state_machine(BaseEvents_e::MESSAGE_TRANSMITTED);

    return;
}

void LoraBase::transmission_timeout_handle(void)
{
    this->radio->Rx(TX_RCT_TIMEOUT_VALUE_US);

    return;
}

void LoraBase::init(void)
{
    this->precise_clock_manager.init();

    // Clear timestamp buffer
    this->set_word_buffer((int32_t*)this->timestamp_buffer,
                          -1,
                          sizeof(this->timestamp_buffer) / sizeof(this->timestamp_buffer[0]));

    // Enter reception mode
    radio->Rx(RX_RCT_SIL_TRACKER_TIMEOUT_US);

    return;
}

void LoraBase::set_word_buffer(int32_t *buffer, int32_t value, uint32_t size)
{
    do
    {
        buffer[--size] = value;
    } while (size);

    return;
}

void LoraBase::execute_state_machine(BaseEvents_e event)
{
    uint32_t curr_time_ns = this->precise_clock_manager.get_current_time_ns();

    switch (this->base_state)
    {
        case BaseStates_e::BASE_WAITING_TRACKER_SEQ:
            if (event == BaseEvents_e::MESSAGE_RECEIVED)
            {
                if (this->timestamp_msg->device_id >= 3)
                {
                    this->set_word_buffer((int32_t*)this->timestamp_buffer,
                                          -1,
                                          sizeof(this->timestamp_buffer) / sizeof(this->timestamp_buffer[0]));

                    this->board->UARTcout << "Receiving tracker sequence\r\n";

                    this->base_state = BaseStates_e::BASE_RECEIVING_TRACKER_SEQ;
                }
                else
                {
                    this->radio->Rx(RX_RCT_SIL_TRACKER_TIMEOUT_US);
                    break;
                }
            }
            else if (event == BaseEvents_e::TIMEOUT)
            {
                this->board->UARTcout << "Timeout waiting tracker\r\n\r\n";
                this->radio->Rx(RX_RCT_SIL_TRACKER_TIMEOUT_US);
                break;
            }
        case BaseStates_e::BASE_RECEIVING_TRACKER_SEQ:
        {
            if (event == BaseEvents_e::MESSAGE_RECEIVED)
            {
                if (this->timestamp_msg->device_id >= 3)
                {
                    this->timestamp_buffer[this->timestamp_msg->message_id].timestamp = curr_time_ns;
                    this->timestamp_buffer[this->timestamp_msg->message_id].rssi = this->received_msg.rssi;
                    this->timestamp_buffer[this->timestamp_msg->message_id].snr = this->received_msg.snr;

                    this->print_rcvd_timestamp_message(curr_time_ns);

                    if (this->timestamp_msg->message_id == RX_LAST_MESSAGE_ID)
                    {
                        this->board->UARTcout << "Tracker sequence reception over\r\n\r\n";
                        this->board->UARTcout << "Start transmission\r\n";

                        this->board->delay_ms(10 * TX_SEQUENCE_PERIOD_MS);

                        // Start transmitting
                        this->timestamp_index = 0;
                        TimestampMessage_s msg_tx { DEVICE_ID,
                                                    this->timestamp_index,
                                                    this->timestamp_buffer[this->timestamp_index].timestamp,
                                                    (int16_t)this->timestamp_buffer[this->timestamp_index].rssi,
                                                    (int8_t)this->timestamp_buffer[this->timestamp_index].snr };
                        this->transmit_message((uint8_t *)&msg_tx, sizeof(msg_tx), TX_SEQUENCE_PERIOD_MS);
                        this->print_sent_timestamp_message(&msg_tx);
                        this->timestamp_index++;

                        this->base_state = BaseStates_e::BASE_TRANSMITTING;
                    }
                    else
                    {
                        this->radio->Rx(RX_RCT_SEQ_TIMEOUT_US);
                    }
                }
                else
                {
                    this->board->UARTcout << "Wrong reception!! ";
                    this->print_rcvd_timestamp_message(curr_time_ns);
                    this->radio->Rx(RX_RCT_SEQ_TIMEOUT_US);
                }
            }
            else if (event == BaseEvents_e::TIMEOUT)
            {
                this->board->UARTcout << "Tracker sequence reception over\r\n\r\n";

                this->board->UARTcout << "Starting transmission\r\n";

                this->board->delay_ms(10 * TX_SEQUENCE_PERIOD_MS);

                // Start transmitting
                this->timestamp_index = 0;
                TimestampMessage_s msg_tx { DEVICE_ID,
                                            this->timestamp_index,
                                            this->timestamp_buffer[this->timestamp_index].timestamp,
                                            (int16_t)this->timestamp_buffer[this->timestamp_index].rssi,
                                            (int8_t)this->timestamp_buffer[this->timestamp_index].snr };
                this->transmit_message((uint8_t *)&msg_tx, sizeof(msg_tx), TX_SEQUENCE_PERIOD_MS);
                this->print_sent_timestamp_message(&msg_tx);
                this->timestamp_index++;

                this->base_state = BaseStates_e::BASE_TRANSMITTING;
            }
        }
        break;

        case BaseStates_e::BASE_TRANSMITTING:
        {
            if (event == BaseEvents_e::MESSAGE_TRANSMITTED)
            {
                if (this->timestamp_index < RX_TIMESTAMP_BUFFER_SIZE)
                {
                    TimestampMessage_s msg_tx { DEVICE_ID,
                                                this->timestamp_index,
                                                this->timestamp_buffer[this->timestamp_index].timestamp,
                                                (int16_t)this->timestamp_buffer[this->timestamp_index].rssi,
                                                (int8_t)this->timestamp_buffer[this->timestamp_index].snr };
                    this->transmit_message((uint8_t *)&msg_tx, sizeof(msg_tx), TX_SEQUENCE_PERIOD_MS);
                    this->print_sent_timestamp_message(&msg_tx);
                    this->timestamp_index++;
                }
                else
                {
                    this->radio->Rx(RX_RCT_SIL_BASE_TIMEOUT_US);

                    this->board->UARTcout << "Transmission over\r\n\r\n";

                    this->base_state = BaseStates_e::BASE_WAITING_BASE2_SEQ;
                }
            }
        }
        break;

        case BaseStates_e::BASE_WAITING_BASE2_SEQ:
        {
            if (event == BaseEvents_e::MESSAGE_RECEIVED)
            {
                if (this->timestamp_msg->device_id == 2)
                {
                    this->board->UARTcout << "Receiving base 2 sequence\r\n";

                    this->print_rcvd_timestamp_message(curr_time_ns);

                    this->base_state = BaseStates_e::BASE_RECEIVING_BASE2_SEQ;

                    this->radio->Rx(RX_RCT_SEQ_TIMEOUT_US);
                }
                else
                {
                    this->wrong_reception_count++;

                    if (this->wrong_reception_count == RX_MAX_RCT_WRONG_RECEPTION_COUNT)
                    {
                        this->board->UARTcout << "No message from base 2. Restarting cycle\r\n";

                        this->base_state = BaseStates_e::BASE_WAITING_TRACKER_SEQ;

                        this->wrong_reception_count = 0;

                        this->radio->Rx(RX_RCT_SIL_TRACKER_TIMEOUT_US);
                    }
                    else
                    {
                        this->radio->Rx(RX_RCT_SIL_BASE_TIMEOUT_US);
                    }
                }
            }
            else if (event == BaseEvents_e::TIMEOUT)
            {
                this->board->UARTcout << "No message from base 2. Restarting cycle\r\n";

                this->radio->Rx(RX_RCT_SIL_TRACKER_TIMEOUT_US);

                this->base_state = BaseStates_e::BASE_WAITING_TRACKER_SEQ;
            }
        }
        break;

        case BaseStates_e::BASE_RECEIVING_BASE2_SEQ:
        {
            if (event == BaseEvents_e::MESSAGE_RECEIVED)
            {
                if (this->timestamp_msg->device_id == 2)
                {
                    this->print_rcvd_timestamp_message(curr_time_ns);

                    if (this->timestamp_msg->message_id == RX_LAST_MESSAGE_ID)
                    {
                        this->board->UARTcout << "Base 2 sequence reception over. Restarting cycle\r\n\r\n";
                        this->radio->Rx(RX_RCT_SIL_TRACKER_TIMEOUT_US);
                        this->base_state = BaseStates_e::BASE_WAITING_TRACKER_SEQ;
                    }
                    else
                    {
                        this->radio->Rx(RX_RCT_SEQ_TIMEOUT_US);
                    }
                }
                else
                {
                    this->radio->Rx(RX_RCT_SEQ_TIMEOUT_US);
                }
            }
            else if (event == BaseEvents_e::TIMEOUT)
            {
                this->board->UARTcout << "Base 2 sequence reception over. Restarting cycle\r\n\r\n";

                this->radio->Rx(RX_RCT_SIL_TRACKER_TIMEOUT_US);

                this->base_state = BaseStates_e::BASE_WAITING_TRACKER_SEQ;
            }
        }
        break;

        default:
        {
            this->board->UARTcout << "FATAL ERROR";
        }
        break;
    }
}

void LoraBase::print_rcvd_timestamp_message(uint32_t timestamp)
{
    this->board->UARTcout << "Received, "
                          << "rssi: "       << this->received_msg.rssi         << ", "
                          << "snr: "        << this->received_msg.snr          << ", "
                          << "timestamp: "  << timestamp                       << "; "
                          << "Payload, "
                          << "device_id: "  << this->timestamp_msg->device_id  << ", "
                          << "message_id: " << this->timestamp_msg->message_id << ", "
                          << "timestamp: "  << this->timestamp_msg->timestamp  << ", "
                          << "snr: "        << this->timestamp_msg->rssi       << ", "
                          << "rssi: "       << this->timestamp_msg->snr
                          << "\r\n";

    return;
}

#endif
