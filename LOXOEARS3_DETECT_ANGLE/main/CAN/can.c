/**
 * @file can.c
 * @author Augustin GUILLAUME (august1@guillaumefamily.ch)
 * @brief C file for the CAN bus library of the DBCS. Contains all required functions, requires ESP-IDF and can.h.
 * @version v1.4
 * @date 04-07-2022
 * @since 02-05-2022
 */

/*  FILE NAME:              can.c
*   CREATION DATE:          02.05.2022
*   LAST MODIFICATION:      05.07.2022
*   CREATOR:                Augustin GUILLAUME
*   COMPANY:                LOXO AG, HEAI-FR, ROSAS
*   DESCRIPTION:            C file for the CAN bus library of the DBCS. Contains all required functions, requires ESP-IDF and can.h.
*/

#include "driver/twai.h"
#include "can.h"
#include "esp_log.h"

#define DATA_BYTE_NBR 2    //With actual protocol, positions sent on 1 byte
#define WAIT_TIME_MS 1   //For sending and receiving messages

const typedef enum{
    eSirenDetector,
    eNbr_IDs,
}DBCS_ID_t;

const typedef enum{
    ID,
    Acceptance_mask,
    Acceptance_code,
    eNbr_Properties,
}CAN_Properties_t;

 const uint32_t ID_array [eNbr_IDs][eNbr_Properties] =
{
    //ID, mask      , code
    {0x8, 0x1FFFFF00, 0x10000800},
};

//Union containing the ID and the message. This is not portable if a change of uC happens
typedef union{
    struct {
        CAN_Message_t message : 8;
        uint8_t ID : 4;
        uint32_t zeros : 17;
    };
    uint32_t CAN_ID_MESSAGE;
}CAN_Identifier_t;

static CAN_Identifier_t identifierTX;

int can_init(uint8_t DBCS_ID)
{
    //Set the CAN ID according to the switches
    identifierTX.zeros = 0x10000;
    identifierTX.ID = ID_array[DBCS_ID][ID];

    //Initialize configuration structures using macro initializers
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();//TWAI_TIMING_CONFIG_500KBITS();
    //Filter all other IDs except MSG_ID
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_CAN_TX, GPIO_CAN_RX, TWAI_MODE_NORMAL);

    //Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        //printf("Driver installed\n");
    } else {
        //printf("Failed to install driver\n");
        return CAN_CRITICAL_ERROR;
    }

    #define ALERTS_TO_ENABLE (TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_FAILED | TWAI_ALERT_TX_IDLE | TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED)
    //Reconfigure alerts to detect Error Passive and Bus-Off error states
    uint32_t alerts_to_enable = ALERTS_TO_ENABLE;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
        //printf("Alerts reconfigured\n");
    } else {
        //printf("Failed to reconfigure alerts");
        return CAN_CRITICAL_ERROR;
    }

    //Start TWAI driver
    if (twai_start() == ESP_OK) {
        //printf("Driver started\n");
    } else {
        //printf("Failed to start driver\n");
        return CAN_CRITICAL_ERROR;
    }

    return CAN_OK;
}

int can_receive_data(CAN_Message_t* messageRX, Positions_array_t* positions)
{
    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(WAIT_TIME_MS)) == ESP_OK) {
        //Process received message
        CAN_Identifier_t identifierRX;
        identifierRX.CAN_ID_MESSAGE = message.identifier;
        *messageRX = identifierRX.message;
        if (message.extd) {
            //printf("Message is in Extended Format\n");
        } else {
            return CAN_RECEIVE_ERROR;
        }
        //printf("ID is %x\n", message.identifier);
        if (message.data_length_code != 0) {
            for (int i = 0; i < message.data_length_code; i++) {
                //printf("Data byte %d = %d\n", i, message.data[i]);
            }
            if(message.data_length_code == DATA_BYTE_NBR)
            {
                (*positions)[message.data[0]] = message.data[2] + (message.data[1] << 8);
            } else {
                //printf("Data received has other than 2 bytes...\n");
            }
        } else {
            //printf("Message without data received\n");
        }
        return CAN_MESSAGE_RX;
    } else {
        //printf("Failed to receive message\n");
        return CAN_RECEIVE_ERROR;
    }
}

int can_send(twai_message_t messageTX)
{
    //Queue message for transmission
    if (twai_transmit(&messageTX, pdMS_TO_TICKS(WAIT_TIME_MS)) == ESP_OK) {
        //printf("Message queued for transmission\n");            
    } else {
        //printf("Failed to queue message for transmission\n");
        return CAN_QUEUING_ERROR;
    }
    return CAN_OK;
}

int can_transmit_no_data(CAN_Message_t canMessageTX)
{
    identifierTX.message = canMessageTX;

    //Configure message to transmit
    twai_message_t messageTX;
    messageTX.identifier = identifierTX.CAN_ID_MESSAGE;
    //messageTX.self = 1;
    messageTX.extd = 1;
    messageTX.data_length_code = 0;

    uint16_t status_error = can_send(messageTX);
    return status_error;
}

//Can transmit messages up to two bytes
int can_transmit_data(CAN_Message_t canMessageTX, uint16_t data, uint8_t bytes_nbr)
{
    const uint8_t mask = 0xFF;

    identifierTX.message = canMessageTX;

    //Configure message to transmit
    twai_message_t messageTX = { 0 };
    messageTX.identifier = identifierTX.CAN_ID_MESSAGE;
    //messageTX.self = 1;
    messageTX.extd = 1;
    messageTX.data_length_code = bytes_nbr;

    switch(bytes_nbr)
    {
        case 1:
            messageTX.data[0] = data & mask;
            break;
        case 2:
            messageTX.data[1] = data & mask;
            messageTX.data[0] = (data >> 8) & mask;
            break;
        default:
            return CAN_QUEUING_ERROR;
            break;
    }

    uint16_t status_error = can_send(messageTX);
    return status_error;
}

int can_handle_alerts(uint16_t status_errors, CAN_Message_t* messageRX)
{
    static const char *TAG = "CanHandleAlerts";
    ESP_LOGI(TAG,"enter in function");
    twai_status_info_t infos;
    if(twai_get_status_info(&infos) != ESP_OK)
    {
        status_errors = CAN_CRITICAL_ERROR;
    }

    uint32_t alerts_triggered;
    //Block indefinitely until an alert occurs
    if (twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(WAIT_TIME_MS)) == ESP_OK)
    {
        if((alerts_triggered & TWAI_ALERT_TX_SUCCESS) != 0)
        {
            //printf("Message sent succesfully !\n");
        }
        if((alerts_triggered & TWAI_ALERT_TX_IDLE) != 0)
        {
            //printf("Queue emptied\n");
        }
        if((alerts_triggered & TWAI_ALERT_TX_FAILED) != 0)
        {
            //printf("Error while attempting to send message...\n");
            status_errors |= CAN_SEND_ERROR;
        }
        /* if((alerts_triggered & TWAI_ALERT_RX_DATA) != 0)
        {
            //printf("Message received, proceeding to treat message\n");
            
            return CAN_MESSAGE_RX;
        } */
        if((alerts_triggered & TWAI_ALERT_BUS_OFF) != 0)
        {
            status_errors |= CAN_BUS_OFF;
        }
        if((alerts_triggered & TWAI_ALERT_BUS_RECOVERED) != 0)
        {
            status_errors |= CAN_RECOVERED;
        }
        if((alerts_triggered & ALERTS_TO_ENABLE) == 0)
        {
            //printf("Unknown error code !\n Error:\t %d\n", alerts_triggered);
            status_errors |= CAN_CRITICAL_ERROR;
        }
    }
    /*if(infos.msgs_to_rx > 0)
    {
        status_errors |= can_receive_data(messageRX, positions);
        if((status_errors & CAN_RECEIVE_ERROR) == 0)
        {
            return CAN_MESSAGE_RX;
        }
    }*/
    if((status_errors & CAN_CRITICAL_ERROR) != 0)
    {
        can_transmit_data(DBCS_REBOOT, status_errors, 2);
        esp_restart();
    }
    if((status_errors & CAN_BUS_OFF) != 0)
    {
        //Proceed to recover the bus
        twai_initiate_recovery();
        return CAN_RECOVERING;
    }
    if((status_errors & CAN_RECOVERED) != 0)
    {
        twai_start();
        can_transmit_no_data(DBCS_HELLO);
    }
    return CAN_OK;
}

uint16_t can_send_status(uint8_t box, uint8_t status, uint16_t position_left, uint16_t position_right)
{
    const uint8_t mask = 0xFF;

    identifierTX.message = STATUS_REPLY;

    //Configure message to transmit
    twai_message_t messageTX;
    messageTX.identifier = identifierTX.CAN_ID_MESSAGE;
    //messageTX.self = 1;
    messageTX.extd = 1;
    messageTX.data_length_code = 2 * DATA_BYTE_NBR;
    messageTX.data[0] = box & mask;
    messageTX.data[1] = status & mask;

    messageTX.data[3] = position_left & mask;
    messageTX.data[2] = (position_left >> 8) & mask;

    messageTX.data[5] = position_right & mask;
    messageTX.data[4] = (position_right >> 8) & mask;

    uint16_t status_error = can_send(messageTX);
    return status_error;
}

//######################################## END OF FILE ################################################