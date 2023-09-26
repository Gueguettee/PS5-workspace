/**
 * @file can.h
 * @author Augustin GUILLAUME (august1@guillaumefamily.ch)
 * @brief Header file containing the CAN library API. requires can.c and ESP-IDF to function.
 * @version v1.4
 * @date 05-07-2022
 * @since 02-05-2022
 */

/*  FILE NAME:              can.h
*   CREATION DATE:          02.05.2022
*   LAST MODIFICATION:      28.06.2022
*   CREATOR:                Augustin GUILLAUME
*   COMPANY:                LOXO AG, HEAI-FR, ROSAS
*   DESCRIPTION:            Header file containing the CAN library API. requires can.c and ESP-IDF to function.
*/

#ifndef CAN_H
#define CAN_H

#include "types.h"

#define GPIO_CAN_TX 9 //Will be changed to 9 after HW modification
#define GPIO_CAN_RX 8

#define CAN_OK              0x0000  //Everything set/sent/received as intended
#define CAN_CRITICAL_ERROR  0x0001  //Error used when twai driver not starting, alerts not configurable or anything else
                                    //requiring a reboot of the microcontroller
#define CAN_RECEIVE_ERROR   0x0002  //Failed to receive a message for unknown reason
#define CAN_QUEUING_ERROR   0x0004  //Failed to queue message
#define CAN_SEND_ERROR      0x0008  //Failed to send message
#define CAN_MESSAGE_RX      0x0010  //Received a new message
#define CAN_BUS_OFF         0x0020  //Bus off due to too many errors
#define CAN_RECOVERING      0x0040  //Bus recovering, going to recovering to stopped
#define CAN_RECOVERED       0x0080  //Bus recovered, needs 128 time 11 bits recessive consecutives

//CAN_Message_t contains all messages from the current protocol. Messages can be added/removed according
//to protocol changes
/**
 * @brief Enum for the different CAN messages according to the communication protocol
 *        between the DBCS and the Processing Unit 
 */
typedef enum{
    PING_REQUEST = 0x00,    //Verifies the connexion between a box and the Processing Unit
    PING_RESPONSE = 0x01,   //Response to a PING request
    INIT = 0x02,            //Initializes the DBCS and starts the homing of the boxes
    MOVE_CURTAIN = 0x10,
    STATUS_REQUEST = 0x20,     //Asks for the DBCS status, box connected or not, curtains positions
    STATUS_REPLY = 0x21,    //Reply with the DBCS status
    DBCS_HELLO = 0x31,      //Sends this message upon start of the program
    DBCS_REBOOT = 0x30,     //Sends this message before rebooting
    BOX_BLOCAGE = 0x40,     //Informs the Processing Unit on a box blocage, Right or Left
    DBCS_ERROR = 0x41,      //Informs the Processing Unit on an error with the corresponding error flag
    POS_REACHED = 0x50      //Informs the Processing Unit what box reached its target positions
}CAN_Message_t;

typedef struct{
    CAN_Message_t messageTX;
    uint8_t box;
    uint16_t flags;
}CAN_TX_types_t;

//API functions
/**
 * @brief can_init initialize the CAN bus with the given ID
 * 
 * @param DBCS_ID 
 * @return int Returns error flags
 */
int can_init(uint8_t DBCS_ID);

/**
 * @brief can_transmit_data sends the desired message with up to 2 bytes of data
 * 
 * @param canMessageTX 
 * @param data 
 * @param bytes_nbr 
 * @return int Returns error flags
 */
int can_transmit_data(CAN_Message_t canMessageTX, uint16_t data, uint8_t bytes_nbr);

/**
 * @brief Transmits CAN messages without datas
 * 
 * @param canMessageTX 
 * @return int Returns error flags
 */
int can_transmit_no_data(CAN_Message_t canMessageTX); //Transmit status messages

/**
 * @brief Handles the errors accordingly to the received flags and treats the messages
 * 
 * @param status_errors Error flags to treat
 * @param messageRX Pointer to the received message, to analyze out of the function
 * @param positions Position array for the received positions
 * @return int Returns error flags
 */
int can_handle_alerts(uint16_t status_errors, CAN_Message_t* messageRX); //Positions_array_t* positions);

/**
 * @brief Sends the status of eache box, wether they are connected or not, and their curtains positions
 *        if the box is connected.
 * 
 * @param box LEFT_BOX or RIGHT_BOX, from types.h
 * @param status CONNECTED or UNCONNECTED, from types.h
 * @param position_left Actual left position, given in mm
 * @param position_right Actual right position, given in mm
 * @return uint16_t Returns error flags
 */
uint16_t can_send_status(uint8_t box, uint8_t status, uint16_t position_left, uint16_t position_right);

/**
 * @brief Sends a message the box reached its targeted positions for both of its curtains
 * 
 * @param box LEFT_BOX or RIGHT_BOX, from types.h
 * @return uint16_t Returns error flags
 */
uint16_t can_pos_reached(uint8_t box);

#endif

//######################################## END OF FILE ################################################