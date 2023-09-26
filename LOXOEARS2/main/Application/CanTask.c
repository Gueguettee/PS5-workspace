/**
  ****************************************************************************
  * @file    CanTask.h
  * @author  Julien Michel
  * @date    6 December 2022
  * @since   6 December 2022
  * @brief   Header files for the CAN-communication between the esp32 and the 
  *          central computer for the LOXears2 project
  ****************************************************************************/
#include <stdio.h>
#include <stdbool.h>

#include "esp_log.h"

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "definitions.h"

#include "../CAN/can.h"
#include "CanTask.h"


void canSend(){
    static const char *TAG = "CanTask"; //For the LOGs
    int error;
    uint16_t can_flags = CAN_OK;
    CAN_Message_t messageRX;

    error = can_init(0);
    ESP_LOGI(TAG,"Init error: %d",error);
    CAN_Message_t message = 0x00;
    while(1){
        
        can_flags = can_handle_alerts(can_flags, &messageRX);

        can_flags = CAN_OK;
        can_flags |= can_transmit_no_data(message);
        //error = can_transmit_data(0x00,0xC,1);
        ESP_LOGI(TAG,"error: %d",error);
        
        /*
        can_flags |= can_transmit_no_data(message + 1);
        
        ESP_LOGI(TAG,"error: %d",error);
        can_flags |= can_transmit_no_data(message + 2);
        ESP_LOGI(TAG,"error: %d",error);
        can_flags |= can_transmit_no_data(message + 3);
        ESP_LOGI(TAG,"error: %d",error);
        */
        can_flags |= can_transmit_data(message, 0xFC, 2);
        ESP_LOGI(TAG,"error: %d",error);

        message = 0x00;
        vTaskDelay(500/portTICK_PERIOD_MS);
        //message = message < 0xFF ? message + 1 : 0x00;
    }
}