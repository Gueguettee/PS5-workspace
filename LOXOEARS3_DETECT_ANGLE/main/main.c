/**
  ****************************************************************************
  * @file    main.c
  * @author  Julien Michel
  * @date    12 November 2022
  * @since   29 October 2022
  * @brief   main file for the LOXears2 project application 
  *          
  ****************************************************************************/
#include <stdio.h>
#include <assert.h>

#include "esp_system.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "definitions.h"
#include "I2S-Driver/I2S.h"
#include "Application/Angle.h"
#include "Application/CanTask.h"

typedef enum {
    eProIdle = tskIDLE_PRIORITY,
    ePrioMDF,
    ePrioBlink
}TASK_Prio_t;

void app_main(void) {
  BaseType_t cTaskCreationResult;
  
  // Create task 
  cTaskCreationResult = xTaskCreate( &Angle,             // TaskFunction_t pvTaskCode,
                                      "MDF_Task",           // const char * const pcName,
                                      4096,                 // uint16_t usStackDepth,
                                      NULL,                 // void* pvParameters,
                                      ePrioMDF,             // UBaseType_t uxPriority,
                                      NULL                  // TaskHandle_t* pxCreatedTask
                                      );

  assert(pdPASS == cTaskCreationResult);
  
  //vTaskDelay(500/portTICK_PERIOD_MS);
  //canSend();
}

