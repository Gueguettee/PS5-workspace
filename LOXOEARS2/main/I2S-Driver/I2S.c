/**
  ****************************************************************************
  * @file    I2S.c
  * @author  Julien Michel
  * @date    16 November 2022
  * @since   29 October 2022
  * @brief   I2S driver to read the microphone module IMD69D130 Shield2Go for 
  *          the LOXears2 project
  ****************************************************************************/
#include <stdio.h>

#include "I2S.h"
#include "driver/i2s.h"
#include "esp_log.h"

#include "definitions.h"

#define READ_BUFF_SIZE (100) //Size of the intermediate buffer (n sample per reading in DMA buffers)
#define READ_BUFF_SIZE2 (100) //Size of the intermediate buffer (n sample per reading in DMA buffers)


#if defined(KALUGA_BOARD)

    #define I2S_BCK_GPIO (19)
    #define I2S_WS_GPIO (20)
    #define I2S_DATA_GPIO (21)

#elif defined(DEVKITC_BOARD)

    #define I2S_BCK_GPIO (0)
    #define I2S_WS_GPIO (1)
    #define I2S_DATA_GPIO (2)

#endif 

void I2S_init(void) {
    // Set the I2S configuration 
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = SAMPLERATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, 
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,
        .dma_buf_count = 8, 
        .dma_buf_len = 400, 
        .use_apll = 0,
    };

    // Set the pinout configuration 
    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_PIN_NO_CHANGE,
        .bck_io_num = I2S_BCK_GPIO,
        .ws_io_num = I2S_WS_GPIO,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DATA_GPIO,
    };
    

    // Call driver installation functions
    ESP_ERROR_CHECK( i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL) );
    ESP_ERROR_CHECK( i2s_set_pin(I2S_NUM_0, &pin_config) );
    ESP_ERROR_CHECK( i2s_set_clk(I2S_NUM_0, SAMPLERATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO) );
}

void I2S_RecordTask(uint16_t arrayLength, int16_t recordArray[2][arrayLength]) {
    static const char *TAG = "I2S_RecordTask"; //For the LOGs
    
    static int32_t readBuff[READ_BUFF_SIZE] = {0}; //Intermediate buffer
    const uint16_t cByteToRead = 4*READ_BUFF_SIZE;  //number of bytes to read, when reading DMA buffers
    static size_t bytes_read = 0;  //to store the number of bytes that have been read, when reading DMA buffers

    for(uint32_t i = 0; i<arrayLength; i+=(READ_BUFF_SIZE/2)) {
        if(i2s_read(I2S_NUM_0, &readBuff, cByteToRead, &bytes_read, portMAX_DELAY) == ESP_OK){
            for(uint16_t j = 0; j<READ_BUFF_SIZE; j+=2){
                //Check if array isn't full
                if((i+j/2) < arrayLength){
                    //Copy the indermiate buffer in the array (16bits values)
                    recordArray[LEFT][i+j/2] = ((readBuff[j] & 0xFFFF0000) >> 16);
                    recordArray[RIGHT][i+j/2] = ((readBuff[j+1] & 0xFFFF0000) >> 16); 
                }
                else {
                    break;
                }
            }
        }
        else {
            ESP_LOGI(TAG,"I2S Reading error");
        }
    }
}
void I2S_RecordTask2(uint16_t arrayLength, int16_t recordArray[2][arrayLength]) {
    static const char *TAG = "I2S_RecordTask"; //For the LOGs
    
    static int32_t readBuff[READ_BUFF_SIZE2] = {0}; //Intermediate buffer
    const uint16_t cByteToRead = 4*READ_BUFF_SIZE2;  //number of bytes to read, when reading DMA buffers
    static size_t bytes_read = 0;  //to store the number of bytes that have been read, when reading DMA buffers

    for(uint32_t i = 0; i<arrayLength; i+=(READ_BUFF_SIZE2/2)) {
        if(i2s_read(I2S_NUM_0, &readBuff, cByteToRead, &bytes_read, portMAX_DELAY) == ESP_OK){
            for(uint16_t j = 0; j<READ_BUFF_SIZE2; j+=2){
                //Check if array isn't full
                if((i+j/2) < arrayLength){
                    //Copy the indermiate buffer in the array (16bits values)
                    recordArray[LEFT][i+j/2] = ((readBuff[j] & 0xFFFF0000) >> 16);
                    recordArray[RIGHT][i+j/2] = ((readBuff[j+1] & 0xFFFF0000) >> 16); 
                }
                else {
                    break;
                }
            }
        }
        else {
            ESP_LOGI(TAG,"I2S Reading error");
        }
    }
}