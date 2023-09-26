/**
  ****************************************************************************
  * @file    MDF.c
  * @author  Julien Michel
  * @date    20 November 2022
  * @since   20 November 2022
  * @brief   Program for the frequency analysis using a MDF function 
  *          algorithm for the LOXears2 project
  ****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"


#include "led_strip/include/led_strip.h"

#include "../I2S-Driver/I2S.h"
#include "MDF.h"

#include "definitions.h"

// ----------------------------------------------------------------------------
//                                Definitions
// ----------------------------------------------------------------------------
#define MIN_TIME_MS (500) //minimum duration of one siren tone in ms
#define MAX_TIME_MS (900) //maximum duration of one siren tone in ms
#define REC_TIME_MS (40) //duration of the record to analyze (measure time step) in ms^
#define REC_TIME_MS2 (10) //duration of the record to analyze (measure time step) in ms^


#define PERIODE_NUMBER (6) //number of periode (one tone) to confirm there is actually a siren 
#define ALARM_TIME (10) //duration of the alarm signal after enough periode of siren in seconds 

#define MIN_FREQUENCY (300) //min frequency of the scale in Hz
#define MAX_FREQUENCY (2000) //max frequency of the scale in Hz
#define FREQUENCY_RES (50) //frequency resolution in Hz

#define CHAN_FRAME ((REC_TIME_MS) * (SAMPLERATE / 1000)) //Number of data per channel in a record frame
#define CHAN_FRAME2 ((REC_TIME_MS2) * (SAMPLERATE / 1000)) //Number of data per channel in a record frame
#define FREQUENCY_NUMBER ((MAX_FREQUENCY - MIN_FREQUENCY) / (FREQUENCY_RES)) //number of frequency values btw. Fmin and Fmax 
#define CIRC_BUFFER_SIZE ((int32_t)floor(MIN_TIME_MS / (REC_TIME_MS+REC_TIME_MS2))) //size of the circular buffer to store the calculated frequencies

#if defined(KALUGA_BOARD)
  #define BLINK_GPIO (45)
  #define GPIO_TEST1 (26)
  #define GPIO_TEST2 (33) 

#elif defined(DEVKITC_BOARD)
  #define BLINK_GPIO (8)
  #define GPIO_TEST1 (4)
  #define GPIO_TEST2 (5) 

#endif 

// Forward declarations
/// @brief  Initialize circular buffer (all value to 0)
/// @param  buffer the address of the circular buffer to initialize
/// @return -
static void circBufferInit(uint16_t buff[CIRC_BUFFER_SIZE]);

/// @brief  Check if a siren is detected with a time analysis of the frequencies 
/// @param  buffer circular buffer of the actual detected frequencies
/// @param  index the actual index for the buffer, the index of the last measured frequency 
/// @return 1 if a siren is detected, 0 otherwise
static bool checkSiren(uint16_t buffer[CIRC_BUFFER_SIZE], uint16_t index);

/// @brief  Configure the RGB-LED to activate when siren is detected
/// @param  -
/// @return -
static void configure_led(void);

static led_strip_t *pStrip_a;
static bool maintainFlag = 0;
const TickType_t cDelay50ms = REC_TIME_MS/portTICK_PERIOD_MS;
const TickType_t cDelay1ms = REC_TIME_MS2/portTICK_PERIOD_MS;
//static const char *TAG1 = "MDF"; //For the LOGs


 
// ----------------------------------------------------------------------------
//                              Main Application
// ----------------------------------------------------------------------------
void MDF_Task(void* pvParameters){

//static const char *TAG1 = "Check"; //For the LOGs
//ESP_LOGI(TAG1,"DEBUT");
//static const char *TAG2 = "Main File";
//ESP_LOGI(TAG2, "[APP] Free memory: %lu bytes", esp_get_free_heap_size());
//ESP_LOGI(TAG2, "Valeur DELAY1ms: %lu ms", cDelay1ms);
//ESP_LOGI(TAG2, "Valeur DELAY50ms: %lu ms", cDelay50ms);




  uint16_t freqBuff[CIRC_BUFFER_SIZE];

  int16_t dataRL[2][CHAN_FRAME2+CHAN_FRAME] = {{0},{0}}; //array to store the sound datas
  int16_t dataRL2[2][CHAN_FRAME2] = {{0},{0}}; //array to store the sound datas
  //int16_t dataRL3[2][CHAN_FRAME2+CHAN_FRAME] = {{0},{0}}; //array to store the sound datas

  static uint32_t sumTable[2][FREQUENCY_NUMBER] = {0}; //array to store the sum of the differences according to the delay
  //int16_t Tableau[2][FREQUENCY_NUMBER][CHAN_FRAME+CHAN_FRAME2]= {{{0},{0},{0}}};

  static uint16_t index = 0; //to parse the circular buffer
  static uint16_t offset = 0; //number of value in the delay (used later)
  static uint32_t sumR = 0; //used to sum all the differences btw. signal and delayed signal of right channel
  static uint32_t sumL = 0; //used to sum all the differences btw. signal and delayed signal of left channel
  
  TickType_t xLastWakeTime;

  circBufferInit(freqBuff); //Initialize circular Buffer
  I2S_init(); //Initialize I2S
  configure_led(); //Configure RGB-LED


  
  
  //Configure gpios for logic analyzer test (comment for final use)
   gpio_reset_pin(GPIO_TEST1);
   gpio_set_direction(GPIO_TEST1, GPIO_MODE_OUTPUT);
   gpio_set_level(GPIO_TEST1, 0);

  
   gpio_reset_pin(GPIO_TEST2);
   gpio_set_direction(GPIO_TEST2, GPIO_MODE_OUTPUT);
   gpio_set_level(GPIO_TEST2, 0);
  

    // Wait for the next cycle.
    //vTaskDelayUntil(&xLastWakeTime, cDelay50ms);

    //For the speed test (comment for final use)
  //  gpio_set_level(GPIO_TEST1, 1);

    //Record values first time for window
    I2S_RecordTask((CHAN_FRAME+CHAN_FRAME2), dataRL);
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < (CHAN_FRAME+CHAN_FRAME2); j++) {
            printf("%d ", dataRL[i][j]);
        }
        printf("\n");
    }
    
    //For the speed test (comment for final use)
   // gpio_set_level(GPIO_TEST2, 1);
// Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  while(1){
    // Wait for the next cycle.
    long long int Timer1 = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime, cDelay1ms);
    long long int Timer2 = xTaskGetTickCount();
    float diff = Timer2 - Timer1;
    //printf("Wait time: %f ms\n", diff);

    //For the speed test (comment for final use)
  //  gpio_set_level(GPIO_TEST1, 1);
  static int count;
    //Record values, shift old datas and put new data 
    I2S_RecordTask(CHAN_FRAME2, dataRL2);
    for (int i = 0; i < 2; i++) {
    memmove(dataRL[i], dataRL[i]+ CHAN_FRAME2, (abs(CHAN_FRAME - CHAN_FRAME2)) * sizeof(int16_t));
    memcpy(dataRL[i], dataRL2[i], CHAN_FRAME2 * sizeof(int16_t));
    }
    if(count == 100){
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < (CHAN_FRAME2); j++) {
            printf("%d ", dataRL2[i][j]);
        }
        printf("\n");
    }
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < (CHAN_FRAME2+CHAN_FRAME); j++) {
            printf("%d ", dataRL[i][j]);
        }
        printf("\n");
    }
    count = 0;
    }
    count++;
    
    
    //For the speed test (comment for final use)
  //  gpio_set_level(GPIO_TEST2, 1);
    for(uint16_t lagFreq = MIN_FREQUENCY; lagFreq < MAX_FREQUENCY; lagFreq += FREQUENCY_RES){
      sumR = 0; 
      sumL = 0;
      offset = round(SAMPLERATE / lagFreq); //number of value in the delay (round) 

      //Sum of the differences btw. the signal and the delayed signal
      for(uint16_t n = 0; n < (CHAN_FRAME + CHAN_FRAME2 - offset); n++){
        sumL += abs(dataRL[LEFT][n + offset] - dataRL[LEFT][n]);
        sumR += abs(dataRL[RIGHT][n + offset] - dataRL[RIGHT][n]);
        }
      sumTable[LEFT][(lagFreq - MIN_FREQUENCY) / FREQUENCY_RES] = sumL;
      sumTable[RIGHT][(lagFreq - MIN_FREQUENCY) / FREQUENCY_RES] = sumR;
      }    
    uint16_t frequency[2] = {0}; //array to store the principal frequency of both channels

    //Find the frequency with the lowest difference at the corresponding delay (principal frequency)
    for(uint16_t f = 0; f < FREQUENCY_NUMBER; f++){
      if(sumTable[LEFT][f] <= sumTable[LEFT][frequency[LEFT]]){
        frequency[LEFT] = f;
      }
      if(sumTable[RIGHT][f] <= sumTable[RIGHT][frequency[RIGHT]]){
        frequency[RIGHT] = f;
      }
    }
    
    //Store the found frequency in circular buffer 
    freqBuff[index] = frequency[LEFT] * FREQUENCY_RES + MIN_FREQUENCY;

    printf("%d\n ", freqBuff[index]);

    //maybe harmonic ?
    if((freqBuff[index] >= 750) && (freqBuff[index] < 1500)){
      freqBuff[index] /= 2;
    }
    //not intersting frequency
    if(!((freqBuff[index] <= 720) && (freqBuff[index] >= 320))){
      freqBuff[index] = frequency[RIGHT] * FREQUENCY_RES + MIN_FREQUENCY;;
      //maybe harmonic ?
      if((freqBuff[index] >= 750) && (freqBuff[index] < 1500)){
        freqBuff[index] /= 2;
      }
      //not interesting frequency      
      if(!((freqBuff[index] <= 720) && (freqBuff[index] >= 320))){
        freqBuff[index] = 0;
      }
    }

    //Just for the speed test
  //  gpio_set_level(GPIO_TEST2, 0);

    if(checkSiren(freqBuff, index)){
      /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
      pStrip_a->set_pixel(pStrip_a, 0, 50, 0, 0);
      /* Refresh the strip to send data */
      pStrip_a->refresh(pStrip_a, 100);
    }
    else if(!maintainFlag){
      /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
      pStrip_a->set_pixel(pStrip_a, 0, 0, 0, 10);
      /* Refresh the strip to send data */
      pStrip_a->refresh(pStrip_a, 100);
    }
    
    //Next index in circular buffer 
    index = index < (CIRC_BUFFER_SIZE - 1) ? index + 1 : 0;
  }
    //Just for the speed test
  //  gpio_set_level(GPIO_TEST1, 0);
}
// ----------------------------------------------------------------------------
//                                Functions
// ----------------------------------------------------------------------------
static void circBufferInit(uint16_t buff[CIRC_BUFFER_SIZE]){
  for(uint16_t i = 0; i < CIRC_BUFFER_SIZE; i++){
   buff[i] = 0;
  }
}

static bool checkSiren(uint16_t buffer[CIRC_BUFFER_SIZE], uint16_t index){
  static const uint16_t tol = FREQUENCY_RES;  //the tolerance to take as same frequency (+-)
  
  static uint32_t maintainTime = 0;
  static uint32_t time = 0;  
  static bool sirenFlag = 0;
  static uint16_t periode = 0;
  static uint16_t oldFreq = 0;

  uint16_t count = 1; //to count the number of same frequency as the last measure

  if(sirenFlag){
    time += REC_TIME_MS;
    //if time of a tone > MAX possible time of a tone -> no siren
    if(time > 2*MAX_TIME_MS){
      sirenFlag = 0;
      periode = 0;
      oldFreq = 0;
    }
  }
  for(uint16_t i = 1; i < CIRC_BUFFER_SIZE; i++){
    if((buffer[index] >= (buffer[(index + i) % CIRC_BUFFER_SIZE] - tol)) && 
       (buffer[index] <= (buffer[(index + i) % CIRC_BUFFER_SIZE] + tol))){
      count ++;
    }
  }
  //check if at least 80% of the last measured frequencies are the same as the very last measure
  if((count >= floor((8 * CIRC_BUFFER_SIZE) / 10)) && (buffer[index] != 0)){
    //if siren possibly detected (one tone detected)
    if(sirenFlag){
      //check if the new frequency is approx. 4/3 * greater as the old tone (1.2 < 1.33 < 1.4)
      if((buffer[index] >= (6 * oldFreq) / 5) && (buffer[index] <= (7 * oldFreq) / 5)){
        periode ++;
        sirenFlag = 0;
      }
      //check if the new frequency is approx. 4/3 * smaller as the old tone (1.2 < 1.33 < 1.4)
      else if((buffer[index] <= (5 * oldFreq) / 6) && (buffer[index] >= (5 * oldFreq) / 7)){
        periode ++;
        sirenFlag = 0;
      }
    }
    //siren possibly detected (one tone detected)
    else{
      sirenFlag = 1;
      oldFreq = buffer[index];
      time = MIN_TIME_MS;
    }
  }

  //to maintain the alarm (ALARM_TIME sec) if it is sure there is a siren
  if(maintainFlag) maintainTime += REC_TIME_MS;
  if(maintainTime >= ALARM_TIME * 1000) maintainFlag = 0;
  if(periode >= PERIODE_NUMBER){
    maintainFlag = 1;
    maintainTime = 0;
  }

  if(periode >= 1) return 1;
  return 0;
}

static void configure_led(void) {
    /* LED strip initialization with the GPIO and pixels number*/
    pStrip_a = led_strip_init(0, BLINK_GPIO, 1);
    /* Set all LED off to clear all pixels */
    pStrip_a->clear(pStrip_a, 50);
}