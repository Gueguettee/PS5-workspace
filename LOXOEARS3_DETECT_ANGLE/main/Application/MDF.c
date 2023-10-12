/**
  ****************************************************************************
  * @file    Angle.c
  * @author  Riasat Arafat
  * @date    10 Juillet 2023
  * @since   10 Juillet 2023
  * @brief   Program for the angle for the LOXO Ears 3 project
  ****************************************************************************/
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "led_strip/include/led_strip.h"

#include "../I2S-Driver/I2S.h"
#include "MDF.h"
#include "Application/CanTask.h"
#include "CAN/can.h"
#include "driver/twai.h"

#include "definitions.h"

// ----------------------------------------------------------------------------
//                                Definitions
// ----------------------------------------------------------------------------
static void calcul_Angle(void);


#define CarWidth (1)// Width of the car 
#define CarLength (3)//Length of the car
#define PI (3.1415926535)//Length of the car



#if defined(KALUGA_BOARD)
  #define BLINK_GPIO (45)
  #define GPIO_TEST1 (26)
  #define GPIO_TEST2 (33) 

#elif defined(DEVKITC_BOARD)
  #define BLINK_GPIO (7)
  //#define GPIO_TEST1 (4)
  //#define GPIO_TEST2 (5) 

#endif 

  uint64_t Tableau[4] = {{0}}; //array to store the timing of arrival
  int16_t Receive_value = 0;
  int SmallestTime = 0; // first mic to receive 
  int FirstMic = 0; // index of forst mic 
  float TDOA1 = 0;// difference of time of arrival between 2 mic
  float TDOA2 = 0;//difference of time of arrival between 2 mic
  double Angle1 = 0;//Angle with TDOA1
  double Angle2 = 0;//Angle with TDOA1
  double Angle1Deg = 0;//Angle in degres
  double Angle2Deg = 0; //Angle in degres
  int Counter = 0;//

// ----------------------------------------------------------------------------
//                              Main Application
// ----------------------------------------------------------------------------
void Angle(void* pvParameters){
  CAN_Message_t Message;
  CAN_Message_t Message1;
  TickType_t xLastWakeTime;

//  uint16_t freqBuff[CIRC_BUFFER_SIZE];
//   Tableau[0] = 1002;
//     Tableau[1] = 1000 ;
//       Tableau[2] = (float)1007.6;
//         Tableau[3] = (float)1005.4;
//     calcul_Angle();// Calcul the angle    
// printf("Angle1 = %f°, Angle2 = %f°\n",Angle1Deg,Angle2Deg);
 
  twai_message_t rx_msg;
  canInit();
  
  xLastWakeTime = xTaskGetTickCount();
  while(1){
    //canInit();
     //vTaskDelayUntil(&xLastWakeTime,500);
     twai_receive(&rx_msg, 110);
     /* printf("Data received at pos 0 : %X \n",rx_msg.data[0]);
     printf("Data received at pos 1 : %X \n",rx_msg.data[1]);
     printf("Data received at pos 3 : %X \n",rx_msg.data[2]);
     printf("Data received at pos 4 : %X \n",rx_msg.data[3]); */

  if (rx_msg.data[0] == 1){
    Tableau[0]= xTaskGetTickCount();//Save the arrival time of Mic 1
    printf("Temps1 = %jd\n",Tableau[0]);
    //Counter ++;
  }
  if (rx_msg.data[0] == 2){
    Tableau[1]= xTaskGetTickCount();//Save the arrival time of Mic 2
    printf("Temps2 = %jd\n",Tableau[1]);
    //Counter ++;
  }
  if (rx_msg.data[0] == 3){
    Tableau[2]= xTaskGetTickCount();//Save the arrival time of Mic 3
    printf("Temps3 = %jd\n",Tableau[2]);
    //Counter ++; 
  }
  if (rx_msg.data[0] == 4){
    Tableau[3]= xTaskGetTickCount();//Save the arrival time of Mic 4
    printf("Temps4 = %jd\n",Tableau[3]);
    //Counter ++;
  }
if ((Tableau[0]&Tableau[1]&Tableau[2]&Tableau[3])!=0)
{

  calcul_Angle();// Calcul the angle
   printf("Angle1 = %f°, Angle2 = %f°\n",Angle1Deg,Angle2Deg);


 // Counter = 0;
  for(int i = 0; i<4;i++) //Clear data received
  {
    Tableau[i] = 0;
  }

}
     for(int i = 0; i<TWAI_FRAME_MAX_DLC;i++) //Clear data received
     {
        rx_msg.data[i] = 0;
     }

    
     
    /* can_receive_data(&Message, 0);
    can_receive_data(&Message1, 1);
    if(Message != CAN_RECEIVE_ERROR)
     {
      printf("Data received at pos 0 : %X \n",Message);
      printf("Data received at pos 1 : %X \n",Message1);
      Message = 0;
      Message1 = 1;
     }    
     else{
        printf("Error1 : %X \n",Message);
        printf("Error 2: %X \n",Message1);
     }  */
    
  }
}

// ----------------------------------------------------------------------------
//                                Functions
// ----------------------------------------------------------------------------
static void calcul_Angle(void) {
SmallestTime = Tableau[0];
 FirstMic = 1;

 for (int i = 1; i < 4; i++) {
        if (Tableau[i] < SmallestTime) {
            SmallestTime = Tableau[i];  // Find the smallest time
            FirstMic = i+1;  // First Mic to receive
        }
    }
  

  if (FirstMic == 1){// if mic 1 detected first Front Left
    TDOA1 = (float)Tableau[3] - (float)Tableau[1]; 
    TDOA2 = (float)Tableau[3] -(float)Tableau[2];
    Angle1 = acos(((TDOA1/1000)*343)/CarLength);
    Angle2 = acos(((TDOA2/1000)*343)/CarWidth);
    Angle1Deg = Angle1 * (180.0/PI);
    Angle2Deg = Angle2 * (180.0/PI);

    Angle2Deg = 90 - Angle2Deg;// Angle 2 has to be same as Angle1
    printf("détecté en premier au micro 1, angle entre micro 4 et 2 en direction du micro 1\n");
  }

  if (FirstMic == 2){ // If mic 2 detected first Front Right
    TDOA1 = (float)Tableau[2] - (float)Tableau[0];
    TDOA2 = (float)Tableau[2] - (float)Tableau[3];
    Angle1 = acos(((TDOA1/1000)*343)/CarLength);
    Angle2 = acos(((TDOA2/1000)*343)/CarWidth);
    Angle1Deg = Angle1 * (180.0/PI);
    Angle2Deg = Angle2 * (180.0/PI);

    Angle2Deg = 90 - Angle2Deg;
    printf("détecté au micro 2, angle entre micro 3 et 1 en direction du micro 2\n");
  }
  if (FirstMic == 3){//If mic 3 detected first Rear Left
    TDOA1 = (float) Tableau[1] - (float)Tableau[3];
    TDOA2 = (float)Tableau[1] - (float)Tableau[0];
    Angle1 = acos(((TDOA1/1000)*343)/CarLength);
    Angle2 = acos(((TDOA2/1000)*343)/CarWidth);
    Angle1Deg = Angle1 * (180.0/PI);
    Angle2Deg = Angle2 * (180.0/PI);

    Angle2Deg = 90 - Angle2Deg;

    printf("détecté au micro 3, angle entre micro 2 et 4 en direction du micro 3\n");
  }
  if (FirstMic == 4){//If mic 4 detected first Rear Right
    TDOA1 = (float)Tableau[0] - (float)Tableau[2];
    TDOA2 = (float)Tableau[0] - (float)Tableau[1];
    Angle1 = acos(((TDOA1/1000)*343)/CarLength);
    Angle2 = acos(((TDOA2/1000)*343)/CarWidth);
    Angle1Deg = Angle1 * (180.0/PI);
    Angle2Deg = Angle2 * (180.0/PI);

    Angle2Deg = 90 - Angle2Deg;
    printf("détecté au micro 4, angle entre micro 1 et 3 en direction du micro 4\n");
  }
}