/**
  ****************************************************************************
  * @file    I2S.h
  * @author  Julien Michel
  * @date    16 November 2022
  * @since   29 October 2022
  * @brief   Header files for I2S driver to read the microphone module 
  *          IMD69D130 Shield2Go for the LOXears2 project
  ****************************************************************************/
#ifndef I2S_H
  #define I2S_H

  #define LEFT (0)
  #define RIGHT (1)

  #define SAMPLERATE (8000) //Sample rate of the recording 

  /** @brief Initialize the I2S communication with the good parameters for 
   *         the Shield2Go board.
  */
  void I2S_init(void);

  /** @brief Save values from I2S in an array and send it to USB with the LOGs
   *  @param arrayLength length of the array to store the recorded datas in
   *  @param recordArray 2-dimensional array to store the recorded datas of both channel 
  */
  void I2S_RecordTask(uint16_t arrayLength, int16_t recordArray[2][arrayLength]);
  void I2S_RecordTask2(uint16_t arrayLength, int16_t recordArray[2][arrayLength]);


#endif