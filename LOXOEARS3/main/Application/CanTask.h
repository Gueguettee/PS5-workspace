/**
  ****************************************************************************
  * @file    CanTask.h
  * @author  Julien Michel
  * @date    6 December 2022
  * @since   6 December 2022
  * @brief   Header files for the CAN-communication between the esp32 and the 
  *          central computer for the LOXears2 project
  ****************************************************************************/
#ifndef CanTask_H
  #define CanTask_H

  void canSend(void);

  void canSendOnce(void);

  void canInit(void);

#endif