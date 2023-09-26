/*  FILE NAME:              types.h
*   CREATION DATE:          15.05.2022
*   LAST MODIFICATION:      15.06.2022
*   CREATOR:                Augustin GUILLAUME
*   COMPANY:                LOXO AG, HEAI-FR, ROSAS
*   DESCRIPTION:            Header file containing types for both CAN and DRV projects. Needs to be
*                           included in the main project as well.
*/

#ifndef TYPES_H
#define TYPES_H

//Desired positions of the curtains
typedef int16_t Positions_array_t[4];

#define LEFT_BOX        0x1
#define RIGHT_BOX       0x2

//Connect input active HIGH
#define CONNECTED       1
#define UNCONNECTED     0

#define B_L_C_L 0   //Box Left, curtain left
#define B_L_C_R 1   //Box Right, curtain right
#define B_R_C_L 2   //Box Right, curtain left
#define B_R_C_R 3   //Box Right, curtain right

#endif

//######################################## END OF FILE ################################################