/*
+------------------------------------------------------------------------------
|  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.
|
|  IMPORTANT: Your use of this Software is limited to those specific rights
|  granted under the terms of a software license agreement between the user who
|  downloaded the software, his/her employer (which must be your employer) and
|  Texas Instruments Incorporated (the "License"). You may not use this Software
|  unless you agree to abide by the terms of the License. The License limits
|  your use, and you acknowledge, that the Software may not be modified, copied
|  or distributed unless embedded on a Texas Instruments microcontroller or used
|  solely and exclusively in conjunction with a Texas Instruments radio
|  frequency transceiver, which is integrated into your product. Other than for
|  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
|  works of, modify, distribute, perform, display or sell this Software and/or
|  its documentation for any purpose.
|
|  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
|  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
|  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
|  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
|  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
|  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
|  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING
|  BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
|  CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
|  SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
|  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
|
|  Should you have any questions regarding your right to use this Software,
|  contact Texas Instruments Incorporated at www.TI.com.
|
+------------------------------------------------------------------------------

Filename:     setTimer34Period.c
Target:       cc2430, cc2431, cc1110, cc2510, cc2511
Author:       EFU
Revised:      26/10-2005
Revision:     0.1
******************************************************************************/
#include "hal.h"


//------------------------------------------------------------------------------------------------------
// See hal.h for a description of this function.
//------------------------------------------------------------------------------------------------------
BYTE halSetTimer34Period(BYTE timer, DWORD period){
  BYTE div = 0;

  if(TICKSPD > 5) { // Checking that the period is not too short.
    if( (period < 2*(TICKSPD-5)) && (period != 0) ){
      return 0;
    }
  }

  if(period == 0){  // If period is 0, max period length and max prescaler
    div = 7;  // division is used.
    period = 255;
  } else {
#if (chip == 2430 || chip == 2431)
    period = ((period*32) >> TICKSPD);// Determining how many timer ticks the period consist of
#endif
#if (chip == 1110 || chip == 2510)
    period = ((period*26) >> TICKSPD);// Determining how many timer ticks the period consist of
#endif
#if (chip == 1111 || chip == 2511)
    period = ((period*24) >> TICKSPD);// Determining how many timer ticks the period consist of
#endif
    while(period > 255){              // If the period is too long, the prescaler division is
      period = (period >> 1);   // increased.
      div++;
      if(div > 7){              // If the period is too long when using max prescaler division,
        return 0;         // 0 is returned.
      }
    }
  }

  if(timer == 4){
    // Timer 4 selected
    T4CTL |= (div << 5);              // Setting prescaler value
    T4CC0 = (BYTE) period;            // Setting timer value.
  } else if(timer == 3){
    // Timer 3 selected
    T3CTL |= (div << 5);              // Setting prescaler value
    T3CC0 = (BYTE) period;            // Setting timer value.
  } else {
    return 0;
  }

  return period;
}
