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
Filename:     setTimer2Period.c
Target:       cc2430, cc2431, cc1110, cc2510, cc2511
Author:       EFU
Revised:      26/10-2005
Revision:     0.1
******************************************************************************/
#include "hal.h"

//-----------------------------------------------------------------------------
// See hal.h for a description of this function.
//-----------------------------------------------------------------------------
#if (chip == 2430 || chip == 2431)
BOOL halSetTimer2Period(BYTE mode, DWORD period){
   if(mode&TIMER2_MAC_TIMER){
      T2CAPHPH = 0x28;  // setting for 320 u-second periods as specified by 802.15.4
      T2CAPLPL = 0x00;  // (0x2800) / 32 = 320 u-seconds
   }
   else {
       T2CAPHPH = 0x7D; // Setting timer to have 1 m-second period
       T2CAPLPL = 0x00; // (0x7D00) / 32 = 1000 u-seconds
   }

   if(period){
       if(period&0xFFF00000) {return 0;}// Setting the number of periods (timer overflows) to generate
       T2PEROF0 = (BYTE) period;        // an interrupt.
       period = (period >> 8);
       T2PEROF1 = (BYTE) period;
       period = ((period >> 8)&0x0F);
       T2PEROF2 = ( T2PEROF2&~0x0F | (BYTE)period );
   }
   return 1;
}
#endif
#if (chip == 1110 || chip == 2510)
BOOL halSetTimer2Period(UINT32 period, UINT8* cnt, UINT8* presc)
{
   BYTE tip = 0;
   UINT16 prescaler = 1;
   UINT16 counter;

   // Times 26 and devided by 64 (crystal clock frequency and minimum tick period of T2).
   period = (UINT32)((float)period * 0.40625);

   // Compensating for TICKSPD.
   period = (period >>  TICKSPD);


   while(period > 65280)
   {
      tip++;
      if(tip == 3)
      {  // Step from 256 to 1024 clock cycles
         period = period >> 1;
      }
      period = period >> 1;
   }

   if(tip > 3)
   {
      return FALSE;
   }

   while(((counter = (period / prescaler))  > 255))
   {
      prescaler++;
   }

   TIMER2_SET_COUNTER((UINT8)counter);
   TIMER2_SET_PRESCALER((UINT8) prescaler);
   TIMER2_SET_TICK_PERIOD(tip);

   *cnt = (UINT8) counter;
   *presc = (UINT8) prescaler;

   return TRUE;
}
#endif
#if (chip == 1111 || chip == 2511)
BOOL halSetTimer2Period(UINT32 period, UINT8* cnt, UINT8* presc)
{
   BYTE tip = 0;
   UINT16 prescaler = 1;
   UINT16 counter;
   BYTE temp;

   // Times 24 and devided by 64 (crystal clock frequency and minimum tick period of T2).
   period = (UINT32)((float)period * 0.375);

   // Compensating for TICKSPD
   temp = TICKSPD;//to avoid IAR warning
   if(temp <= CLKSPD) { period >>= TICKSPD; }
   else { period >>= CLKSPD; }

   while(period > 65280)
   {
      tip++;
      if(tip == 3)
      {  // Step from 256 to 1024 clock cycles
         period = period >> 1;
      }
      period = period >> 1;
   }

   if(tip > 3)
   {
      return FALSE;
   }

   while(((counter = (period / prescaler))  > 255))
   {
      prescaler++;
   }

   TIMER2_SET_COUNTER((UINT8)counter);
   TIMER2_SET_PRESCALER((UINT8) prescaler);
   TIMER2_SET_TICK_PERIOD(tip);

   *cnt = (UINT8) counter;
   *presc = (UINT8) prescaler;

   return TRUE;
}
#endif
