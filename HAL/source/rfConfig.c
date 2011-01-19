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
Filename:     rfConfig.c
Target:       cc2430, cc2431, cc1110, cc2510, cc2511
Author:       EFU
Revised:      26/10-2005
Revision:     0.1
******************************************************************************/

#include "hal.h"

//-----------------------------------------------------------------------------
// See hal.h for a description of this function.
//-----------------------------------------------------------------------------
BOOL halRfConfig(UINT32 frequency)
{
   BOOL status;

   //Turning on crystal oscillator
   SET_MAIN_CLOCK_SOURCE(CRYSTAL);

   // Setting the frequency
   halRfSetRadioFrequency(frequency);

#if (chip == 2430 || chip == 2431)
   // Checking that the entered frequency is valid
   if (frequency > 2047000)
   {

     // turning on power to analog part of radio and waiting for voltage regulator.
     RFPWR = 0x04;
     while( RFPWR & 0x10 );

     // Turning off Address Decoding
     MDMCTRL0H &= ~ADR_DECODE;

     // Setting for AUTO CRC
     MDMCTRL0L |= AUTO_CRC;

     // Turning on AUTO_TX2RX
     FSMTC1 = ((FSMTC1 & (~AUTO_TX2RX_OFF & ~RX2RX_TIME_OFF))  | ACCEPT_ACKPKT);

     // Turning off abortRxOnSrxon.
     FSMTC1 &= ~0x20;

#endif
#if (chip == 1110)
   if (frequency < 1000000)
   {
     // 315 MHz band
     if (frequency < 400000)
     {
       PA_TABLE0 = 0x63;
     }
     // 433 MHz band
     else if (frequency < 800000)
     {
       PA_TABLE0 = 0x33;
     }
     // 868 MHz band
     else if (frequency < 890000)
     {
       PA_TABLE0 = 0x66;
     }
     // 915 MHz band
     else
     {
       PA_TABLE0 = 0x66;
     }

     /**********************************************************************
     *                                                                     *
     * 250 kbps MSK  setup (for other data rates or modulation formats,    *
     * please see SmartRF Studio).                                         *
     *                                                                     *
     **********************************************************************/

     // Dynamic packet length
     PKTLEN = 0xFF;
     PKTCTRL0 = 0x05;

     // Append status
     PKTCTRL1 = 0x84;

     // IF frequency
     FSCTRL1 = 0x0B;

     //
     //      FSCTRL0 = 0x00;

     // filter BW, data rate,
     //      MDMCFG4 = 0x0E;
     MDMCFG4 = 0x2D;
     MDMCFG3 = 0x3B;
     // Modulation format, detection level
     MDMCFG2 = 0x73;
     MDMCFG1 = 0x42;
     //      MDMCFG1 = 0x22;

     // Deviation setting
     DEVIATN = 0x00;

     // Calibration synth
     MCSM0 = 0x10;

     // Frequency offset compensation configuration
     FOCCFG = 0x1D;

     // Bit synchronization
     BSCFG = 0x1C;

     // AGC and front end settings (from SmartRf04)
     AGCCTRL2 = 0xC7;
     //      AGCCTRL1 = 0x40;
     AGCCTRL1 = 0x00;
     AGCCTRL0 = 0xB2;
     FREND1 = 0xB6;

     FSCAL3 = 0xEA;

     // Synth calibration
     //      FSCAL0 = 0x19;
     FSCAL0 = 0x11;
     //      PA_TABLE0 = 0xC3;

     // Calibrating synth.
     SIDLE();
     SCAL();
     while(MARCSTATE != 0x01);

     INT_SETFLAG(INUM_RFTXRX,INT_CLR);
#endif

#if (chip == 1111)  
   if (frequency < 1000000)
   {
     // 315 MHz band
     if (frequency < 400000)
     {
       PA_TABLE0 = 0x63;
     }
     // 433 MHz band
     else if (frequency < 800000)
     {
       PA_TABLE0 = 0x33;
     }
     // 868 MHz band
     else if (frequency < 890000)
     {
       PA_TABLE0 = 0x66;
     }
     // 915 MHz band
     else
     {
       PA_TABLE0 = 0x66;
     }

     /**********************************************************************
     *                                                                     *
     * 250 kbps MSK  setup (for other data rates or modulation formats,    *
     * please see SmartRF Studio).                                         *
     *                                                                     *
     **********************************************************************/
         
     // Dynamic packet length
     PKTLEN = 0xFF;
     PKTCTRL0 = 0x05;
     // Append status
     PKTCTRL1 = 0x04;

     // IF frequency
     FSCTRL1 = 0x0A;
     FSCTRL0 = 0x00;

     // filter BW, data rate,
     MDMCFG4 = 0x1D;
     MDMCFG3 = 0x55;
     // Modulation format, detection level
     MDMCFG2 = 0x73;
     MDMCFG1 = 0x23;
     MDMCFG0 = 0x11;

     // Deviation setting
     DEVIATN = 0x00;

     // Calibration synth
     MCSM2 = 0x07;
     MCSM1 = 0x30;
     MCSM0 = 0x10;

     // Frequency offset compensation configuration
     FOCCFG = 0x1D;

     // Bit synchronization
     BSCFG = 0x1C;

     // AGC and front end settings (from SmartRf04)
     AGCCTRL2 = 0xC7;
     AGCCTRL1 = 0x00;
     AGCCTRL0 = 0xB2;
     FREND1 = 0xB6;

     // Synth calibration
     FSCAL3 = 0xEA;
     FSCAL0 = 0x11;

     // Are these needed ?
     // From Smart RF Studio
     FOCCFG = 0x1D;
     BSCFG = 0x1C;
     FSTEST = 0x59;
     PTEST = 0x7F;
     AGCTEST = 0x3F;
     TEST2 = 0x88;
     TEST1 = 0x31;
     TEST0 = 0x0B;

     // Calibrating synth.
     SIDLE();
     SCAL();
     while(MARCSTATE != 0x01);

     INT_SETFLAG(INUM_RFTXRX,INT_CLR);
#endif

#if(chip == 2510)
   if (frequency > 2400000)
   {
     /**********************************************************************
     *                                                                     *
     * 250 kbps MSK  setup (for other data rates or modulation formats,    *
     * please see SmartRF Studio).                                         *
     *                                                                     *
     **********************************************************************/

     // Dynamic packet length and append status
     PKTLEN = 0xFF;
     PKTCTRL0 = 0x05;
     PKTCTRL1 = 0x04;

     // IF frequency
     FSCTRL1 = 0x0A;
     FSCTRL0 = 0x00;

     // filter BW, data rate,
     MDMCFG4 = 0x2D;
     MDMCFG3 = 0x3B;

     // Modulation format, detection level
     MDMCFG2 = 0x73;
     MDMCFG1 = 0x22;
     MDMCFG0 = 0xF8;

     // Deviation setting
     DEVIATN = 0x00;

     // Calibration synth
     MCSM2 = 0x07;
     MCSM1 = 0x30;
     MCSM0 = 0x10;

     // Frequency offset compensation configuration
     FOCCFG = 0x1D;

     // Bit synchronization
     BSCFG = 0x1C;

     // AGC settings
     AGCCTRL2	= 0xC7;
     AGCCTRL1	= 0x00;
     AGCCTRL0	= 0xB2;

     // Front end settings (from SmartRf04)
     FREND1 = 0xB6;
     FREND0 = 0x10;

     // Synth calibration
     FSCAL3 = 0xEA;
     FSCAL2 = 0x0A;
     FSCAL1 = 0x00;
     FSCAL0 = 0x11;

     // From Smart RF Studio
     FOCCFG = 0x1D;
     BSCFG = 0x1C;
     FSTEST = 0x59;
     PTEST = 0x7F;
     AGCTEST = 0x3F;
     TEST2 = 0x88;
     TEST1 = 0x31;
     TEST0 = 0x0B;


     // Output power
     PA_TABLE0 = 0xFF;

     // Calibrating synth.
     SIDLE();
     SCAL();
     while(MARCSTATE != 0x01);

     INT_SETFLAG(INUM_RFTXRX,INT_CLR);

#endif

#if(chip == 2511)
   if (frequency > 2400000)
   {
     /**********************************************************************
     *                                                                     *
     * 250 kbps MSK  setup (for other data rates or modulation formats,    *
     * please see SmartRF Studio).                                         *
     *                                                                     *
     **********************************************************************/

     // Dynamic packet length and append status
     PKTLEN = 0xFF;
     PKTCTRL0 = 0x05;
     PKTCTRL1 = 0x04;

     // IF frequency
     FSCTRL1 = 0x0A;
     FSCTRL0 = 0x00;

     // filter BW, data rate,
     MDMCFG4 = 0x1D;
     MDMCFG3 = 0x55;

     // Modulation format, detection level
     MDMCFG2 = 0x73;
     MDMCFG1 = 0x23;
     MDMCFG0 = 0xF8;

     // Deviation setting
     DEVIATN = 0x00;

     // Calibration synth
     MCSM0 = 0x07;
     MCSM1 = 0x30;
     MCSM0 = 0x10;

     // Frequency offset compensation configuration
     FOCCFG = 0x1D;

     // Bit synchronization
     BSCFG = 0x1C;

     // AGC settings
     AGCCTRL2	= 0xC7;
     AGCCTRL1	= 0x00;
     AGCCTRL0	= 0xB2;

     // Front end settings (from SmartRf04)
     FREND1 = 0xB6;
     FREND0 = 0x10;

     // Synth calibration
     FSCAL3 = 0xEA;
     FSCAL2 = 0x0A;
     FSCAL1 = 0x00;
     FSCAL0 = 0x11;

     // From Smart RF Studio
     FOCCFG = 0x1D;
     BSCFG = 0x1C;
     FSTEST = 0x59;
     PTEST = 0x7F;
     AGCTEST = 0x3F;
     TEST2 = 0x88;
     TEST1 = 0x31;
     TEST0 = 0x0B;


     // Output power
     PA_TABLE0 = 0xFF;

     // Calibrating synth.
     SIDLE();
     SCAL();
     while(MARCSTATE != 0x01);

     INT_SETFLAG(INUM_RFTXRX,INT_CLR);

#endif
     status = TRUE;
   }
   else {
      status = FALSE;
   }

   return status;
}


//-----------------------------------------------------------------------------
// See hal.h for a description of this function.
//-----------------------------------------------------------------------------
void halRfSetRadioFrequency(UINT32 frequency)
{
#if (chip == 2430 || chip == 2431)
   frequency /= (UINT32)1000;
   frequency -= (UINT32)2048;

   FSCTRLL = LOBYTE(frequency);
   FSCTRLH &= ~0x03;
   FSCTRLH |= (HIBYTE(frequency) & 0x03);
#endif
#if (chip == 0000)
   // TODO: make sure casting is done correctly
#endif
#if (chip == 1110 || chip == 2510)
   frequency = (frequency << 10);
   frequency /= 1000;
   frequency = (frequency << 6);
   frequency /= 26;
   FREQ0 = (BYTE) frequency;
   frequency >>= 8;
   FREQ1 = (BYTE) frequency;
   frequency >>= 8;
   FREQ2 = (BYTE) frequency;
#endif
#if (chip == 1111 || chip == 2511)
   frequency = (frequency << 10);
   frequency /= 1000;
   frequency = (frequency << 6);
   frequency /= 24;
   FREQ0 = (BYTE) frequency;
   frequency >>= 8;
   FREQ1 = (BYTE) frequency;
   frequency >>= 8;
   FREQ2 = (BYTE) frequency;
#endif
   return;
}

