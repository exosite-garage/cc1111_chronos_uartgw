// *************************************************************************************************
//
// Copyright 2009 BM innovations GmbH (www.bm-innovations.com), all rights reserved.
//
// This trial version of the "BlueRobin(TM) transmitter library for the Texas Instruments 
// CC1111 SoC" may be used for non-profit non-commercial purposes only. If you want to use 
// BlueRobin(TM) in a commercial project, please contact the copyright holder for a 
// separate license agreement.  
//
// By using this trial version of the "BlueRobin(TM) transmitter library for the Texas Instruments 
// CC430 SoC", you implicitly agree that you will not modify, adapt, disassemble, decompile, 
// reverse engineer, translate or otherwise attempt to discover the source code of the 
// "BlueRobin(TM) transmitter library for the Texas Instruments CC1111 SoC".
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
//
// IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF 
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// *************************************************************************************************
// Project setup for USB dongle
// *************************************************************************************************


#ifndef __PROJECT_H
#define __PROJECT_H


// *************************************************************************************************
// Include section

#include <bm.h>
#include "hal.h"

// *************************************************************************************************
// Defines section

//#define TX_SIMPLE_TEST
//#define TX_MP_TEST

// Debug
#ifdef CC1111EM
  
  #define TOGGLE_LED              { if ((P1_1 & BIT0) == BIT0) P1_1 &= ~BIT0; else P1_1 |= BIT0; }
  #define LED_ON                  { P1_1 |= BIT0; }
  #define LED_OFF                 { P1_1 &= ~BIT0; }

  #define DEBUG_OUTPUT
  #ifdef DEBUG_OUTPUT
    #define DBG_PIN_H(port, bit)	{port |= bit;}
    #define DBG_PIN_L(port, bit)	{port &= ~bit;}
    #define TOGGLE_P0_5           { if ((P0_5 & BIT0) == BIT0) P0_5 &= ~BIT0; else P0_5 |= BIT0; } 
    #define TOGGLE_P0_4           { if ((P0_4 & BIT0) == BIT0) P0_4 &= ~BIT0; else P0_4 |= BIT0; } 
    #define TOGGLE_P0_3           { if ((P0_3 & BIT0) == BIT0) P0_3 &= ~BIT0; else P0_3 |= BIT0; } 
    #define TOGGLE_P0_2           { if ((P0_2 & BIT0) == BIT0) P0_2 &= ~BIT0; else P0_2 |= BIT0; } 
    #define TOGGLE_P0_1           { if ((P0_1 & BIT0) == BIT0) P0_1 &= ~BIT0; else P0_1 |= BIT0; } 
    #define TOGGLE_P0_0           { if ((P0_0 & BIT0) == BIT0) P0_0 &= ~BIT0; else P0_0 |= BIT0; }
  #else
    #define DBG_PIN_H(port, bit)	{ }
    #define DBG_PIN_L(port, bit)	{ }
    #define TOGGLE_P0_5             { } 
    #define TOGGLE_P0_4             { } 
    #define TOGGLE_P0_3             { } 
    #define TOGGLE_P0_2             { } 
    #define TOGGLE_P0_1             { } 
    #define TOGGLE_P0_0             { } 
  #endif

#else
    
  #define TOGGLE_LED              { if ((P1_0 & BIT0) == BIT0) P1_0 &= ~BIT0; else P1_0 |= BIT0; }
  #define LED_ON                  { P1_0 |= BIT0; }
  #define LED_OFF                 { P1_0 &= ~BIT0; }

  #define TP_H                    { P1_4 |= BIT0; }
  #define TP_L                    { P1_4 &= ~BIT0; }
  
  #define DBG_PIN_H(port, bit)	  { }
  #define DBG_PIN_L(port, bit)	  { }
  #define TOGGLE_P0_5             { } 
  #define TOGGLE_P0_4             { } 
  #define TOGGLE_P0_3             { } 
  #define TOGGLE_P0_2             { } 
  #define TOGGLE_P0_1             { } 
  #define TOGGLE_P0_0             { } 

#endif

// Product ID
#define PRODUCT_ID    (0x12345678)

// BlueRobin TX serial
#define TX_SERIAL_NO  (1234567u)

// Radio to be used
//#define CC2500
#define CC1100


#define NO				(0)
#define YES				(1)


/************************************************************
* STANDARD BITS
************************************************************/
#define BIT0                   (0x0001)
#define BIT1                   (0x0002)
#define BIT2                   (0x0004)
#define BIT3                   (0x0008)
#define BIT4                   (0x0010)
#define BIT5                   (0x0020)
#define BIT6                   (0x0040)
#define BIT7                   (0x0080)
#define BIT8                   (0x0100)
#define BIT9                   (0x0200)
#define BITA                   (0x0400)
#define BITB                   (0x0800)
#define BITC                   (0x1000)
#define BITD                   (0x2000)
#define BITE                   (0x4000)
#define BITF                   (0x8000)

#endif // __PROJECT_H
