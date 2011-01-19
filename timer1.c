// *************************************************************************************************
//
//	Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/ 
//	 
//	 
//	  Redistribution and use in source and binary forms, with or without 
//	  modification, are permitted provided that the following conditions 
//	  are met:
//	
//	    Redistributions of source code must retain the above copyright 
//	    notice, this list of conditions and the following disclaimer.
//	 
//	    Redistributions in binary form must reproduce the above copyright
//	    notice, this list of conditions and the following disclaimer in the 
//	    documentation and/or other materials provided with the   
//	    distribution.
//	 
//	    Neither the name of Texas Instruments Incorporated nor the names of
//	    its contributors may be used to endorse or promote products derived
//	    from this software without specific prior written permission.
//	
//	  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//	  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//	  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//	  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//	  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//	  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//	  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//	  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//	  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//	  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//	  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *************************************************************************************************
// Timer1 driver
// *************************************************************************************************


// *************************************************************************************************
// Include section
#include "timer1.h"


// *************************************************************************************************
// Defines section
#define ACLK_SPEED_HZ         32768u
#define TCLK_SPEED_HZ         187500u
#define ACLK_PER_FULL_CYCLE   11453u // (32768*65536/187500=11453,25) - round up for later correction


// *************************************************************************************************
// Global Variable section
struct timer1 sTimer1;



// *************************************************************************************************
// Implementation

void reset_timer1(void)
{
  sTimer1.enable = 0;
  sTimer1.iflag = 0;
  sTimer1.aclk = 0;
  sTimer1.nb_full_cycles = 0;
  sTimer1.last_cycle_count = 0;
  sTimer1.usb_service_disable = 0;
  sTimer1.cycles = 0;
}



u8 set_timer1_cycles(void)
{
  int t1cc0;
  char repeat = 0;

  if (sTimer1.nb_full_cycles > 0)
  {
    // Disable IM during timer set
    T1CCTL0 &= ~BIT6;
  
    // Set compare value to 0xFFFF (wait 1 full cycle)
    SET_WORD(T1CC0H, T1CC0L, 0xFFFF);
    // Make sure compare value is written correctly
    GET_WORD(T1CC0H, T1CC0L, t1cc0);
    while (t1cc0 != 0xFFFF) 
    {
      SET_WORD(T1CC0H, T1CC0L, 0xFFFF);
      GET_WORD(T1CC0H, T1CC0L, t1cc0);
      if (repeat++ > 10) break;
    }
    // Decrease full cycle count
    sTimer1.nb_full_cycles--;
    
    // Clear Timer 1 Channel 0-2 + overflow interrupt flag
    T1CTL &= ~0xF0;
    // Enable IM after timer set
    T1CCTL0 |= BIT6;
  }
  else if (sTimer1.last_cycle_count > 5) //1) // Skip last 5 TCLK (~ 1 ACLK) - too close for IRQ
  {
    // Disable IM during timer set
    T1CCTL0 &= ~BIT6;

    // Set compare value to last period lenght - 1
    SET_WORD(T1CC0H, T1CC0L, sTimer1.last_cycle_count-1);
    // Make sure compare value is written correctly
    GET_WORD(T1CC0H, T1CC0L, t1cc0);
    while (t1cc0 != sTimer1.last_cycle_count-1) 
    {
      SET_WORD(T1CC0H, T1CC0L, sTimer1.last_cycle_count-1);
      GET_WORD(T1CC0H, T1CC0L, t1cc0);
      if (repeat++ > 10) break;
    }
    // Clear last cycle count - next IRQ calls s/w handler
    sTimer1.last_cycle_count = 0;

    // Suspend USB service via Timer4 until TX is over
    sTimer1.usb_service_disable = 1;
    
    // Clear Timer 1 Channel 0-2 + overflow interrupt flag
    T1CTL &= ~0xF0;
    // Enable IM after timer set
    T1CCTL0 |= BIT6;
  }  
  else return (0); // no cycles could be set
  
  return (1); // cycles successfully set
  
}


// Set up Timer1 in compare mode to emulate 32kHz timer
void set_timer1(u16 aclk)
{
  //u32 total_ticks;
  
  // Disable IM during timer set
  T1CCTL0 &= ~BIT6;
  
  // Disable s/w timer int
  sTimer1.enable = 0;
  
  // Calculate number of full cycles  
  sTimer1.nb_full_cycles = 0;
  while(aclk >= ACLK_PER_FULL_CYCLE)
  {
    aclk -= ACLK_PER_FULL_CYCLE;
    sTimer1.nb_full_cycles++;
  }

  // remainder is last timer period
  sTimer1.last_cycle_count = (aclk * TCLK_SPEED_HZ) / ACLK_SPEED_HZ;

  // Set Timer1 registers
  set_timer1_cycles();
  
  // Clear Timer 1 Channel 0-2 + overflow interrupt flag
  T1CTL &= ~0xF0;
  // Enable IM after timer set
  T1CCTL0 |= BIT6;

  // Enable s/w timer int
  sTimer1.enable = 1;
}


void set_timer1_abs(u16 aclk)
{
  u16 timediff;
  
  // Calculate time distance between last compare time and next compare time
  if (aclk > sTimer1.aclk) 
    timediff = aclk - sTimer1.aclk;
  else if (aclk == sTimer1.aclk) 
    timediff = 0xFFFF;
  else
    timediff = (65536 - sTimer1.aclk) + aclk;

  // Keep current value for next delta calculation  
  sTimer1.aclk = aclk;

  // Set timer1 relative to current time  
  set_timer1(timediff);
}


void enable_timer1_irq(void)
{
  sTimer1.enable = 1;
}

void disable_timer1_irq(void)
{
  sTimer1.enable = 0;
}

void clear_timer1_irq(void)
{
  sTimer1.iflag = 0;
}
