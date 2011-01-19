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
// Flash write routines
// *************************************************************************************************


// *************************************************************************************************
// Include section
#include "project.h"
#include "flash.h"


// *************************************************************************************************
// Defines section



// *************************************************************************************************
// Global Variable section

// Reserve XDATA memory for code execution
__no_init u8 __xdata flash_write_code[32] @ FLASH_WRITE_XDATA_ADDR;

// Flash erase / write routines can be called
u8 flash_update_init = 0;


// *************************************************************************************************
// Static Function prototype section


// *************************************************************************************************
// Implementation



// *************************************************************************************************
// Modify MCU registers so that flash erase / write functions can be called
// *************************************************************************************************
void flash_start_update(void)
{
  // stop all IRQ except TIMER4 and USB
  IEN0 &= ~(BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0);
  IEN1 &= ~(              BIT3 | BIT2 | BIT1 | BIT0);
  IEN2 &= ~(BIT5 | BIT4 | BIT3 | BIT2 |        BIT0);
  
  // Set flash write timing
  FWT  = ( 0x22 >> CLKSPD );

  // Copy flash write code to XDATA code
  flash_write_code[0] = 0x75; // MOV FCTL, 0x02
  flash_write_code[1] = 0xAE;
  flash_write_code[2] = 0x02;

  flash_write_code[3] = 0x00; // NOP
  flash_write_code[4] = 0x00; // NOP

  flash_write_code[5] = 0x74; // MOV A, data_lo
  flash_write_code[6] = 0xFF;
  flash_write_code[7] = 0xF5; // MOV FWDATA, A
  flash_write_code[8] = 0xAF;

  flash_write_code[9] = 0x00; // NOP
  flash_write_code[10] = 0x00; // NOP

  flash_write_code[11] = 0x74; // MOV A, data_hi
  flash_write_code[12] = 0xFF;
  flash_write_code[13] = 0xF5; // MOV FWDATA, A
  flash_write_code[14] = 0xAF;

  flash_write_code[15] = 0x00; // NOP
  flash_write_code[16] = 0x00; // NOP
  
  flash_write_code[17] = 0xE5; // MOV A, FCTL
  flash_write_code[18] = 0xAE;
  flash_write_code[19] = 0x54; // loop: ANL A,0x40
  flash_write_code[20] = 0x40; 
  flash_write_code[21] = 0x70; // JNZ loop
  flash_write_code[22] = 0xFA; 

  flash_write_code[23] = 0x22;  // RET
  
  // Set init flag to prevent normal code execution
  flash_update_init = 1;
}


// *************************************************************************************************
// Read 8-bit byte from flash memory
// *************************************************************************************************
u8 flash_byte_read(u16 addr)
{
  u8 __code * pcode = (u8 __code *) addr;
  u8 data = pcode[0];
  
  return (data);
  
}


// *************************************************************************************************
// Read 16-bit word from flash memory
// *************************************************************************************************
u16 flash_word_read(u16 addr)
{
  u8 __code * pcode = (u8 __code *) addr;
  u16 data = (u16)(pcode[0] << 8);
  data += pcode[1];

  return (data);
  
}


// *************************************************************************************************
// Write 16-bit word to flash memory
// *************************************************************************************************
u8 flash_word_write(u16 addr, u16 data, u16 key, u8 verify)
{
  u16 readback;
  
  // Return immediately if wrong key was passed
  if (key != FLASH_EW_KEY) return (0);
  
  // Disable all interrupts
  INT_GLOBAL_ENABLE(INT_OFF);
  
  // Wait until flash write/erase busy bit is cleared
  while ((FCTL & BIT7) == BIT7) {}
  
  // Set address register to flash address in word units
  FADDRH = (u8) (addr >> 9);
  FADDRL = (u8) (addr >> 1);
  
  // Copy only flash write data to XDATA code
  flash_write_code[6] = (u8) (data >> 8);
  flash_write_code[12] = (u8) (data);
  
  // Branch to XDATA code
  asm("LCALL 0xFC00");
  
  // Enable all interrupts
  INT_GLOBAL_ENABLE(INT_ON);

  // Verify that word was written
  if (verify)
  {
    readback = flash_word_read(addr);
    if (readback != data) return (0);
  }
  
  return(1);
}



// *************************************************************************************************
// Erase 1kB page
// *************************************************************************************************
u8 page_erase(u8 page, u16 key, u8 verify) 
{
  u16 i;
  
  // Return immediately if wrong key was passed
  if (key != FLASH_EW_KEY) return (0);
  
  // Check if page address is valid
  if (page < FLASH_RW_PAGE_MIN || page > FLASH_RW_PAGE_MAX) return (0);
  
  // Wait until flash write/erase busy bit is cleared
  while ((FCTL & BIT7) == BIT7) {}

  // Disable all interrupts
  INT_GLOBAL_ENABLE(INT_OFF);
  
  // Store page in address register
  FADDRH = (page) << 1;
  FADDRL = 0x00; 

  // Set flash write timing
  FWT = ( 0x22 >> CLKSPD );
  
  // Start page erase
  FCTL = 0x01;        
  asm("nop");
  
  // Wait until flash write/erase busy bit is cleared
  while ((FCTL & BIT7) == BIT7) {}

  // Enable all interrupts
  INT_GLOBAL_ENABLE(INT_ON);
  
  // Verify that page content is all 0xFF
  if (verify)
  {
    for (i=0; i<FLASH_PAGE_SIZE; i++) 
    {
      if (flash_byte_read(page*FLASH_PAGE_SIZE + i) != 0xFF) return (0);
    }
  }
  
  return (1);
}



