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
// Test routines used for calibration during production test
// *************************************************************************************************


// *************************************************************************************************
// Include section
#include "project.h"
#include "rftest.h"
#include "timer1.h"


// *************************************************************************************************
// Defines section
#define RFTEST_SYNC_WORD		(0xCABA)
#define RFTEST_PACKET_LENGTH		(10u)
#define RFTEST_PACKET_COUNT		(10u)
#define RFTEST_OUTPUT_POWER		(0x5A)
#define RFTEST_OUTPUT_POWER_MAX		(0xC0)
#define RFTEST_ACLK_DEVIATION_MAX	(4u)
#define RFTEST_FREQEST_MIN		(-25)
#define RFTEST_FREQEST_MAX		(+25)


// *************************************************************************************************
// Global Variable section
u8 	rftest_packet[RFTEST_PACKET_LENGTH];
u8      rftest_packet_ptr;
u8 	rftest_count;
u32     rftest_time;

typedef struct {
	u8    valid;
	u16   time;
	u8    packet_nb;
	u8    freqoffset;
} s_rftest;

s_rftest rftest[RFTEST_PACKET_COUNT];
		
const u8 ref_packet[RFTEST_PACKET_LENGTH] = { 0x00, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55 };



// *************************************************************************************************
// Extern section
extern u16 test_result;


// *************************************************************************************************
// Static Function prototype section

void rftest_init(void);
void rftest_radio_init(void);


// *************************************************************************************************
// Implementation


// *************************************************************************************************
// Init test variables
// *************************************************************************************************
void rftest_init(void)
{
  u8 i;
  
  // Init and calibrate radio
  rftest_radio_init();
  
  // Clear RX variables
  rftest_count = 0;
  for (i=0; i<RFTEST_PACKET_COUNT; i++)
  {
          rftest[i].valid      = 0;
          rftest[i].time       = 0;
          rftest[i].packet_nb  = 0;
          rftest[i].freqoffset = 0;
  }
}


// *************************************************************************************************
// Test RF reception
// *************************************************************************************************
void test_rf(void) 
{
  u16 time1, time2, packet1, packet2, div;
  s16 ref, delta;
  u8 i;
  s16 freqest;
  u8 freqest1;
  s16 freqest0;
  u8 continue_test = YES;
  
  // Config radio for RX
  rftest_init();

  // Set timeout in 1.5 seconds
  reset_timer1();
  set_timer1(32768*1.5);
  T1CCTL0 = 0x44;
  T1CTL   = 0x02;

  // Clear RF interrupts
  S1CON &= ~(BIT1 | BIT0);  // Clear MCU interrupt flag 
  RFIF &= ~BIT4;            // Clear "receive/transmit done" interrupt 
  RFIM |= BIT4;             // Enable "receive/transmit done" interrupt

  // Enable IRQ
  INT_ENABLE(INUM_RFTXRX, INT_ON);
  INT_ENABLE(INUM_RF, INT_ON);
  INT_ENABLE(INUM_T1, INT_ON);
  
  // Clear test result variable
  test_result = 0;
  
  // Continue to receive packets until timeout
  while ((rftest_count < RFTEST_PACKET_COUNT) && (!sTimer1.iflag))
  {
    // Start new RX if radio is IDLE
    if (MARCSTATE == 0x01) SRX();
  }
  SIDLE();
  
  // Analyze received packets
  if (sTimer1.iflag || rftest_count < RFTEST_PACKET_COUNT/2)
  {
    // Timeout - no or not enough packets received
    test_result = 0x8000;
    continue_test = NO;
  }
  else
  {
      // Store last and first packet
      for (i=0; i<RFTEST_PACKET_COUNT; i++)
      {
        if (rftest[i].valid) 
        {
          time1   = rftest[i].time;
          packet1 = rftest[i].packet_nb;
          break;
        }
      }
      for (i=RFTEST_PACKET_COUNT-1; i>0; i--)
      {
        if (rftest[i].valid) 
        {
          time2   = rftest[i].time;
          packet2 = rftest[i].packet_nb;
          break;
        }
      }
      
      // Calculate mean freqest
      freqest = 0;
      div = 0;
      for (i=0; i<RFTEST_PACKET_COUNT; i++)
      {
        if (rftest[i].valid) 
        {
            // Sign extend negative numbers
            if ((rftest[i].freqoffset & BIT7) == BIT7) 
            {
              freqest += (s16)(0xFF00 | rftest[i].freqoffset);
            }
            else
            {
              freqest += (s16)(rftest[i].freqoffset);
            }
            div++;
        }
      }
      
      // FREQEST range check
      freqest0 = freqest/(s16)div;
      if ((freqest0 < RFTEST_FREQEST_MIN) || (freqest0 > RFTEST_FREQEST_MAX)) 
      {
              test_result = 0x2000 | (u8)freqest0;
              continue_test = NO;
      }
      else
      { // FREQEST range ok
      
        // Calculate distance between 1 and 2
        // TX sends packet each 3276 ACLK ticks (= 1145 T1CLK ticks)
        ref = (packet2 - packet1)*3276;
        delta = time2 - time1;
  
        if ((ref - delta) < RFTEST_ACLK_DEVIATION_MAX)
        {
          // Store ACLK deviation in test result
          test_result = ((ref - delta)&0x0F) << 8; 
          
          // Store average FREQEST
          if (freqest < 0)
          {
            freqest1 = (u8)((freqest*(-1))/div);
            freqest1 = ~freqest1 + 1;
            test_result |= freqest1;
          }
          else
          {
            test_result |= (u8)(freqest/div);
          } 
        }
        else if ((delta - ref) < RFTEST_ACLK_DEVIATION_MAX)
        {
          // Store ACLK deviation in test result
          test_result = ((delta - ref)&0x0F) << 8; 
          
          // Store average FREQEST
          if (freqest < 0)
          {
            freqest1 = (u8)((freqest*(-1))/div);
            freqest1 = ~freqest1 + 1;
            test_result |= freqest1;
          }
          else
          {
            test_result |= (u8)(freqest/div);
          } 			
        }
        else
        {
          // Too high ACLK deviation
          test_result = 0x4000;
          continue_test = NO;
        }
      }
  }
  
  // 2nd test stage - use RF offset to catch another few packets
  if (continue_test == YES)
  {
    rftest_count = 0;
    
    // Clear RX variables
    for (i=0; i<RFTEST_PACKET_COUNT; i++)
    {
            rftest[i].valid      = 0;
            rftest[i].time 	     = 0;
            rftest[i].packet_nb  = 0;
            rftest[i].freqoffset = 0;
    }
    
    // Set frequency offset
    FSCTRL0 = freqest0;
    
    // Set timeout in 0.5 seconds
    reset_timer1();
    set_timer1(32768*0.5);
    
    // Try to receive just 1 packet
    while ((rftest_count < 1) && (!sTimer1.iflag))
    {
      // Start new RX if radio is IDLE
      if (MARCSTATE == 0x01) SRX();
    }
    SIDLE();
    
    // Analyze received packets frequency offset - should be close to 0
    if (rftest_count > 0)
    {
            if (!((rftest[0].freqoffset >= 0xFE) || (rftest[0].freqoffset <= 0x02)))
            {
                    test_result = 0x1000;
            }
    }
    else
    {
            test_result = 0x1000;
    }
  }

  // Disable IRQ
  INT_ENABLE(INUM_RFTXRX, INT_OFF);
  INT_ENABLE(INUM_RF, INT_OFF);
  INT_ENABLE(INUM_T1, INT_OFF);

  // Stop Timer1 
  T1CTL  = 0x00;
  T1CNTL = 0x55;
  reset_timer1();
}


// *************************************************************************************************
// Sets the radio hardware to the required initial state.
// *************************************************************************************************
void rftest_radio_init(void)
{
  u8 FSCAL3_Register_u8;

  SYNC1       = RFTEST_SYNC_WORD >> 8;  /*  Sync word, high byte                                */
  SYNC0       = (u8) RFTEST_SYNC_WORD;  /*  Sync word, low byte                                 */
  PKTLEN      = RFTEST_PACKET_LENGTH;   /*  Packet length                                       */
  PKTCTRL1    = 0x00;               /*  Packet automation control                           */
  PKTCTRL0    = 0x00;               /*  Packet automation control                           */
  ADDR        = 0x00;               /*  Device address                                      */
  CHANNR      = 0x00;               /*  Channel number                                      */
  FSCTRL1     = 0x12;               /*  Frequency synthesizer control                       */
  FSCTRL0     = 0x00;               /*  Frequency synthesizer control                       */
#ifdef ISM_EU
  FREQ2       = 0x24;               /*  Frequency control word, high byte                   */
  FREQ1       = 0x2D;               /*  Frequency control word, middle byte                 */
  FREQ0       = 0x55;               /*  Frequency control word, low byte                    */
  MDMCFG4     = 0x3D;               /*  Modem configuration                                 */
  MDMCFG3     = 0x55;               /*  Modem configuration                                 */
  MDMCFG2     = 0x15;               /*  Modem configuration                                 */
  MDMCFG1     = 0x12;               /*  Modem configuration                                 */
  MDMCFG0     = 0x11;               /*  Modem configuration                                 */
#else
  #ifdef ISM_US
  FREQ2       = 0x26;               /*  Frequency control word, high byte                   */
  FREQ1       = 0x19;               /*  Frequency control word, middle byte                 */
  FREQ0       = 0x11;               /*  Frequency control word, low byte                    */
  MDMCFG4     = 0x3D;               /*  Modem configuration                                 */
  MDMCFG3     = 0x55;               /*  Modem configuration                                 */
  MDMCFG2     = 0x15;               /*  Modem configuration                                 */
  MDMCFG1     = 0x12;               /*  Modem configuration                                 */
  MDMCFG0     = 0x11;               /*  Modem configuration                                 */ 
  #else
    #ifdef ISM_LF
    FREQ2       = 0x12;               /*  Frequency control word, high byte                   */
    FREQ1       = 0x0A;               /*  Frequency control word, middle byte                 */
    FREQ0       = 0xAA;               /*  Frequency control word, low byte                    */
    MDMCFG4     = 0x3D;               /*  Modem configuration                                 */
    MDMCFG3     = 0x55;               /*  Modem configuration                                 */
    MDMCFG2     = 0x15;               /*  Modem configuration                                 */
    MDMCFG1     = 0x12;               /*  Modem configuration                                 */
    MDMCFG0     = 0x11;               /*  Modem configuration                                 */ 
    #else
      #error "No ISM band specified"
    #endif
  #endif
#endif
  DEVIATN     = 0x60;               /*  Modem deviation setting                             */
  MCSM2       = 0x07;               /*  Main Radio Control State Machine configuration      */
  MCSM1       = 0x02;               /*  Main Radio Control State Machine configuration      */
  MCSM0       = 0x18;               /*  Main Radio Control State Machine configuration      */
  FOCCFG      = 0x1D;               /*  Frequency Offset Compensation configuration         */
  BSCFG       = 0x1C;               /*  Bit Synchronization configuration                   */
  AGCCTRL2    = 0xC7;               /*  AGC control                                         */
  AGCCTRL1    = 0x10;               /*  AGC control                                         */
  AGCCTRL0    = 0xB2;               /*  AGC control                                         */
  FREND1      = 0xB6;               /*  Front end RX configuration                          */
  FREND0      = 0x10;               /*  Front end TX configuration                          */
  FSCAL3      = 0xEA;               /*  Frequency synthesizer calibration                   */
  FSCAL2      = 0x2A;               /*  Frequency synthesizer calibration                   */
  FSCAL1      = 0x00;               /*  Frequency synthesizer calibration                   */
  FSCAL0      = 0x1F;               /*  Frequency synthesizer calibration                   */
  IOCFG2      = 0x00;               /*  Radio Test Signal Configuration (P1_7)              */
  IOCFG1      = 0x00;               /*  Radio Test Signal Configuration (P1_6)              */
  IOCFG0      = 0x00;               /*  Radio Test Signal Configuration (P1_5)              */
  TEST1       = 0x31;

  // Read FSCAL3 register, set bits enabling charge pump calibration and write register again
  FSCAL3_Register_u8 = FSCAL3 | 0x20;
  FSCAL3 = FSCAL3_Register_u8;

  // Set output power
  PA_TABLE0 = RFTEST_OUTPUT_POWER; 

  // Start calibration manually
  SIDLE();
  SCAL();

  // Wait until calibration completed
  while(MARCSTATE != 0x01);
  FSCAL3 &= ~0x30;  
    
  // Enter powerdown mode
  SIDLE();
}


// *************************************************************************************************
// RF TX/RX IRQ providing data handling
// *************************************************************************************************
void rftest_RfTxRxIsr(void)
{
  u8 pass, i;
  u16 time, timeout;
  
  // First clear pointer to fifo
  rftest_packet_ptr = 0;

  // Read first byte
  rftest_packet[rftest_packet_ptr++] = RFD;
  
  // Load timeout counter  
  timeout = 400;

  // Wait until all 10 bytes have been received
  while ( (rftest_packet_ptr < RFTEST_PACKET_LENGTH)  && (timeout-- != 0) ) 
  {
    if ((TCON & BIT1) == BIT1)
    {
      // Read next bytes
      rftest_packet[rftest_packet_ptr++] = RFD;

      // Clear IRQ flag
      TCON &= ~BIT1;
    }
  }

  // Indicate that RX is over
  if (rftest_packet_ptr == RFTEST_PACKET_LENGTH) 
  {
    // Radio off during decoding
    SIDLE();
    
    // Store current 16-bit time
    time  = T1CNTL;
    time |= T1CNTH << 8;
    time = (sTimer1.cycles*11453) + (u16)(((u32)time*32768)/187500);
    
    // Check if packet is valid
    pass = 1;
    for (i=1; i<RFTEST_PACKET_LENGTH; i++)
    {
      if (rftest_packet[i] != ref_packet[i]) pass = 0;
    }
    
    // Continue if packet ok
    if (pass)
    {
      rftest[rftest_count].valid        = 1;
      rftest[rftest_count].time 	= time;
      rftest[rftest_count].packet_nb	= rftest_packet[0];
      rftest[rftest_count].freqoffset	= FREQEST;
      rftest_count++;
    }
  }
}


// *************************************************************************************************
// RF TX/RX general IRQ providing status handling
// *************************************************************************************************
void rftest_RfIsr(void)
{
  u8 rfif_reg = RFIF;
  
  // Clear CPU int flag
  S1CON &= ~0x03;
  
  // check status register
  if ((rfif_reg & BIT7) == BIT7)
  {
    // TX underflow error
    SIDLE();
  }
  else if ((rfif_reg & IRQ_RXOVF) == IRQ_RXOVF)
  {
    // RX overflow error
    SIDLE();
    while(MARCSTATE != 0x01);
    rftest_packet_ptr = 0;
  } 

  // Clear IRQ
  RFIF = 0x00;  
}


// *************************************************************************************************
// Continuous TX - used for spectrum analysis and high current check
// *************************************************************************************************
void start_continuous_tx(void)
{
  // Disable IRQ
  INT_ENABLE(INUM_RFTXRX, INT_OFF);
  INT_ENABLE(INUM_RF, INT_OFF);

#ifdef TX_SIMPLE_TEST
  CHANNR = 0;
#else
  // Send on far away channel
  CHANNR = 0xFF;
#endif
  
  // Max output power
  PA_TABLE0 = RFTEST_OUTPUT_POWER_MAX;
  
  // Packet length is 1
  PKTLEN = 1;
  
  // Write first byte to RFD register
  RFD = 0x55;
  
  // Transmit
  STX();  
}



// *************************************************************************************************
// Exit continuous TX
// *************************************************************************************************
void stop_continuous_tx(void)
{
  SIDLE();
}