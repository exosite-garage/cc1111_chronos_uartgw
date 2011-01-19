// *************************************************************************************************
//
//	Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
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
// Wireless Update function
// *************************************************************************************************

// *************************************************************************************************
// Include section
#include "wbsl.h"
#include "hal.h"
#include "ioCCxx10_bitdef.h"
#include <string.h>


// *************************************************************************************************
// Local Functions Define
u8 transmitDMA(unsigned char *src, u8 size);
void rxModeOff(void);
void rxModeOn(void);
void sendDataPacket(void);
void sendInitPacket(void);


// *************************************************************************************************
// Global Variable section

/* reserve space for the maximum possible peer Link IDs */
static unsigned char  wNumCurrentPeers;

// extern defines
volatile u8 wbsl_flag;
volatile u8 wbsl_packet_flag;
u8 wbsl_data[WBSL_MAX_PAYLOAD_LENGTH];
u8 wbsl_data_length = 0;
volatile u8 packet_ready_flag;
u8 TxBuffer[TX_SIZE];

u32 total_size_in_bytes;

TIMER_TABLE_ENTRY timeout_table[MAX_TIMEOUT_TABLE_SIZE];

volatile u8 rxtx_flag = 0;

// Handle number of ACKs sent for an indivudual packet
unsigned char wbsl_number_of_retries;

unsigned char RxBuffer[255];
unsigned char discoveryPayload[8] = {7,0,WBSL_AP_ADDRESS,0xBA,0x5E,0xBA,0x11,9};
unsigned char discoveryAck[4] = {3, 0, WBSL_AP_ADDRESS, 0};
unsigned char initPacket[] = {5,0,WBSL_AP_ADDRESS,0,0,0};
unsigned char wbsl_status;
unsigned char wbsl_rftxif = 0;
unsigned char wbsl_irq_done = 0;
unsigned char wbsl_txfifo_filled = 0;
unsigned char update_complete       = 0;
unsigned char wbsl_rxtxMode = 0;
unsigned char ed_address;
u16 watchVoltage;


//Settings Structure for the RX/TX DMA Channel
DMA_DESC dmachs;

// Store the total number of packets to be sent to the Watch
//u16 totalPackets = 0;
unsigned int total_packets;
// Keep track of which packet needs to be sent to the Watch
u16 currentPacket = 0;
// Variable to see if the Init Packet has been successfully sent
u8  initOk = 0;



// *************************************************************************************************
// @fn          wbsl_config
// @brief       Configures the Radio Settings for WBSL
// @param       none
// @return      none
// *************************************************************************************************
void wbsl_config(void)
{
 
   // WBSL will set the ones needed
   RFIM = 0;
   RFIF = 0;                               // Clear Interrupts
    
   // OSC=XT, TICKSPD=fref/128, CLKSPD=24MHz
   CLKCON = BIT7 | BIT5 | BIT4 | BIT3; 

   //Set the SYNC words to be used
   SYNC1      =   0xD3; // SYNC1: high nyte of Sync Word
   SYNC0      =   0x91;  // SYNC0: low nyte of Sync Word
   //Setup radio
   FSCTRL1     = 0x12;   // Frequency synthesizer control.
   FSCTRL0     = 0x00;   // Frequency synthesizer control.
#ifdef ISM_EU
   // 869.525MHz
   FREQ2       = 0x24;               //  Frequency control word, high byte
   FREQ1       = 0x3A;               //  Frequency control word, middle byte
   FREQ0       = 0xEE;               //  Frequency control word, low byte
   CHANNR      = 0;                  // Channel number.
   PA_TABLE0   = 0x8C;               // PA output power setting.
#else
#ifdef ISM_US
   // 902MHz (CHANNR=20 --> 906MHz)
   FREQ2       = 0x25;               //  Frequency control word, high byte
   FREQ1       = 0x95;               //  Frequency control word, middle byte
   FREQ0       = 0x55;               //  Frequency control word, low byte
   CHANNR      = 20;                // Channel number.
   PA_TABLE0   = 0x8B;              // PA output power setting.
#else
#ifdef ISM_LF
  // 433.30MHz
  FREQ2 = 0x12;                     //  Frequency control word, high byte
  FREQ1 = 0x14;                     //  Frequency control word, middle byte
  FREQ0 = 0x7A;                     //  Frequency control word, low byte
  CHANNR = 0;                       // Channel number.
  PA_TABLE0 = 0x8D;                 // PA output power setting.
#else
  #error "Wrong ISM band specified (valid are ISM_LF, ISM_EU and ISM_US)"
#endif // ISM_LF
#endif // ISM_US
#endif // ISM_EU
   MDMCFG4     = 0x1D;               // Modem configuration.
   MDMCFG3     = 0x55;               // Modem configuration.
   MDMCFG2     = 0x13;               // Modem configuration.
   MDMCFG1     = 0x23;               // Modem configuration.
   MDMCFG0     = 0x11;               // Modem configuration.
   
   DEVIATN   = 0x63;   // Modem deviation setting (when FSK modulation is enabled).
   FREND1    = 0xB6;   // Front end RX configuration.
   FREND0    = 0x10;   // Front end TX configuration.
   MCSM0     = 0x18;   // Main Radio Control State Machine configuration.
   MCSM1     = 0x3C;
   MCSM2     = 0x07;
   WOREVT1   = 0x87;
   WOREVT0   = 0x6B;
   WORCTL    = 0xF8;
   FOCCFG    = 0x1D;   // Frequency Offset Compensation Configuration.
   BSCFG     = 0x1C;   // Bit synchronization Configuration.
   AGCCTRL2  = 0xC7;   // AGC control.
   AGCCTRL1  = 0x00;   // AGC control.
   AGCCTRL0  = 0xB0;   // AGC control.
   FSCAL3    = 0xEA;   // Frequency synthesizer calibration.
   FSCAL2    = 0x2A;   // Frequency synthesizer calibration.
   FSCAL1    = 0x00;   // Frequency synthesizer calibration.
   FSCAL0    = 0x1F;   // Frequency synthesizer calibration.
   FSTEST    = 0x59;
   TEST2     = 0x88;   // Various test settings.
   TEST1     = 0x31;   // Various test settings.
   TEST0     = 0x09;   // Various test settings.
   PTEST     = 0x7F;
   AGCTEST   = 0x88;
   IOCFG2    = 0x29;
   IOCFG1    = 0x1E;
   IOCFG0    = 0x1B;
   
   PKTCTRL1  = 0x06;   // Packet automation control.
   PKTCTRL0  = 0x45;   // Packet automation control.
   ADDR      = WBSL_AP_ADDRESS;   // Device address.
   PKTLEN    = 0xFE;   // Packet length.
}

// *************************************************************************************************
// @fn          wbsl_RfIsr
// @brief       called when the a packet has been received, the actual packet handling is done
//              by the DMA. In this function we check for RX Overflow and set the corresponding
//              flags to inform the application a packet is ready on the RxBuffer
// @param       none
// @return      none
// *************************************************************************************************
void wbsl_RfIsr(void)
{
   u8 rfif_reg = RFIF;
  
   // We should only be here in RX mode, not in TX mode, nor if RX mode was turned on during CCA
   if(wbsl_rxtxMode != WBSL_RX_MODE)
   {
      RFIF = 0;
      return;
   }

   // Check for RX Overflow
   if((rfif_reg & IRQ_DONE) && (rfif_reg & IRQ_RXOVF))
   {
      RFIF &= ~IRQ_DONE;
      RFIF &= ~IRQ_RXOVF;
      S1CON &= ~(0x03); /* Clear MCU interrupt flag */

      // Go to IDLE state to get out of overflow
      rxModeOff();

      /* clean out buffer to help protect against spurious frames */
      memset(RxBuffer, 0x00, sizeof(RxBuffer));

      // Start again RX Mode
      rxModeOn();

      return;
   }

   RFIF &= IRQ_DONE;  // Clear the RF interrupt
   S1CON &= ~(0x03); // Clear CPU int flag
   
   rxtx_flag = WBSL_RXTX_RECEIVED; //Signal the application that a packet is in RxBuffer
}

// *************************************************************************************************
// @fn          resetTimer
// @brief       resets the timeout table, so that the timer interrupt does not check
//              the timeout values
// @param       u8 index        index of the timer in the timeout_table you want to use
// @return      none
// *************************************************************************************************
void resetTimer(u8 index)
{
   // Check that the index doesn't exceed the number of timeouts we have
   if(index >= MAX_TIMEOUT_TABLE_SIZE)
      return;
   
   timeout_table[index].counter = 0;    // Reset counter
   timeout_table[index].timeout = 0;    // Reset timeout (Disable)
   timeout_table[index].flag = 0;       // Reset Flag
}

// *************************************************************************************************
// @fn          setTimer
// @brief       sets the timeout table values, so that the timer interrupt checks
//              these values and sets the timeout flag accordingly
// @param       u8 index        index of the timer in the timeout_table you want to use
//              u16 timeout     number of interrupts that have to happen before we set the timeout flag
// @return      none
// *************************************************************************************************
void setTimer(unsigned char index, unsigned int timeout)
{
   // Check that the index doesn't exceed the number of timeouts we have
   if(index >= MAX_TIMEOUT_TABLE_SIZE)
      return;
   
   timeout_table[index].counter = 0;    // Reset counter
   timeout_table[index].flag = 0;       // Reset Flag
   timeout_table[index].timeout = timeout;   // Set timeout
}

// *************************************************************************************************
// @fn          rxModeOn
// @brief       Puts the radio in RX Mode, sets all the needed values to activate the DMA
//              and enables the needed interrupts
// @param       none
// @return      none
// *************************************************************************************************
void rxModeOn(void)
{
   u8 *dstAdd  = RxBuffer;
   DMA_DESC *dmach;
   dmach = (DMA_DESC *)(&dmachs);
   
   // Configure the DMA to move a packet from the RFD register to the RxBuffer
   SET_WORD(dmach->DESTADDRH, dmach->DESTADDRL, (u16)dstAdd);
   SET_WORD(dmach->SRCADDRH, dmach->SRCADDRL, (u16)&X_RFD);    // Set the source address to RFD to get Data from the RF
   SET_WORD(dmach->LENH, dmach->LENL, 0x0100);
   dmach->VLEN = VLEN_1_P_VALOFFIRST_P_2;                   // Use first byte for transfer count + 2 more status bytes
   dmach->PRIORITY = PRI_HIGH;                              // High Priority.
   dmach->M8 = M8_USE_8_BITS;                               // Transfer all 8 bits in each byte
   dmach->IRQMASK = IRQMASK_DISABLE;                        // Don't generate an interrupt when DMA is done
   dmach->DESTINC = DESTINC_1;                              // Increment the Destination address 1 byte each transfer
   dmach->SRCINC = SRCINC_0;                                // source Address is constant
   dmach->TRIG = DMATRIG_RADIO;                             // RF packet byte received/transmit
   dmach->TMODE = TMODE_SINGLE;                              // Transfer block of data (length len) after each DMA trigger
   dmach->WORDSIZE = WORDSIZE_BYTE;                         // Transfer a byte at a time

   /* abort any DMA transfer that might be in progress */
   DMAARM = DMA_ABORT | BM( WBSL_DMA_CHAN );

   /* clean out buffer to help protect against spurious frames */
   memset(RxBuffer, 0x00, sizeof(RxBuffer));

   DMAARM |= BM( WBSL_DMA_CHAN );

   /* Clear interrupts */
   S1CON &= ~(0x03); /* Clear MCU interrupt flag */
   RFIF &= ~IRQ_DONE;           /* Clear the interrupt at the source */

   //Set Mode to RX
   wbsl_rxtxMode = WBSL_RX_MODE;
   SRX();  // Strobe RX Mode

   /* enable "receive/transmit done" interrupts */
   RFIM |= IRQ_DONE;
}

// *************************************************************************************************
// @fn          rxModeOff
// @brief       Puts the radio in Idle Mode, disables the DMA and RF Interrupts
// @param       none
// @return      none
// *************************************************************************************************
void rxModeOff(void)
{
   /*disable RF receive interrupts */
   RFIM &= ~IRQ_DONE;

   //Go to RF Idle mode
   wbsl_rxtxMode = WBSL_IDLE_MODE;
   SIDLE();
   while(MARCSTATE != MARC_STATE_IDLE);

   /* Abort any ongoing DMA transfer */
   DMAARM = DMA_ABORT | BM( WBSL_DMA_CHAN );

   /* Clear any pending DMA interrupts */
   DMAIRQ &= ~BM(WBSL_DMA_CHAN);

   /* Clear interrupts */
   S1CON &= ~(0x03);            /* Clear MCU interrupt flag */
   RFIF &= ~IRQ_DONE;           /* Clear the interrupt at the source */
}

// *************************************************************************************************
// @fn          transmitDMA
// @brief       Sets all the needed values to activate the DMA transfer and puts the Radio in Tx Mode
//              to transmit the Buffer received, either in CCA Mode or in Force Mode.
// @param       u8* src        The Buffer which is going to be sent through RF
//              u8 size        The size of the buffer received
// @return      none
// *************************************************************************************************
u8 transmitDMA(u8 *src, u8 size){
  
   u8 retValue = WBSL_TX_RES_SUCCESS;
#ifdef CCA_MODE
   u8 ccaRetries;
  // u16 backOffDelay;
#endif 
   
   DMA_DESC *dmach;
   dmach = (DMA_DESC *)(&dmachs);
  
   /* Turn off reciever. We can ignore/drop incoming packets during transmit. */
   rxModeOff();
   
   // Configure DMA to transfer from the SOURCE to the RFD register
   SET_WORD(dmach->DESTADDRH, dmach->DESTADDRL, (u16)&X_RFD);
   SET_WORD(dmach->SRCADDRH, dmach->SRCADDRL, (u16)src);    // Set the source address to TxBuffer for testing
   SET_WORD(dmach->LENH, dmach->LENL, 0x0100);
   dmach->VLEN = VLEN_1_P_VALOFFIRST;                       // Use first byte for transfer count N + 1
   dmach->PRIORITY = PRI_HIGH;                              // High Priority.
   dmach->M8 = M8_USE_8_BITS;                               // Transfer all 8 bits in each byte
   dmach->IRQMASK = IRQMASK_DISABLE;                        // Don't generate an interrupt when DMA is done
   dmach->DESTINC = DESTINC_0;                              // Destination Address is constant
   dmach->SRCINC = SRCINC_1;                                // Increment the source address 1 byte each transfer
   dmach->TRIG = DMATRIG_RADIO;                             // RF packet byte received/transmit
   dmach->TMODE = TMODE_SINGLE;                              // Transfer block of data (length len) after each DMA trigger
   dmach->WORDSIZE = WORDSIZE_BYTE;                         // Transfer a byte at a time

#ifdef CCA_MODE
   ccaRetries = 4;
  
   /* ===============================================================================
    *    CCA Loop
    *  =============
    */
   while(1)
   {
      // Arm the DMA Channel used for the WBSL
      DMAARM |= BM( WBSL_DMA_CHAN );
       
      //Strobe RX to sense air for CCA
      SRX();
        
      /* Wait for radio to enter the RX mode */
      while(MARCSTATE != MARC_STATE_RX);
        
      /* Check RSSI */
      {
         u16 delay = 30 * 16 * 12;
         do{
            if(PKTSTATUS & (CCA_FLAG | CS_FLAG))
            {
               break;
            }
            delay -= 16;
         }while(delay > 0);
      }
      /*End of RSSI Check */
        
      // Strobe TX to send the message
      STX();
        
      if(MARCSTATE != MARC_STATE_RX)
      {
         // Clear Channel Assessment Passed
         /* wait for transmit to complete */
         while(!(RFIF & IRQ_DONE));
            
         /* Clear the interrupt flag */
         RFIF &= ~IRQ_DONE;
            
         break;
      }
      else
      {
         // Clear Channel Assessment Failed
         if(ccaRetries != 0)
         {
            // Go to idle during Backoff
            SIDLE();
            while(MARCSTATE != MARC_STATE_IDLE);
              
            //backOffDelay = 10000;
            //while(--backOffDelay > 0);
            // Back off delay
            setTimer(WBSL_TIMEOUT_INDEX,1);
            // Wait for either the Timeout or packet received flag
            while(!timeout_table[WBSL_TIMEOUT_INDEX].flag);
            // Reset timer so that next time it starts fresh
            resetTimer(WBSL_TIMEOUT_INDEX);
              
            ccaRetries--;         
         }
         else /* No CCA retries are left, abort */
         {
            /* set return value for failed transmit and break */
            return;
         }   
            
      }
   }
#else
   // Arm the DMA Channel used for the WBSL
   DMAARM |= BM( WBSL_DMA_CHAN );
   //Set Mode to TX
   wbsl_rxtxMode = WBSL_TX_MODE;

   STX();  // Strobe TX Mode

   /* wait for transmit to complete */
   while(!(RFIF & IRQ_DONE));

   /* Clear the interrupt flag */
   RFIF &= ~IRQ_DONE;
#endif
   // Go to IDLE
   rxModeOff();

   return (retValue);
}

// *************************************************************************************************
// @fn          wbsl_link
// @brief       Checks if a discovery packet has been received, checks the integrity of it
//              and sends the discovery ACK
// @param       none
// @return      u8 status
//              WBSL_LINK_FAIL    No discovery packet was received
//              WBSL_LINK_SUCC    Discovery packet received and ACK sent
// *************************************************************************************************
u8 wbsl_link(void)
{
   u8 status = WBSL_LINK_FAIL;
   u8 i = 0;
   u8 validPacket = 0;
   //u32 timeout = 0;
   u8 crcOk = 0;

   //Check if packet was received
   if(rxtx_flag == WBSL_RXTX_RECEIVED)
   {
      // Clear RX Flag
      rxtx_flag = 0;

      // Get CRC Status, which is appended in the last bit of the 2nd byte of the status bytes
      crcOk = RxBuffer[RxBuffer[0] + WBSL_CRC_STATUS_OFFSET] & CRC_STATUS;

      //Check that packet was a broadcast packet and that the CRC Status is Ok
      if((RxBuffer[AP_ADDRESS_OFFSET_RX] == 0x0) && crcOk)
      {

         // Check if packet has the discovery Payload
         validPacket = 1;
         for (i=0; i<DISCOVERY_PAYLOAD_LENGTH; i++)
         {
            if (RxBuffer[i+4] != discoveryPayload[i+3]) validPacket = 0;
         }

         //If packet O.K., save device address and send ACK
         if(validPacket)
         {
            // Save Watch Address to later direct all packets to this watch
            ed_address = RxBuffer[ED_ADDRESS_OFFSET_RX];

            // Address the discovery packet to the Watch
            discoveryAck[ED_ADDRESS_OFFSET_TX] = ed_address;

            // Positive ACK of the discovery
            discoveryAck[DISCOVERY_ACK_OFFSET] = WBSL_LINK_SUCC;

            //timeout = 10000;
            //while(timeout-- > 0);  
            // Give time to watch to be on RX Mode
            setTimer(WBSL_TIMEOUT_INDEX,1);
            // Wait for either the Timeout or packet received flag
            while(!timeout_table[WBSL_TIMEOUT_INDEX].flag);
            // Reset timer so that next time it starts fresh
            resetTimer(WBSL_TIMEOUT_INDEX);
            

            LED_ON;
            transmitDMA(discoveryAck, sizeof(discoveryAck)); // Send discovery ACK to Watch
            LED_OFF;

            status = WBSL_LINK_SUCC;
            wbsl_flag = WBSL_STATUS_LINKED;
         }
      }
   }

   return status;
}

// *************************************************************************************************
// @fn          sendInitPacket
// @brief       Send the Init packet to the Watch, which contains the total packets to be received
//              during the communication, it also waits for the ACK of the Init packet, if it is not
//              received this function will be called again by the main process
// @param       none
// @return      none
// *************************************************************************************************
void sendInitPacket(void)
{
   u8 crcOk = 0;
   // Initialize the INIT packet to be sent to the newly paired device
   // initPacket[INIT_TOTAL_PACKETS_OFFSET] = totalPackets;
   initPacket[ED_ADDRESS_OFFSET_TX] = ed_address;
   // Test REMOVE
   initPacket[5] = wbsl_number_of_retries;
     
   setTimer(WBSL_TIMEOUT_INDEX,TIMEOUT_FOR_ACK);
   
   //Wait until packet is ready to be sent or timeout
   while(packet_ready_flag != WBSL_PACKET_FULL && !timeout_table[WBSL_TIMEOUT_INDEX].flag );
    
   /* Check if the timeout happened before the packet was full,
    * if so, trigger the update to stop, there is an error in communication between 
    * GUI and Dongle has ocurred or a manual stop has been triggered
    */
   if(packet_ready_flag != WBSL_PACKET_FULL)
   {
      setFlag(wbsl_flag, WBSL_TRIGGER_STOP);
      return;
   }
   // Reset timer so that next time it starts fresh
   resetTimer(WBSL_TIMEOUT_INDEX);
     
   LED_ON;                                                  // Mark Initiate Transmit
   transmitDMA(initPacket, sizeof( initPacket));
   LED_OFF;                                               //Turn off led after packet sent
   rxModeOn();              // Turn on RX Mode to wait for the Discovery ACK package

   // Set timeout to receive
   setTimer(WBSL_TIMEOUT_INDEX,TIMEOUT_FOR_ACK);
   // Wait for either the Timeout or packet received flag
   while(!timeout_table[WBSL_TIMEOUT_INDEX].flag && rxtx_flag != WBSL_RXTX_RECEIVED);

   // Reset timer so that next time it starts fresh
   resetTimer(WBSL_TIMEOUT_INDEX);

   // Radio off during decoding
   rxModeOff();

    //Check if packet was received
   if(rxtx_flag == WBSL_RXTX_RECEIVED)
   {
      // Get CRC Status, which is appended in the last bit of the 2nd byte of the status bytes
      crcOk = RxBuffer[RxBuffer[0] + WBSL_CRC_STATUS_OFFSET] & CRC_STATUS;
      // Clear RX Flag
      rxtx_flag = 0;
      if(RxBuffer[AP_ADDRESS_OFFSET_RX] == WBSL_AP_ADDRESS &&
         RxBuffer[ED_ADDRESS_OFFSET_RX] == ed_address &&
         RxBuffer[ED_ADDRESS_OFFSET_RX + 1] == WBSL_ACK_SUCC &&
         crcOk)
      {
         // Reset the retry counter for ACKs
         wbsl_number_of_retries = 0;
         initOk = 1;
            
         // Flag that the buffer is ready to be filled again
         packet_ready_flag = WBSL_PACKET_EMPTY;
         // Trigger GUI to send first data packet
         wbsl_packet_flag = WBSL_SEND_NEW_DATA_PACKET;
      }
   }
}

// *************************************************************************************************
// @fn          sendDataPacket
// @brief       Send the next data packet to the watch and waits for the ACK, if received it triggers
//              the GUI to send the next packet to be sent to the watch, if not, the same packet will
//              be sent again the next time this function is called
// @param       none
// @return      none
// *************************************************************************************************
void sendDataPacket(void)
{
   u8 crcOk = 0;
   // Change the needed fields for the TX Buffer
   TxBuffer[CURRENT_PACKET_NR_OFFSET] = (currentPacket >> 8) & 0xFF;
   TxBuffer[CURRENT_PACKET_NR_OFFSET + 1] = currentPacket & 0xFF;
    
   // Address the packet to the Watch we're synched to
   TxBuffer[ED_ADDRESS_OFFSET_TX] = ed_address;
   
   setTimer(WBSL_TIMEOUT_INDEX,TIMEOUT_FOR_ACK);
    
   //Wait until packet is ready to be sent or timeout
   while(packet_ready_flag != WBSL_PACKET_FULL && !timeout_table[WBSL_TIMEOUT_INDEX].flag );
    
   /* Check if the timeout happened before the packet was full,
    * if so, trigger the update to stop, there is an error in communication between 
    * GUI and Dongle has ocurred or a manual stop has been triggered
    */
   if(packet_ready_flag != WBSL_PACKET_FULL)
   {
      setFlag(wbsl_flag, WBSL_TRIGGER_STOP);
      return;
   }
   // Reset timer so that next time it starts fresh
   resetTimer(WBSL_TIMEOUT_INDEX);
   
   // Send packet to ED
   LED_ON;
   transmitDMA(TxBuffer,TxBuffer[0]+1);
   LED_OFF;     //Transmit Complete

   rxModeOn();              // Turn on RX Mode to wait for the ACK package

   // Set timeout to receive ACK
   setTimer(WBSL_TIMEOUT_INDEX,TIMEOUT_FOR_ACK);
   // Wait for either the Timeout or packet received flag
   while(!timeout_table[WBSL_TIMEOUT_INDEX].flag && rxtx_flag != WBSL_RXTX_RECEIVED);

   // Radio off during decoding
   rxModeOff();

   // Reset timer so that next time it starts fresh
   resetTimer(WBSL_TIMEOUT_INDEX);

   //Check if packet was received and check for status fields
   if(rxtx_flag == WBSL_RXTX_RECEIVED)
   {
      // Get CRC Status, which is appended in the last bit of the 2nd byte of the status bytes
      crcOk = RxBuffer[RxBuffer[0] + WBSL_CRC_STATUS_OFFSET] & CRC_STATUS;

      // Clear RX Flag
      rxtx_flag = 0;
      if(RxBuffer[AP_ADDRESS_OFFSET_RX] == WBSL_AP_ADDRESS &&
         RxBuffer[ED_ADDRESS_OFFSET_RX] == ed_address &&
         RxBuffer[ED_ADDRESS_OFFSET_RX + 1] == WBSL_ACK_SUCC &&
         RxBuffer[ED_ADDRESS_OFFSET_RX + 2] == ((currentPacket >> 8) &  0x7F) &&
         RxBuffer[ED_ADDRESS_OFFSET_RX + 3] ==  (currentPacket & 0xFF) &&
         crcOk)
      {
         // Reset the retry counter for ACKs
         wbsl_number_of_retries = 0;
         currentPacket++;
         wbsl_status = (currentPacket*100)/total_packets;
         // Trigger GUI to send next data packet
         wbsl_packet_flag = WBSL_SEND_NEW_DATA_PACKET;

         // Flag that the buffer is ready to be filled again
         packet_ready_flag = WBSL_PACKET_EMPTY;
      }
   }
}

// *************************************************************************************************
// @fn          wbsl_sendPacket
// @brief       This function handles the sending of the different packets that conform the whole
//              firmware download, it checks if a packet is ready to be sent and sends it, it also waits
//              for the ACK of the packet or a timeout, in the latter case when called again, it will send
//              the same packet.
// @param       none
// @return      none
// *************************************************************************************************
void wbsl_sendPacket(void)
{
   // Increment the number of retries to send a packet
   wbsl_number_of_retries++;
    
   // Check if too many retries for one packet have been already made
   if(wbsl_number_of_retries > WBSL_MAXIMUM_RETRIES)
   {
      // Trigger the stop of the WBSL Update procedure
      setFlag(wbsl_flag, WBSL_TRIGGER_STOP);
      setFlag(wbsl_flag, WBSL_STATUS_ERROR);
      return;
   }
   
   if(!initOk)
   {
      // Send the init packet
      sendInitPacket();
   }
   else if(currentPacket < total_packets)
   {
      // Send a regular data packet
      sendDataPacket();
   }
}

// *************************************************************************************************
// @fn          wbsl_reset
// @brief       Reset variables needed for the WBSL
// @param       none
// @return      none
// *************************************************************************************************
void wbsl_reset(void)
{
   wbsl_number_of_retries = 0;
   wbsl_status = 0;
   wbsl_flag = WBSL_STATUS_LINKING;
   // Initialize the packet status flag so that GUI knows when it needs to send next packet
   wbsl_packet_flag = WBSL_DISABLED;  
   currentPacket = 0;
   total_packets = 0;
   packet_ready_flag = WBSL_PACKET_EMPTY;
   initOk = 0;
   //Initialize the RX Flag
   rxtx_flag = 0;
}

// *************************************************************************************************
// @fn          wbsl_main
// @brief       This is the main routine, which calls the Link function, if succesfull, it keeps calling
//              the sendPacket function until all packages has been sent, it also checks for the wbsl_flag
//              in case the procedure is stopped before completing the download, either manually or due to an 
//              error
// @param       none
// @return      none
// *************************************************************************************************
void wbsl_main(void)
{
   //u32 txDelay;

   // Assign DMA Configuration Struct to the DMA Channel 0
   DMA0CFGH = HIBYTE( &dmachs );
   DMA0CFGL = LOBYTE( &dmachs );
    
   // Initialize used variables
   wbsl_reset();
    
   //Init TX Buffer
   TxBuffer[0] = sizeof(TxBuffer) - 1;  // Substract the length field (1 byte)
   TxBuffer[AP_ADDRESS_OFFSET_TX] = WBSL_AP_ADDRESS; // Include my address in all packets

   //Update WBSL Flag
   wbsl_flag = WBSL_STATUS_LINKING;

   //Enable/Disable all the needed RF interrupts
   INT_ENABLE(INUM_RFTXRX, INT_OFF);        // Disable RXTX Interrupt
   INT_ENABLE(INUM_RF, INT_ON);            // Enable General RF Interrupt

   LED_OFF;

   wNumCurrentPeers = 0;

   wbsl_status = 0;
    
   // Make sure radio is in IDLE Mode before starting
   rxModeOff();

   while(1)
   {
      if(!wNumCurrentPeers)
      {
         rxModeOn();              // Turn on RX Mode to wait for the Discovery package
         //First try pairing with an End Device
         while(1)
         {
            // Try to link with an end device
            if(wbsl_link() == WBSL_LINK_SUCC)
            {
               wbsl_flag = WBSL_STATUS_LINKED;
               //Keep track of how many peers are connected
               wNumCurrentPeers++;
               // Trigger GUI to send first info packet (total bytes of file)
               wbsl_packet_flag = WBSL_SEND_INFO_PACKET;
               rxModeOff();              // Turn off RX Mode after succesfully linking with ED
               break;
            }

            // If GUI has triggered to stop WBSL, do so
            if(getFlag(wbsl_flag,WBSL_TRIGGER_STOP))
            {
               break;
            }
         }
      }
      
      // If GUI or Watch has triggered to stop WBSL, do so, but leave everything
      // in a workable state.
      if(getFlag(wbsl_flag,WBSL_TRIGGER_STOP))
      {
         // Immediately turn off RF interrupts
         INT_ENABLE(INUM_RFTXRX, INT_OFF);
         INT_ENABLE(INUM_RF, INT_OFF);
         RFIM &= ~IRQ_DONE;                       // Disable Packet Received/Transmitted Interrupt
         RFIF = 0;                               // Clear Interrupts
         //Put radio in Idle mode
         rxModeOff();
          
         break;
      }
      
      //Keep downloading update image
      if (currentPacket < total_packets || !initOk)
      {  
        // txDelay = 6000;
        // while(txDelay-->0);
         wbsl_sendPacket();    // Send the packet
         
         // Delay to give time to watch to be in RX after sending ACK
         setTimer(WBSL_TIMEOUT_INDEX,1);
         // Wait for either the Timeout or packet received flag
         while(!timeout_table[WBSL_TIMEOUT_INDEX].flag);
         // Reset timer so that next time it starts fresh
         resetTimer(WBSL_TIMEOUT_INDEX);  
      }

      //If the update has complete, finish the Transmission and turn off WBSL
      if(update_complete || currentPacket >= total_packets)
      {
         setFlag(wbsl_flag, WBSL_TRIGGER_STOP);
      }
   }
    
   wNumCurrentPeers = 0;
   // Set timeout to receive ACK
   // This is to allow the GUI to read the last progress status in case it reached 100
   setTimer(WBSL_TIMEOUT_INDEX,300);
   while(!timeout_table[WBSL_TIMEOUT_INDEX].flag);
   // Reset timer before exiting
   resetTimer(WBSL_TIMEOUT_INDEX);
}