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
// Main function
// *************************************************************************************************


// *************************************************************************************************
// Include section
#include "project.h"
#include "timer1.h"
#include "BM_API.h"
#include "simpliciti.h"
#include "rftest.h"
#include "flash.h"
#include "wbsl.h"

#include "hal_types.h"
#include "usb_uart.h"


// *************************************************************************************************
// Defines section

// Service USB process directly during initial enumeration
#define ENUMERATION_TIME    (20000u)


// *************************************************************************************************
// Global Variable section

// Normal operation
unsigned char system_status        = HW_IDLE;
unsigned char simpliciti_on        = 0;
unsigned char bluerobin_on         = 0;
unsigned char wbsl_on              = 0;
unsigned char bluerobin_start_now  = 0;
unsigned char simpliciti_start_now = 0;
unsigned char wbsl_start_now       = 0;

// SimpliciTI Sync variables
u8 simpliciti_sync_buffer[BM_SYNC_DATA_LENGTH];
u8 simpliciti_sync_buffer_status;

// WBSL
u8 ptrTX;
u8 wbsl_size;
u8 wbsl_inc;
u8 current_packet_size;
u8 wbsl_OPCode;
u8 wbsl_packetLength;

// Test
u8  test_on        = 0;
u8  test_step      = 0;
u8  test_step_over = 0;
u16 test_result    = 0;
u8  frequoffset    = 0;


// *************************************************************************************************
// Extern variable section
extern u8 HeartRate_u8;   // BlueRobin heartrate to transmit
extern u8 Speed_u8;       // BlueRobin speed to transmit
extern u16 Distance_u16;  // BlueRobin distance to transmit
extern u8 rf_tx_over;     // Flag to indicate last byte has been sent
extern u8 BRTX__SC_u8;    // BlueRobin sequence counter
extern u32 BRTX__ID_u32;  // BlueRobin TX ID
extern void BR_RfTxRxIsr(void); // BlueRobin RF TXRX ISR
extern unsigned char usb_buffer[USB_MAX_MESSAGE_LENGTH+2];
extern unsigned char usb_sendack;


// *************************************************************************************************
// Extern functions section

// BlueRobin init function
void InitProject_v(void);
// BlueRobin timer ISR
extern void BRTX_OC_IRQ_v(void); 
// SimpliciTI RF ISR
extern void MRFI_RfIsr(void);
// BlueRobin RF ISR
extern void BR_RfIsr(void);
// Set the identification (ID, serial number) of the transmitter
extern void BRTX_SetID_v(u32 ID_u32);
// Stop transmission
extern void BRTX_Stop_v(void);


// *************************************************************************************************
// Function prototype section
void bluerobin_start(void);
void bluerobin_stop(void);
void simpliciti_config(void);


// *************************************************************************************************
// Implementation

int main( void )
{  
  u16 enumeration = 0;
  u16 i = 0;
  u8  usb_service_is_disabled = 0;
  u8  cal;

#ifdef CC1111EM
  // P1.1 (LED) to output, default state is on
  P1DIR |= BIT1; 
  LED_ON;
  
  #ifdef DEBUG_OUTPUT  
    // P2.0 to output
    P2DIR |= BIT0; 
    P2_0 &= ~BIT0; 
    
    // P0.5 - P0.0 to output
    P0DIR |= BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;
    P0_5 &= ~BIT0; 
    P0_4 &= ~BIT0; 
    P0_3 &= ~BIT0; 
    P0_2 &= ~BIT0; 
    P0_1 &= ~BIT0; 
    P0_0 &= ~BIT0; 
  #endif  

#else
  // P1.0 (LED) to output '1'
  P1DIR |= BIT0; 
  LED_ON;
  // P1.4 (TP) to output '0'
  P1DIR |= BIT4; 
  TP_L;
  
  // P0.0 - P0.5 to output (unused)
  P0DIR |= BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;
  // P1.5, P1.6, P1.7 to output (unused)
  P1DIR |= BIT5 | BIT6 | BIT7;
  // P2.0, P2.3, P2.4 to output (unused)
  P2DIR |= BIT0 | BIT3 | BIT4;
#endif
  
  // Clock control
  // Default settings are 0x5C
  SLEEP &= ~(BIT2 | BIT1 | BIT0);   // PM0, both OSC ON   
  asm("NOP");
  while(!(SLEEP & (BIT6 | BIT5)));  // Wait for both OSC to be stable
  for (i=0; i<20000; i++);          // Simple delay
  CLKCON &= ~BIT6;                  // Crystal on 
  CLKCON &= ~(0x3F);                // clkspd=000, tickspd=000
  CLKCON |= BIT3 | BIT4 | BIT5;     // clkspd = 24MHz, tickspd = 24MHz/128=187500Hz
  while(!(SLEEP & (BIT6 | BIT5)));  // Wait for both OSC to be stable
  for (i=0; i<20000; i++);          // Simple delay  

  // Assign BlueRobin TX ID
  BRTX_SetID_v(TX_SERIAL_NO); 
  
  // Reset simpliciti_data
  simpliciti_data[0] = 0xFF;
  
  // Get calibration data from memory - range check added to prevent wrong calibration
  cal = flash_byte_read(0x7FF0);
  if (cal == 0x00) frequoffset = flash_byte_read(0x7FF2);
  if ((frequoffset > 30) && (frequoffset < (256-30))) frequoffset = 0;
  
  // Priority levels: USB, Timer1 (3) -> RF (3) -> Timer4 (1) -> others (0)
  IP1 |= BIT5 + BIT1 + BIT0 + BIT4;
  IP0 |= BIT5 + BIT1 + BIT0;

#ifdef TX_SIMPLE_TEST
  // TX out only
  test_on=1;
  rftest_radio_init(); 
  start_continuous_tx(); 
  while(1); 
#endif
#ifdef TX_MP_TEST
  test_on=1;
  test_step = 1;
  test_step_over = 0;
#endif
  
  // Init USB driver and enable global IRQ
  usbUartInit(115200);
  
  // Service USB functions normally during enumeration phase
  while ( enumeration < ENUMERATION_TIME ) 
  {
    usbUartProcess();
    enumeration++;
  }
  LED_OFF;
  
  // After enumeration start Timer4 IRQ to service USB driver from now on
  // f=187500Hz/64/5=3kHz/5 --> 1.7ms / IRQ  
  T4CCTL0 = 0x44;
  T4CC0 = 0x04;
  T4CTL = 0xDE;
  INT_ENABLE(INUM_T4, INT_ON);  

  // Enable interrupts
  INT_GLOBAL_ENABLE(TRUE);  
  
  // Main control loop
  while(1) 
  { 
    // For BlueRobin
    if ( bluerobin_start_now && !simpliciti_on && !wbsl_on)
    {
      // Start BlueRobin stack
      bluerobin_start(); 
      system_status = HW_BLUEROBIN_TRANSMITTING;
      // Reset start flag
      bluerobin_start_now = 0;
    }
    else if ( bluerobin_on )
    {  
      // Temporarily suspend USB service when a BlueRobin TX is pending
      if (sTimer1.usb_service_disable && !usb_service_is_disabled)
      {
        T4CTL &= ~BIT4;
        usb_service_is_disabled = 1;
      }
      
      if( sTimer1.iflag )
      {
        // Call BlueRobin timer ISR
        BRTX_OC_IRQ_v();
        
        // Allow USB service again
        if (usb_service_is_disabled)
        {
          sTimer1.usb_service_disable = 0;
          usb_service_is_disabled = 0;
          T4CTL |= BIT4;
        }
        
        // New!!!
        sTimer1.iflag = 0;
      }
    }
    // For SimpliciTI AP
    else if ( simpliciti_start_now && !wbsl_on )
    {
      LED_ON;
      // Clear start trigger
      simpliciti_start_now = 0;
      // Config hardware for SimpliciTI
      simpliciti_config();
      // Assign new system status
      system_status = HW_SIMPLICITI_LINKED;
      simpliciti_on = 1;
      // Start access point and stay there until exit flag is set
      simpliciti_main();
      // Clear SimpliciTI flags
      system_status = HW_SIMPLICITI_STOPPED;   
      simpliciti_on = 0;
      // Clean up after SimpliciTI
      simpliciti_data[0] = 0xFF;
      LED_OFF;
    }
    else if( wbsl_start_now && !simpliciti_on)
    {
      LED_ON;
      // Clear start Trigger
      wbsl_start_now = 0;
      //Config the RF module for WBSL
      wbsl_config();
      // Assign new System Status
      system_status = HW_WBSL_LINKED;
      wbsl_on = 1;
      // Start access point and try to pair  with an End Device,
      // once paired, download the software update then return.
      wbsl_main();
      
      // Check if there was an error during the Update procedure to alert the GUI
      if(getFlag(wbsl_flag,WBSL_STATUS_ERROR))
      {
        system_status = HW_WBSL_ERROR;
      }
      else
      {
        system_status = HW_WBSL_STOPPED;
      }
      //Clear WBSL Flags
      wbsl_on = 0;
      wbsl_data[0] = 0xFF;
      
      LED_OFF;
    }    
    else if (test_on)
    {
      if (!test_step_over) 
      {
        // Each test is executed just once
        switch (test_step)
        {
          case 1:   test_rf(); 
                    break;
          case 2:   start_continuous_tx(); 
                    break;
          case 3:   stop_continuous_tx(); 
                    break;
        }
        test_step_over = 1;
      }
    }
  }
}


// *************************************************************************************************
// Decode received command, extract data and trigger actions.
// *************************************************************************************************
void usb_decode(void)
{
  u32 id = 0;
  u16 test_pw = 0;
  u8 test_byte;
  u16 test_addr;
  u8 i;

  // Check if start marker is set
  if (usb_buffer[PACKET_BYTE_START] != 0xFF) return;

  // Check command code
  switch (usb_buffer[PACKET_BYTE_CMD])
  {
    // Generic commands
    case BM_GET_PRODUCT_ID:   usb_buffer[PACKET_BYTE_FIRST_DATA+3] = (u8)(PRODUCT_ID>>24);
                              usb_buffer[PACKET_BYTE_FIRST_DATA+2] = (u8)(PRODUCT_ID>>16);
                              usb_buffer[PACKET_BYTE_FIRST_DATA+1] = (u8)(PRODUCT_ID>>8);
                              usb_buffer[PACKET_BYTE_FIRST_DATA]   = (u8)(PRODUCT_ID); 
                              break;
    case BM_GET_STATUS:       usb_buffer[PACKET_BYTE_FIRST_DATA] = system_status; // + 1; 
                              break;

    // BlueRobin TX commands
    case BM_RESET:            bluerobin_stop();
                              bluerobin_on = 0;
                              simpliciti_on = 0;
                              wbsl_on = 0;
                              system_status = HW_IDLE;
                              break;
    case BM_START_BLUEROBIN:  if (simpliciti_on) setFlag(simpliciti_flag, SIMPLICITI_TRIGGER_STOP);
                              if (!bluerobin_on)
                              {
                                // Can only start once instance
                                bluerobin_start_now = 1;
                              }
                              break;
    case BM_STOP_BLUEROBIN:   bluerobin_stop();
                              system_status = HW_BLUEROBIN_STOPPED;
                              break;
    case BM_SET_BLUEROBIN_ID: id  = ((unsigned long)usb_buffer[PACKET_BYTE_FIRST_DATA+3] << 12) << 12;
                              id += ((unsigned long)usb_buffer[PACKET_BYTE_FIRST_DATA+2] << 8) << 8;
                              id += (unsigned long)usb_buffer[PACKET_BYTE_FIRST_DATA+1] << 8;
                              id += (unsigned long)usb_buffer[PACKET_BYTE_FIRST_DATA];
                              BRTX__ID_u32 = id;
                              break;
    case BM_GET_BLUEROBIN_ID: id = BRTX__ID_u32;
                              usb_buffer[PACKET_BYTE_FIRST_DATA+3] = (unsigned char)(id >> 24);
                              usb_buffer[PACKET_BYTE_FIRST_DATA+2] = (unsigned char)(id >> 16);
                              usb_buffer[PACKET_BYTE_FIRST_DATA+1] = (unsigned char)(id >> 8);
                              usb_buffer[PACKET_BYTE_FIRST_DATA] = (unsigned char)(id);
                              break;
    case BM_SET_HEARTRATE:    HeartRate_u8 = usb_buffer[PACKET_BYTE_FIRST_DATA];
                              break;
    case BM_SET_SPEED:        Speed_u8      = usb_buffer[PACKET_BYTE_FIRST_DATA];    
                              Distance_u16  = (unsigned int)(usb_buffer[PACKET_BYTE_FIRST_DATA+2] << 8); 
                              Distance_u16 += usb_buffer[PACKET_BYTE_FIRST_DATA+1]; 
                              break;
   
    // SimpliciTI RX commands
    case BM_START_SIMPLICITI: if (bluerobin_on) bluerobin_stop();
                              // Can only start one stack
                              if (!simpliciti_on) 
                              {
                                system_status = HW_SIMPLICITI_TRYING_TO_LINK;
                               // simpliciti_start_rx_only_now = 1;
                                simpliciti_start_now = 1;
                              }
                              break;
    case BM_GET_SIMPLICITIDATA:   
                              if (getFlag(simpliciti_flag, SIMPLICITI_TRIGGER_RECEIVED_DATA))
                              {
                                // Assemble IN packet (ID/BTN, DataX, DataY, DataZ)
                                usb_buffer[PACKET_BYTE_FIRST_DATA+3] = simpliciti_data[3];
                                usb_buffer[PACKET_BYTE_FIRST_DATA+2] = simpliciti_data[2];
                                usb_buffer[PACKET_BYTE_FIRST_DATA+1] = simpliciti_data[1];
                                usb_buffer[PACKET_BYTE_FIRST_DATA]   = simpliciti_data[0];
                                // Mark buffer as already read
                                simpliciti_data[0] = 0xFF; 
                                clearFlag(simpliciti_flag, SIMPLICITI_TRIGGER_RECEIVED_DATA);
                              }
                              else
                              {
                                // Return packet with "already read" marker
                                usb_buffer[PACKET_BYTE_FIRST_DATA]   = 0xFF;
                              }
                              break;
                              
    // SimpliciTI Sync commands
    case BM_SYNC_START:    /*   if (bluerobin_on) bluerobin_stop();
                              // Can only start one stack
                              if (!simpliciti_on) 
                              {
                                system_status = HW_SIMPLICITI_TRYING_TO_LINK;
                                simpliciti_start_sync_now = 1;
                              }*/
                              break;
    case BM_SYNC_SEND_COMMAND:   
                              // Copy command data to SimpliciTI buffer
                              if (simpliciti_on) 
                              {
                                for (i=0; i<BM_SYNC_DATA_LENGTH; i++) simpliciti_data[i] = usb_buffer[PACKET_BYTE_FIRST_DATA+i];
                                // Set flag to send out command when receiving next ready-to-receive packet
                                setFlag(simpliciti_flag, SIMPLICITI_TRIGGER_SEND_CMD);
                              }
                              break;
    case BM_SYNC_GET_BUFFER_STATUS:   
                              // Set response    
                              usb_buffer[3] = simpliciti_sync_buffer_status;
                              // Set reply packet length 
                              usb_buffer[PACKET_BYTE_SIZE] = 1 + PACKET_OVERHEAD_BYTES;
                              break;
    case BM_SYNC_READ_BUFFER:  
                              // Copy bytes from sync buffer to USB buffer
                              if (simpliciti_sync_buffer_status)
                              {
                                for (i=0; i<BM_SYNC_DATA_LENGTH; i++) usb_buffer[PACKET_BYTE_FIRST_DATA+i] = simpliciti_data[i];
                                // Free buffer
                                simpliciti_sync_buffer_status = 0;
                              }
                              // Set reply packet length 
                              usb_buffer[PACKET_BYTE_SIZE] = BM_SYNC_DATA_LENGTH + PACKET_OVERHEAD_BYTES;
                              break;
                              

    // SimpliciTI shared commands                              
    case BM_STOP_SIMPLICITI:  // Stop through remote control
                              setFlag(simpliciti_flag, SIMPLICITI_TRIGGER_STOP);
                              break;

                              
                        
    // WBSL commands
    case BM_START_WBSL:       //Start WBSL procedure
                              
                              // Can only start one stack
                              if (bluerobin_on) bluerobin_stop();
                              // Can only start one stack
                              if (simpliciti_on) setFlag(simpliciti_flag, SIMPLICITI_TRIGGER_STOP);
                              
                              if(!wbsl_on)
                              {
                                system_status = HW_WBSL_TRYING_TO_LINK;
                               // Trigger the BSL Start
                                wbsl_start_now = 1;
                              }
                              break;
                              
    case BM_STOP_WBSL:        //Stop running the BSL
                              //TODO
                              setFlag(wbsl_flag, WBSL_TRIGGER_STOP);
                              LED_OFF;
                              break;
    
    case BM_GET_WBSL_STATUS:     //Get the status of how far along is the transmission of the Update
                                usb_buffer[PACKET_BYTE_FIRST_DATA]   = wbsl_status;
                              break;
    
    case BM_GET_PACKET_STATUS_WBSL: 
                                if(wbsl_on)
                                {
                                   //Get the status of wheter or not a new packet needs to be sent from the GUI
                                   usb_buffer[PACKET_BYTE_FIRST_DATA]   = wbsl_packet_flag;
                                }
                                else
                                {
                                   // This Command shouldn't have come when wbsl is off, tell the GUI an error has ocurred
                                   usb_buffer[PACKET_BYTE_FIRST_DATA] = WBSL_ERROR;
                                }
                                break;
    case BM_GET_MAX_PAYLOAD_WBSL:  // Send the max number of bytes allowed in the payload
                                usb_buffer[PACKET_BYTE_FIRST_DATA]   = WBSL_MAX_PAYLOAD_LENGTH;
                                break;
    case BM_SEND_DATA_WBSL:   
                             // Set the flag as processing so that the GUI knows that it needs to wait before sending next packet
                             wbsl_packet_flag = WBSL_PROCESSING_PACKET;
                             
                              // Check if this is the first message received for the current packet
                             if (packet_ready_flag == WBSL_PACKET_EMPTY)
                             {
                               current_packet_size = 0;
                               wbsl_inc = usb_buffer[PACKET_BYTE_FIRST_DATA - 1] - 3 - 2; // Get total of bytes received and substract overhead (3 from USB and 2 from Packet Length and Address Byte
                               
                               // Save the OP Code Byte
                               wbsl_OPCode = usb_buffer[PACKET_BYTE_FIRST_DATA];
                               
                               if(wbsl_OPCode == WBSL_INIT_PACKET) // Init Packet
                               {
                                  TxBuffer[WBSL_OVERHEAD_LENGTH] = usb_buffer[PACKET_BYTE_FIRST_DATA + 1];
                                  TxBuffer[WBSL_OVERHEAD_LENGTH + 1] = usb_buffer[PACKET_BYTE_FIRST_DATA + 2];
                                  
                                   // Save the total number of packets to be sent
                                  total_packets = usb_buffer[PACKET_BYTE_FIRST_DATA + 1] +((usb_buffer[PACKET_BYTE_FIRST_DATA + 2] << 8) & 0xFF00);

                                  initPacket[INIT_TOTAL_PACKETS_OFFSET] = total_packets & 0xFF;
                                  initPacket[INIT_TOTAL_PACKETS_OFFSET+1] = (total_packets >> 8) & 0xFF;
                                  packet_ready_flag = WBSL_PACKET_FULL;
                                  ptrTX = 0;
                               }
                               else   // Regular data packet 
                               {
                                 wbsl_packetLength = usb_buffer[PACKET_BYTE_FIRST_DATA + 1];
                                 
                                 if(wbsl_OPCode == WBSL_ADDRESS_PACKET)
                                   TxBuffer[WBSL_OPCODE_OFFSET] = WBSL_ADDRESS_PACKET; // Set the address OPCODE
                                 else
                                   TxBuffer[WBSL_OPCODE_OFFSET] = WBSL_NORMAL_PACKET; // Set the packet as a normal packet
                                 
                                 //Update size of packet to be sent and substract Overhead and Length field
                                 TxBuffer[0] = wbsl_packetLength + WBSL_OVERHEAD_LENGTH - 1;
                                  
                                 for(i=0;i<wbsl_inc;i++)  // Copy all Received Bytes to the TxBuffer
                                 {
                                    TxBuffer[i+WBSL_OVERHEAD_LENGTH] = usb_buffer[PACKET_BYTE_FIRST_DATA + i + 2];
                                 }

                                  packet_ready_flag = WBSL_PACKET_FILLING;
                                  ptrTX = wbsl_inc;  // Update pointer
                                  
                                  // Update the current size to know when the complete packet has been received
                                  current_packet_size += wbsl_inc;
                                  
                                  // Check if complete packet has been received
                                  if(current_packet_size >= wbsl_packetLength)
                                  {
                                    packet_ready_flag = WBSL_PACKET_FULL;
                                    ptrTX = 0;
                                  }
                               }
                             }
                             else if(packet_ready_flag == WBSL_PACKET_FILLING)
                             {
                                wbsl_inc = usb_buffer[PACKET_BYTE_FIRST_DATA - 1] - 3; // Get total of bytes received and substract overhead
                                
                                for(i=0;i<wbsl_inc;i++)// Copy all Received Bytes to the TxBuffer
                                {
                                  TxBuffer[ptrTX + i + WBSL_OVERHEAD_LENGTH] = usb_buffer[PACKET_BYTE_FIRST_DATA + i];
                                }
                                ptrTX += wbsl_inc;  // Update pointer
                                
                                // Update the current size to know when the complete packet has been received
                                current_packet_size += wbsl_inc;

                                // Check if total bytes have been received or the maximum payload length has been reached
                                // reaching max payload length and still having bytes to be read is an error since bytes would be lost
                                if(ptrTX >= WBSL_MAX_PAYLOAD_LENGTH || current_packet_size >= wbsl_packetLength)
                                {
                                  packet_ready_flag = WBSL_PACKET_FULL;
                                  ptrTX = 0;
                                }
                             }
                              break; 

                             
    // Test commands
    case BM_INIT_TEST:        test_pw = (u16)usb_buffer[PACKET_BYTE_FIRST_DATA+1] << 8;
                              test_pw += (u16)usb_buffer[PACKET_BYTE_FIRST_DATA];
                              if (test_pw == RFTEST_PW) 
                              {
                                test_on        = 1;
                                test_step      = 0;
                                test_step_over = 1;
                                bluerobin_on   = 0;
                                simpliciti_on  = 0;
                              }
                              else                      
                              {
                                test_on       = 0;
                              }
                              break;
  
    case BM_NEXT_TEST:        if (test_on)
                              {
                                test_result = 0;
                                test_step_over = 0;
                                test_step++;
                              }
                              break;
                              

    case BM_GET_TEST_RESULT:  if (test_on)
                              {
                                usb_buffer[PACKET_BYTE_FIRST_DATA]   = (u8)test_result;
                                usb_buffer[PACKET_BYTE_FIRST_DATA+1] = (u8)(test_result >> 8);
                              }
                              break;
                           
    case BM_WRITE_BYTE:       if (test_on)
                              {
                                test_byte  = usb_buffer[PACKET_BYTE_FIRST_DATA];
                                test_addr  = (u16)usb_buffer[PACKET_BYTE_FIRST_DATA+1];
                                test_addr += ((u16)usb_buffer[PACKET_BYTE_FIRST_DATA+2] << 8);

                                // Allow flash write access                                
                                flash_start_update();
                                
                                // Write calibration data only if memory location empty
                                if (flash_byte_read(test_addr) == 0xFF) 
                                {
                                  // Write byte to cal page
                                  flash_word_write(test_addr, ((u16)test_byte << 8) + 0xFF, FLASH_EW_KEY, 0);
                                }
                              }
                              break;

  }
  
  // Return packet with original data, but modified command byte (acknowledge)
  usb_sendack = 1;
  usb_buffer[PACKET_BYTE_CMD] = HW_NO_ERROR;
}



// *************************************************************************************************
// Config clock and timers for SimpliciTI
// *************************************************************************************************
void simpliciti_config(void)
{
  // OSC=XT, TICKSPD=fref/64, CLKSPD=12MHz
  CLKCON = BIT7 | BIT5 | BIT4; 
    
  // Reset radio registers to default settings
  SYNC1       = 0xD3;               /*  Sync word, high byte                                */
  SYNC0       = 0x91;               /*  Sync word, low byte                                 */
  PKTCTRL1    = 0x04;               /*  Packet automation control                           */
  ADDR        = 0x00;               /*  Device address                                      */
  CHANNR      = 0x00;               /*  Channel number                                      */
  MCSM2       = 0x07;               /*  Main Radio Control State Machine configuration      */
  IOCFG2      = 0x00;               /*  Radio Test Signal Configuration (P1_7)              */
  IOCFG1      = 0x00;               /*  Radio Test Signal Configuration (P1_6)              */
  IOCFG0      = 0x00;               /*  Radio Test Signal Configuration (P1_5)              */
  
  // Reset RF int mask bits - SimpliciTI will set them 
  RFIM = 0;
}


// *************************************************************************************************
// Start BlueRobin transmission
// *************************************************************************************************
void bluerobin_start(void)
{
  // OSC=XT, TICKSPD=fref/128, CLKSPD=24MHz
  CLKCON = BIT7 | BIT5 | BIT4 | BIT3; 
  
  // Enable RF IRQs
  RFIM |= BIT7 | BIT4;
  
  // Clear Timer1
  T1CNTL = 0x55;

  // Enable Timer1 interrupt, enable compare mode
  T1CCTL0 = 0x44;

  // Start T1 in modulo mode
  T1CTL = 0x02;

  // Init hardware and BlueRobin stack
  INT_ENABLE(INUM_T1, INT_ON);
  
  // Init s/w
  InitProject_v();
  
  // Set FSCTRL0 to calibration value
  FSCTRL0 = frequoffset;
  
  // Set on flag
  bluerobin_on = 1;
}


// *************************************************************************************************
// Stop BlueRobin transmission
// *************************************************************************************************
void bluerobin_stop(void)
{
  BRTX_Stop_v();

  // Stop Timer1 
  INT_ENABLE(INUM_T1, INT_OFF);
  T1CTL = 0x00;

  // Clear Timer1
  T1CNTL = 0x55;

  // Clear Timer1 structure
  reset_timer1();
  
  // Clear on flag
  bluerobin_on = 0;
}


// *************************************************************************************************
// Forward RF IRQ to right handler 
// *************************************************************************************************
#pragma vector=RF_VECTOR
__interrupt void rf_ISR(void)
{
  if ( bluerobin_on )
  {
    BR_RfIsr();    
  }
  else if ( simpliciti_on )
  {
    MRFI_RfIsr();  
  }
  else if ( test_on )
  {
    rftest_RfIsr();    
  }
  else if ( wbsl_on )
  {
    wbsl_RfIsr();
  }
}

// *************************************************************************************************
// Forward RF TX/RX IRQ to right handler 
// *************************************************************************************************
#pragma vector=RFTXRX_VECTOR
__interrupt void ISR_RFTXRX(void)
{
  if ( bluerobin_on )
  {
    BR_RfTxRxIsr();    
  }
  else if ( test_on )
  {
    rftest_RfTxRxIsr();    
  }
}


// *************************************************************************************************
// Timer1 ISR
// *************************************************************************************************
#pragma vector=T1_VECTOR
__interrupt void t1Timer_ISR(void)
{
  // Clear IRCON.T1IF
  IRCON &= ~0x02;

  // Clear Timer 1 Channel 0-2 + overflow interrupt flag
  T1CTL &= ~0xF0;

  // Return immediately if timer not enabled
  if (!sTimer1.enable) return;

  // Return immediately if s/w int flag not reset
  if (sTimer1.iflag) return;
  
  // Set timer cycles, and call s/w int handler if no more cycles to set
  if (!set_timer1_cycles()) 
  {
    // Call BlueRobin TX function
    sTimer1.iflag = 1;  
  }
  
  // Increase global cycle count
  sTimer1.cycles++;
}


// *************************************************************************************************
// Timer4 ISR
// *************************************************************************************************
#pragma vector=T4_VECTOR
__interrupt void timer4_ISR(void)
{
  
  // If WBSL_ON check if any timeout entry needs to be serviced
  if( wbsl_on ){
    //If the timeout entry is enabled
    if( timeout_table[WBSL_TIMEOUT_INDEX].timeout > 0){  
        //Check if timeout has been reached
       if(++timeout_table[WBSL_TIMEOUT_INDEX].counter == timeout_table[WBSL_TIMEOUT_INDEX].timeout)
       {
         timeout_table[WBSL_TIMEOUT_INDEX].counter = 0;    // Reset counter
         timeout_table[WBSL_TIMEOUT_INDEX].timeout = 0;    // Reset timeout (Disable)
         timeout_table[WBSL_TIMEOUT_INDEX].flag = 1;       // Set the flag so that WBSL knows that
       }
    }
  }

  // Clear IRQ flag
  TIMIF &= ~BIT3;
  
  // Service USB functions
  usbUartProcess();
}


// *************************************************************************************************
// Assign all unused IRQ
// *************************************************************************************************
void catch_stray_irq(void)
{
  asm("nop");
}
#pragma vector=ADC_VECTOR
__interrupt void adc_ISR(void) { catch_stray_irq(); }
#pragma vector=URX0_VECTOR
__interrupt void urx0_ISR(void) { catch_stray_irq(); }
#pragma vector=URX1_VECTOR
__interrupt void urx1_ISR(void) { catch_stray_irq(); }
#pragma vector=ENC_VECTOR
__interrupt void enc_ISR(void) { catch_stray_irq(); }
#pragma vector=ST_VECTOR
__interrupt void st_ISR(void) { catch_stray_irq(); }
#pragma vector=UTX0_VECTOR
__interrupt void utx0_ISR(void) { catch_stray_irq(); }
#pragma vector=DMA_VECTOR
__interrupt void dma_ISR(void) { catch_stray_irq(); }
#pragma vector=T2_VECTOR
__interrupt void t2_ISR(void) { catch_stray_irq(); }
#pragma vector=UTX1_VECTOR
__interrupt void utx1_ISR(void) { catch_stray_irq(); }
#pragma vector=P1INT_VECTOR
__interrupt void p1_ISR(void) { catch_stray_irq(); }
#pragma vector=WDT_VECTOR
__interrupt void wdt_ISR(void) { catch_stray_irq(); }
