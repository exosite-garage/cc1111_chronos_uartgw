/**********************************************************************************************
  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights granted under
  the terms of a software license agreement between the user who downloaded the software,
  his/her employer (which must be your employer) and Texas Instruments Incorporated (the
  "License"). You may not use this Software unless you agree to abide by the terms of the
  License. The License limits your use, and you acknowledge, that the Software may not be
  modified, copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio frequency
  transceiver, which is integrated into your product. Other than for the foregoing purpose,
  you may not use, reproduce, copy, prepare derivative works of, modify, distribute,
  perform, display or sell this Software and/or its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS”
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
  WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
  IN NO EVENT SHALL TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
  THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
  INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST
  DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY
  THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/


// *************************************************************************************************
// Include section
#include <string.h>
#include "bsp.h"
#include "mrfi.h"
#include "bsp_leds.h"
#include "bsp_buttons.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "nwk_frame.h"
#include "nwk.h"
#include "app_remap_led.h"
#include "simpliciti.h"


// *************************************************************************************************
// Defines section
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

// *************************************************************************************************
// Prototypes section

/* callback handler */
uint8_t sCB(linkID_t);


// *************************************************************************************************
// Extern section
extern uint8_t sInit_done;


// *************************************************************************************************
// Global Variable section

/* reserve space for the maximum possible peer Link IDs */
static linkID_t linkID0;
static uint8_t  sNumCurrentPeers;

/* work loop semaphores */
static volatile uint8_t sPeerFrameSem;
static volatile uint8_t sJoinSem;

volatile unsigned char simpliciti_flag;
unsigned char simpliciti_data[SIMPLICITI_MAX_PAYLOAD_LENGTH];
unsigned char ed_data[SIMPLICITI_MAX_PAYLOAD_LENGTH];

// AP main routine
void simpliciti_main(void)
{
  bspIState_t intState;
  uint8_t j;
  uint8_t len;
  uint32_t led_toggle = 0;
  uint8_t   pwr;

  // Init variables  
  simpliciti_flag = SIMPLICITI_STATUS_LINKING;

  // Init SimpliciTI
  SMPL_Init(sCB);
  
  // Set output power to +1.1dBm (868MHz) / +1.3dBm (915MHz)
  pwr = IOCTL_LEVEL_2;
  SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SETPWR, &pwr);
  
   // LED off
  BSP_TURN_OFF_LED1();
    
  /* main work loop */
  while (1)
  {
    // Wait for the Join semaphore to be set by the receipt of a Join frame from a
    //device that supports an End Device.
    if (sJoinSem && !sNumCurrentPeers)
    {
      /* listen for a new connection */
      while (1)
      {
        if (SMPL_SUCCESS == SMPL_LinkListen(&linkID0))
        {
          // We have a connection
          simpliciti_flag = SIMPLICITI_STATUS_LINKED;
          BSP_TURN_ON_LED1();
          break;
        }
        /* Implement fail-to-link policy here. otherwise, listen again. */
      }

      sNumCurrentPeers++;

      BSP_ENTER_CRITICAL_SECTION(intState);
      sJoinSem--;
      BSP_EXIT_CRITICAL_SECTION(intState);
    }

    /* Have we received a frame on one of the ED connections?
     * No critical section -- it doesn't really matter much if we miss a poll
     */
    if (sPeerFrameSem)
    {
      // Continuously try to receive end device packets
      if (SMPL_SUCCESS == SMPL_Receive(linkID0, ed_data, &len))
      {
        // Acceleration / ppt data packets are 4 byte long
        if (len == 4)
        {
          BSP_TOGGLE_LED1();
          memcpy(simpliciti_data, ed_data, 4);
          setFlag(simpliciti_flag, SIMPLICITI_TRIGGER_RECEIVED_DATA);
        }        
        // Sync packets are either R2R (2 byte) or data (19 byte) long
        else if ((len == 2) || (len == 19))
        {
          // Indicate received packet
          BSP_TOGGLE_LED1();

          // Decode end device packet
          switch (ed_data[0])
          {
              case SYNC_ED_TYPE_R2R: 
                                    // Send reply
                                    if (getFlag(simpliciti_flag, SIMPLICITI_TRIGGER_SEND_CMD))
                                    {
                                      // Clear flag
                                      clearFlag(simpliciti_flag, SIMPLICITI_TRIGGER_SEND_CMD);
                                      // Command data was set by USB buffer previously
                                      len = BM_SYNC_DATA_LENGTH;
                                    }
                                    else // No command currently available
                                    {
                                      simpliciti_data[0] = SYNC_AP_CMD_NOP;
                                      simpliciti_data[1] = 0x55;
                                      len = 2;
                                    }
                
                                    // Send reply packet to end device
                                    SMPL_Send(linkID0, simpliciti_data, len);
                                    break;
                                 
            case SYNC_ED_TYPE_MEMORY: 
            case SYNC_ED_TYPE_STATUS:
                                    // If buffer is empty, copy received end device data to intermediate buffer
                                    if (!simpliciti_sync_buffer_status)
                                    {
                                      for (j=0; j<BM_SYNC_DATA_LENGTH; j++) simpliciti_data[j] = ed_data[j];
                                      simpliciti_sync_buffer_status = 1;
                                    }
                                    // Set buffer status to full
                                    break;

          }
        }
      }
    }
    
    // Exit function if SIMPLICITI_TRIGGER_STOP flag bit is set in USB driver    
    if (getFlag(simpliciti_flag, SIMPLICITI_TRIGGER_STOP)) 
    {
      // Immediately turn off RF interrupt
      IEN2 &= ~BIT0;
      RFIM = 0;
      RFIF = 0;
      // Clean up after SimpliciTI and enable restarting the stack
      linkID0 = 0;
      sNumCurrentPeers = 0;
      sJoinSem = 0;
      sPeerFrameSem = 0;
      sInit_done = 0;
      // LED off
      BSP_TURN_OFF_LED1();
      return;
    }
    
    // Blink slowly to indicate that access point is on
    if (!sNumCurrentPeers)
    {
      if (led_toggle++>150000)
      {
        BSP_TOGGLE_LED1();
        led_toggle = 0;
      }
    }
  }
}



/* Runs in ISR context. Reading the frame should be done in the */
/* application thread not in the ISR thread. */
uint8_t sCB(linkID_t lid)
{
  if (lid)
  {
    sPeerFrameSem++;
  }
  else
  {
    sJoinSem++;
  }

  /* leave frame to be read by application. */
  return 0;
}
