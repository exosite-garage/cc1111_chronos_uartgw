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
// Generic defines and variables

// Macros
#define getFlag(val, flag)		            ((val&flag)==flag)
#define setFlag(val, flag)		            (val|=flag)
#define clearFlag(val, flag)		            (val&=(~flag))
#define toggleFlag(val, flag)		            (val^=flag)

// Entry point into SimpliciTI library
extern void simpliciti_main(void);

// Maximum data length
#define SIMPLICITI_MAX_PAYLOAD_LENGTH       	    (32u)

// Data to send / receive 
extern unsigned char simpliciti_data[SIMPLICITI_MAX_PAYLOAD_LENGTH];

// 1 = send one or more reply packets, 0 = no need to reply
extern unsigned char simpliciti_reply;

// Radio frequency offset taken from calibration memory
// Compensates crystal deviation from 26MHz nominal value
extern unsigned char rf_frequoffset;

// Flag for status information and external control through USB driver
extern volatile unsigned char simpliciti_flag;
#define SIMPLICITI_STATUS_LINKING		    (BIT0) 	
#define SIMPLICITI_STATUS_LINKED		    (BIT1)
#define SIMPLICITI_STATUS_ERROR		            (BIT2)
#define SIMPLICITI_TRIGGER_SEND_DATA 	            (BIT3)
#define SIMPLICITI_TRIGGER_RECEIVED_DATA 	    (BIT4)
#define SIMPLICITI_TRIGGER_STOP		            (BIT5)
#define SIMPLICITI_TRIGGER_SEND_CMD                 (BIT6)


// ---------------------------------------------------------------
// SimpliciTI Sync

// 1 = data in buffer, 0 = buffer empty
extern unsigned char simpliciti_sync_buffer_status;

#define BM_SYNC_DATA_LENGTH                         (19u)

// Device data  (0)TYPE   (1) - (31) DATA 
#define SYNC_ED_TYPE_R2R                            (1u)
#define SYNC_ED_TYPE_MEMORY                         (2u)
#define SYNC_ED_TYPE_STATUS                         (3u)

// Host data    (0)CMD    (1) - (31) DATA 
#define SYNC_AP_CMD_NOP                             (1u)
#define SYNC_AP_CMD_GET_STATUS			    (2u)
#define SYNC_AP_CMD_SET_WATCH                       (3u)
#define SYNC_AP_CMD_GET_MEMORY_BLOCKS_MODE_1   	    (4u)
#define SYNC_AP_CMD_GET_MEMORY_BLOCKS_MODE_2   	    (5u)
#define SYNC_AP_CMD_ERASE_MEMORY                    (6u)
#define SYNC_AP_CMD_EXIT			    (7u)


