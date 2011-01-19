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
// Defines required for USB communication
// *************************************************************************************************

// USB packet length
#define	PACKET_OVERHEAD_BYTES	    3
#define	PACKET_DATA_BYTES	    2
#define PACKET_TOTAL_BYTES          (PACKET_OVERHEAD_BYTES + PACKET_DATA_BYTES)

// Packet bytes
//
// Byte 0	        Start marker (0xFF)
#define PACKET_BYTE_START          (0u)
// Byte 1	        Command code
#define PACKET_BYTE_CMD            (1u)
// Byte 2	        Packet size (including overhead)
#define PACKET_BYTE_SIZE           (2u)
// Byte 3..packet_size  Data
#define PACKET_BYTE_FIRST_DATA     (3u)


// Command codes
#define BM_GET_STATUS           0x00
#define BM_GET_PRODUCT_ID       0x20

// BlueRobin
#define BM_RESET                    0x01
#define BM_START_BLUEROBIN          0x02
#define BM_SET_BLUEROBIN_ID         0x03
#define BM_GET_BLUEROBIN_ID         0x04
#define BM_SET_HEARTRATE            0x05    
#define BM_STOP_BLUEROBIN           0x06
#define BM_SET_SPEED                0x0A

// Simpliciti
#define BM_START_SIMPLICITI         0x07
#define BM_GET_SIMPLICITIDATA       0x08
#define BM_STOP_SIMPLICITI          0x09

// Sync
#define BM_SYNC_START		    0x30
#define BM_SYNC_SEND_COMMAND        0x31
#define BM_SYNC_GET_BUFFER_STATUS   0x32
#define BM_SYNC_READ_BUFFER	    0x33

//Wireless BSL
#define BM_START_WBSL               0x40
#define BM_GET_WBSL_STATUS          0x41
#define BM_INIT_OK_WBSL             0x42
#define BM_INIT_INVALID_WBSL        0x43
#define BM_TRANSFER_OK_WBSL         0x44
#define BM_TRANSFER_INVALID_WBSL    0x45
#define BM_STOP_WBSL                0x46
#define BM_SEND_DATA_WBSL           0x47
#define BM_GET_PACKET_STATUS_WBSL   0x48
#define BM_GET_MAX_PAYLOAD_WBSL     0x49

// Test
#define BM_INIT_TEST                0x70
#define BM_NEXT_TEST                0x71
#define BM_WRITE_BYTE               0x72
#define BM_GET_TEST_RESULT          0x73

// System states  
#define HW_IDLE                         0x00
#define HW_SIMPLICITI_STOPPED           0x01
#define HW_SIMPLICITI_TRYING_TO_LINK    0x02
#define HW_SIMPLICITI_LINKED            0x03
#define HW_BLUEROBIN_STOPPED            0x04
#define HW_BLUEROBIN_TRANSMITTING       0x05
#define HW_ERROR			0x05
#define HW_NO_ERROR			0x06
#define HW_NOT_CONNECTED		0x07
#define HW_SIMPLICITI_LINK_TIMEOUT      0x08
#define HW_WBSL_TRYING_TO_LINK          0x09
#define HW_WBSL_LINKED                  0x0A
#define HW_WBSL_ERROR                   0x0B
#define HW_WBSL_STOPPED                 0x0C
#define HW_WBSL_LINK_TIMEOUT            0x0D