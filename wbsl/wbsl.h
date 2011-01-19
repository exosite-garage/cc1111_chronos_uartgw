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

#ifndef __WBSL_H
#define __WBSL_H

#include <ioCC1111.h>

//#define CCA_MODE

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


// Macros
#define getFlag(val, flag)		            ((val&flag)==flag)
#define setFlag(val, flag)		            (val|=flag)
#define clearFlag(val, flag)		            (val&=(~flag))
#define toggleFlag(val, flag)		            (val^=flag)


typedef struct {
   unsigned int counter;
   unsigned int timeout;
   unsigned char flag;
}TIMER_TABLE_ENTRY;

/* bit value */
#ifndef BV
#define BV(n)      (1 << (n))
#endif

#define WBSL_AP_ADDRESS                             (0xCA)
#define WBSL_MAX_PAYLOAD_LENGTH                     (247u)

#if WBSL_MAX_PAYLOAD_LENGTH < 50
#error "MAXIMUM PAYLOAD CANNOT BE LOWER TO 50 BYTES, DUE TO PROTOCOL TIMING CONSTRAINTS"
#elif WBSL_MAX_PAYLOAD_LENGTH > 247
#error "MAXIMUM PAYLOAD CANNOT BE HIGHER THAN 247 BYTES.
#endif

#define WBSL_OVERHEAD_LENGTH                        (6u)
#define WBSL_TOTAL_LENGTH                           WBSL_MAX_PAYLOAD_LENGTH + WBSL_OVERHEAD_LENGTH
#define DISCOVERY_PAYLOAD_LENGTH                    (4u)
#define DISCOVERY_OVERHEAD_LENGTH                   (3u)
#define AP_ADDRESS_OFFSET_RX                        (1u)
#define ED_ADDRESS_OFFSET_RX                        (2u)
#define BATTERY_VOLTAGE_OFFSET                      (3u)

#define WBSL_CRC_STATUS_OFFSET                      (2)
#define CRC_STATUS                                  (0x80)

#define WBSL_OPCODE_OFFSET                          (5u)

#define AP_ADDRESS_OFFSET_TX                        (2u)
#define ED_ADDRESS_OFFSET_TX                        (1u)

#define DISCOVERY_ACK_OFFSET                        (3u)
#define INIT_TOTAL_PACKETS_OFFSET                   (3u)
#define CURRENT_PACKET_NR_OFFSET                    (3u)

#define WBSL_TX_RES_FAILED                          (0)
#define WBSL_TX_RES_SUCCESS                         (1u)

#define WBSL_ACK_PKT_SIZE                           (5u)

/* DMA channel number */
#define WBSL_DMA_CHAN                               0
#define DMA_ABORT                                   (0x80)

// Battery end of life voltage threshold -> do not accept connection
#define BATTERY_LOW_THRESHOLD			(240u)

// Flag for status information and external control through USB driver
extern volatile unsigned char wbsl_flag;
#define WBSL_STATUS_LINKING		            (BIT0) 	
#define WBSL_STATUS_LINKED		            (BIT1)
#define WBSL_STATUS_ERROR		            (BIT2)
#define WBSL_TRIGGER_SEND_DATA 	                    (BIT3)
#define WBSL_TRIGGER_RECEIVED_DATA 	            (BIT4)
#define WBSL_TRIGGER_STOP		            (BIT5)
#define WBSL_TRIGGER_SEND_CMD                       (BIT6)
#define WBSL_LOW_BATT                               (BIT7)

// Flag to check if a new packet is needed from the GUI
extern volatile unsigned char wbsl_packet_flag;
#define WBSL_DISABLED                               (BIT0)
#define WBSL_PROCESSING_PACKET                      (BIT1)
#define WBSL_SEND_INFO_PACKET                       (BIT2)
#define WBSL_SEND_NEW_DATA_PACKET                   (BIT3)
#define WBSL_ERROR                                  (BIT7)

// Flag for status information, to see if WBSL is in RX/TX/IDLE mode
extern volatile unsigned char wbslMode_flag;
#define WBSL_IDLE_MODE                              (BIT0)
#define WBSL_RX_MODE                                (BIT1)
#define WBSL_TX_MODE                                (BIT2)

extern volatile unsigned char rxtx_flag;
#define WBSL_RXTX_RECEIVED                          (BIT0)
#define WBSL_RXTX_SEND                              (BIT1)

// Values for linking failed or successful
#define WBSL_LINK_FAIL                              0
#define WBSL_LINK_SUCC                              1

// Values for Packet ACK
#define WBSL_ACK_FAIL                               (0)
#define WBSL_ACK_SUCC                               (1u)

#define TIMEOUT_FOR_ACK                             (440)

#define WBSL_MAXIMUM_RETRIES                        (5u)

extern volatile unsigned char packet_ready_flag;
#define WBSL_PACKET_EMPTY                           (BIT0)
#define WBSL_PACKET_FILLING                         (BIT1)
#define WBSL_PACKET_FULL                            (BIT2)
#define WBSL_PACKET_ADDRESS                         (BIT3)

//WBSL OP Code Type
#define WBSL_INIT_PACKET                            0x00
#define WBSL_ADDRESS_PACKET                         0x01
#define WBSL_NORMAL_PACKET                          0x02

#define ADDRESS_FLAG_BIT                            0x80

#define MAX_TIMEOUT_TABLE_SIZE                      (1u)
#define WBSL_TIMEOUT_INDEX                          (0u)

// 1 = data in buffer, 0 = buffer empty
extern unsigned char wbsl_sync_buffer_status;
extern unsigned char ed_address;


extern unsigned char wbsl_data[WBSL_MAX_PAYLOAD_LENGTH];
extern unsigned char wbsl_status;
extern unsigned char wbsl_txfifo_filled;

extern unsigned long total_size_in_bytes;
extern unsigned int total_packets;

#define TX_SIZE                                       WBSL_TOTAL_LENGTH
extern unsigned char TxBuffer[TX_SIZE];
extern unsigned char initPacket[6];

// Table that stores the timeout value, this will be used by Timer4 and WBSL.c to detect timeout in ACK
extern TIMER_TABLE_ENTRY timeout_table[MAX_TIMEOUT_TABLE_SIZE];


// Function Prototypes
extern void wbsl_main(void);
extern void wbsl_config(void);
extern void wbsl_RfIsr(void);
extern void wbsl_RfTxRxIsr(void);
extern void wbsl_sendPacket(void);
extern void wbsl_stopRadio(void);
extern void wbsl_enableRadio(void);
extern unsigned char wbsl_link(void);
extern void wbsl_reset(void);

extern void resetTimer(unsigned char index);
extern void setTimer(unsigned char index, unsigned int timeout);

#endif
