// *************************************************************************************************
// Copyright 2007 BM wireless Ltd.&Co.KG, all rights reserved.
// The information contained herein is confidential property of BM wireless Ltd.&Co.KG.
// The use, copying, transfer or disclosure of such information is prohibited
// except by written agreement with BM wireless Ltd.&Co.KG.
// *************************************************************************************************
//
// Actual revision: $Revision: 1.2 $
// Revision label:  $Name: $
// Revision state:  $State: Coding $
//
// *************************************************************************************************


// *************************************************************************************************
// Include section


// *************************************************************************************************
// Defines section

// Function jump table entries
#define USER_MAIN         0
#define USER_INIT         1
#define USER_USB_DECODE   2
#define RF_ISR            3
#define RFTXRX_ISR        4
#define TIMER1_ISR        5
#define TIMER3_ISR        6

// boundaries for flash erase and r/w area
#define FLASH_RW_PAGE_MIN           (8u)
#define FLASH_RW_PAGE_MAX           (30u)
#define FLASH_PAGE_SIZE             (1024u)

// flash write/erase key
#define FLASH_EW_KEY                (0x5732)

// XDATA start address for flash write routine
#define FLASH_WRITE_XDATA_ADDR      (0xFC00)

typedef void ( *func_ptr )( void ) ;


// *************************************************************************************************
// Extern section
extern __code const func_ptr func_jump_table[];

extern void flash_start_update(void);
extern u8 flash_byte_read(u16 addr);
extern u16 flash_word_read(u16 addr);
extern u8 flash_word_write(u16 addr, u16 data, u16 key, u8 verify);
extern u8 page_erase(u8 page, u16 key, u8 verify);
extern void user_usb_decode(void);

extern u8 flash_update_init;