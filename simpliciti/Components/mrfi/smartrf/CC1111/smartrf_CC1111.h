/***************************************************************
 *  SmartRF Studio(tm) Export
 *
 *  Radio register settings specifed with C-code
 *  compatible #define statements.
 *
 ***************************************************************/

// [BM] Modified radio settings for 433MHz, 868MHz, 915MHz 

#ifndef SMARTRF_CC1111_H
#define SMARTRF_CC1111_H

#define SMARTRF_RADIO_CC1111

// ISM_EU configuration
//
// Chipcon
// Product = CC1111
// Chip version = D (VERSION = 0x03)
// X-tal frequency = 24 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 250.000000 kHz
// Deviation = 32 kHz
// Datarate = 76.721191 kBaud
// Modulation = (1) GFSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 869.524658 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = -
// Sync mode = (3) 30/32 sync word bits detected
// Format of RX/TX data = (0) Normal mode
// CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
// Forward Error Correction = (0) FEC disabled
// Length configuration = (1) Variable packet length mode, packet length configured by the first received byte after sync word.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 1
// Address check = (0) No address check
// Device address = 0

// ISM_US configuration
//
// Chipcon
// Product = CC1111
// Chip version = D (VERSION = 0x03)
// X-tal frequency = 24 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 250.000000 kHz
// Deviation = 32 kHz
// Datarate = 76.721191 kBaud
// Modulation = (1) GFSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 905.998901 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = -
// Sync mode = (3) 30/32 sync word bits detected
// Format of RX/TX data = (0) Normal mode
// CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
// Forward Error Correction = (0) FEC disabled
// Length configuration = (1) Variable packet length mode, packet length configured by the first received byte after sync word.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 1
// Address check = (0) No address check
// Device address = 0

#define SMARTRF_SETTING_FSCTRL1     0x09
#define SMARTRF_SETTING_FSCTRL0     0x00
#ifdef ISM_EU
  #define SMARTRF_SETTING_FREQ2       0x24
  #define SMARTRF_SETTING_FREQ1       0x3A
  #define SMARTRF_SETTING_FREQ0       0xEE
#else
  #ifdef ISM_US  
  // 902MHz (CHANNR=20 --> 906MHz)
  #define SMARTRF_SETTING_FREQ2       0x25
  #define SMARTRF_SETTING_FREQ1       0x95
  #define SMARTRF_SETTING_FREQ0       0x55
    // 912MHz (CHANNR=0)
//    #define SMARTRF_SETTING_FREQ2       0x26
//    #define SMARTRF_SETTING_FREQ1       0x00
//    #define SMARTRF_SETTING_FREQ0       0x00
  #else
    #ifdef ISM_LF
      // 433.30MHz
      #define SMARTRF_SETTING_FREQ2       0x12
      #define SMARTRF_SETTING_FREQ1       0x14
      #define SMARTRF_SETTING_FREQ0       0x7A
    #else
      #error "Wrong ISM band specified (valid are ISM_LF, ISM_EU and ISM_US)"
    #endif // ISM_LF
  #endif // ISM_US
#endif // ISM_EU
#define SMARTRF_SETTING_MDMCFG4     0x6B
#define SMARTRF_SETTING_MDMCFG3     0xA3
#define SMARTRF_SETTING_MDMCFG2     0x13
#define SMARTRF_SETTING_MDMCFG1     0x23
#define SMARTRF_SETTING_MDMCFG0     0x11
#define SMARTRF_SETTING_CHANNR      0x00
#define SMARTRF_SETTING_DEVIATN     0x43
#define SMARTRF_SETTING_FREND1      0xB6
#define SMARTRF_SETTING_FREND0      0x10
#define SMARTRF_SETTING_MCSM0       0x18
#define SMARTRF_SETTING_FOCCFG      0x1D
#define SMARTRF_SETTING_BSCFG       0x1C
#define SMARTRF_SETTING_AGCCTRL2    0xC7
#define SMARTRF_SETTING_AGCCTRL1    0x00
#define SMARTRF_SETTING_AGCCTRL0    0xB0
#define SMARTRF_SETTING_FSCAL3      0xEA
#define SMARTRF_SETTING_FSCAL2      0x2A
#define SMARTRF_SETTING_FSCAL1      0x00
#define SMARTRF_SETTING_FSCAL0      0x1F
#define SMARTRF_SETTING_TEST2       0x81
#define SMARTRF_SETTING_TEST1       0x35
#define SMARTRF_SETTING_TEST0       0x09
#define SMARTRF_SETTING_PA_TABLE0   0x50
#define SMARTRF_SETTING_PKTCTRL1    0x04
#define SMARTRF_SETTING_PKTCTRL0    0x05
#define SMARTRF_SETTING_ADDR        0x00
#define SMARTRF_SETTING_PKTLEN      0xFF

#endif // SMARTRF_CC1111_H

