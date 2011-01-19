#ifndef __EMBEDDED_UART__
#define __EMBEDDED_UART__
// Define to configure AP as embedded system talking to other uC over UART
#define EMBEDDED_UART 1
     
#define UART_XMIT(num, x)         \
   do{                            \
      U##num##DBUF = x;           \
      while(U1CSR & 0x01);        \
   }while (0)

void uart_send_packet(unsigned char byte, unsigned char x, unsigned char y, unsigned char z);
void uart_analyze_packet(unsigned char byte, unsigned char x, unsigned char y, unsigned char z);
void bin2ascii(unsigned char * pBuf,unsigned char val);
  
#endif //__EMBEDDED_UART__