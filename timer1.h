#include "project.h"

extern void reset_timer1(void);
extern u8 set_timer1_cycles(void);
extern void set_timer1(u16 aclk);
extern void set_timer1_abs(u16 aclk);
extern void enable_timer1_irq(void);
extern void disable_timer1_irq(void);
extern void clear_timer1_irq(void);

struct timer1
{
  u8            enable;
  u8            iflag;
  u16           aclk;
  u8            nb_full_cycles;  // 0xFFFF
  u16           last_cycle_count;  // 0x????
  u8            usb_service_disable;
  u8            cycles;
};
extern struct timer1 sTimer1;