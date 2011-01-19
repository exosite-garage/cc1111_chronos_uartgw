#define RFTEST_PW                       (0xABAB)

extern void test_rf(void);
extern void rftest_RfIsr(void);
extern void rftest_RfTxRxIsr(void);
extern void start_continuous_tx(void);
extern void stop_continuous_tx(void);
extern void rftest_radio_init(void);

