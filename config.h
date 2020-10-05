// general
#define DEBUG   1
#define VC_BAUD 115200

// peripherals
#define SEVSEG_DIO     4
#define SEVSEG_CLK     5
#define BUZZER_PIN     6
#define SW_PIN         3
#define L0X1_XSHUT_PIN 12
#define L0X2_XSHUT_PIN 13
#define L0X1_ADDR      0x30 // default 0x29
#define L0X2_ADDR      0x31 // default 0x29

// enums
typedef enum {
  state_idle = 0x00,
  state_clock,
  state_stop,
  state_complete
} state_t;

// structs
typedef struct {
  uint8_t sec;
  uint8_t millis;
  uint8_t stop_src; // 1:carina, 2:timeout, 3:button, 4:tube removed
} clock_t;