/* Includes ----------------------------------------------------------------- */
#include <TM1637Display.h>
#include <Wire.h>
#include <VL53L0X.h>

#include "config.h"

/* Global instances --------------------------------------------------------- */
TM1637Display display(SEVSEG_CLK, SEVSEG_DIO);
#define START_COND_LIMIT 3000
#define STOP_COND_LIMIT  2000

// VL0X instances
VL53L0X lox1;
VL53L0X lox2;
uint16_t range1 = 9999;
uint16_t range2 = 9999;

// state control
state_t state = state_idle;
volatile uint8_t incrementState = 0;

// counter
uint16_t timer1_counter = 0;
uint16_t g_sevseg_output = 0;

// clock
volatile clock_t clock        = {0, 0, 0};
uint32_t millis_trigger_start = 0;  // used for triggering start condition
uint8_t trigger_start_cond    = 0;
uint8_t log_carina            = 0;
uint32_t millis_carina        = 0;
uint8_t tube_removed          = 0;
uint32_t millis_tube_removed  = 0;
uint32_t millis_trigger_stop  = 0;  // used for triggering stop condition
uint8_t trigger_stop_cond     = 0;

/* constants ---------------------------------------------------------------- */
const uint8_t sevseg_dotMask = 64;

/* setup -------------------------------------------------------------------- */
void setup()
{
  Serial.begin(VC_BAUD);
  Wire.begin();

  // init sevseg
  display.setBrightness(0x0f);
  display.clear();
  display.showNumberDecEx(0, sevseg_dotMask, 1);

  // init buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, 0); // turn off buzzer

  // init user button
  pinMode(SW_PIN, INPUT_PULLUP);

  // init L0X's
  pinMode(L0X1_XSHUT_PIN, OUTPUT);
  pinMode(L0X2_XSHUT_PIN, OUTPUT);
  initL0Xs();

  // init timer
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;

  timer1_counter = 64911; // 100Hz
  TCNT1 = timer1_counter; // preload timer
  TCCR1B |= (1 << CS12);  // 256 prescaler
  TIMSK1 |= (1 << TOIE1); // enable OVF Interrupt
  interrupts();
}

void loop()
{
  switch (state)
  {
  case state_idle:
    range1 = getRange(lox1);

    // change state when hover more than limits
    if (range1 < 40 && trigger_start_cond == 0)
    {
      // start counter
      millis_trigger_start = millis();
      trigger_start_cond = 1;

      Serial.println("Start condition");
    }

    // check for hovering
    if (trigger_start_cond)
    {
      if (range1 < 40) // still in range
      {
        if (millis() - millis_trigger_start >= START_COND_LIMIT) {
          buzzBeep(100);
          state = state_clock;
          Serial.println("Clock started");
        }
      }
      else  // out of range
      {
        millis_trigger_start = millis(); // reset millis
        trigger_start_cond = 0;
        Serial.println("Out of range");
      }
    }

    break;

  case state_clock:
    range1 = getRange(lox1);
    range2 = getRange(lox2);

    Serial.print(range1); Serial.print("   "); Serial.println(range2);

    // write clock output sevseg
    g_sevseg_output = clock.sec * 100 + clock.millis;
    if (g_sevseg_output > 2000) g_sevseg_output = 2000;
    display.showNumberDecEx(g_sevseg_output, sevseg_dotMask, 1);
    
    // clocking
    if (clock.millis >= 100)
    {
      clock.millis = 0;
      clock.sec++;
    }

    // trigger carina logging when tube reaches sensor 2
    if (range2 < 55 && log_carina == 0) {
      log_carina = 1;
      buzzBeep(20);
      millis_carina = millis();

      Serial.println("Start logging carina!");
    }

    // carina
    if (log_carina) {
      if (millis() - millis_carina >= 3000) {
        // if tube stays too long at carina then stop carina clock and clock
        clock.stop_src = 1;
      } 
      else {
        if (range2 >= 60) {
          // if tube exits carina then stop carina clock
          log_carina = 0;
          millis_carina = 0;

          tube_removed = 1;
          // millis_tube_removed = millis();
        }
      }
    }

    if (tube_removed) {
      // change state when hover more than limits
      if (range1 < 40 && trigger_stop_cond == 0)
      {
        // start counter
        millis_trigger_stop = millis();
        trigger_stop_cond = 1;

        Serial.println("Stop condition");
      }

      // check for hovering
      if (trigger_stop_cond)
      {
        if (range1 < 40) // still in range
        {
          if (millis() - millis_trigger_stop >= STOP_COND_LIMIT) {
            buzzBeep(100);
            state = state_stop;
            Serial.println("Clock stopped");
          }
        }
        else  // out of range
        {
          millis_trigger_start = millis(); // reset millis
          trigger_stop_cond = 0;
          Serial.println("Out of range");
        }
      }
    }

    // timeout
    if (clock.sec >= 20) clock.stop_src = 2;

    // stop clock by user btn
    if (!digitalRead(SW_PIN)) clock.stop_src = 3;

    // exit state
    if (clock.stop_src > 0)
    {
      Serial.println("Clock stopped.");
      
      turnOffTimer1();

      if (clock.stop_src == 1)
        repeatBeep(10);
      else if (clock.stop_src == 2 || clock.stop_src == 4)
        repeatBeep(2);
      else if (clock.stop_src == 3)
        repeatBeep(3); 

      state = state_stop;
    }

    break;

  case state_stop:
    delay(1000);
    for (int i=0; i<2; ++i) {
      buzzBeep(300);
      display.clear();
      delay(300);
      display.showNumberDecEx(g_sevseg_output, sevseg_dotMask, 1);
      delay(300);
    }

    state = state_complete;
    break;

  case state_complete:
    // reset value
    clock = {0, 0, 0};
    incrementState = 0;
    g_sevseg_output = 0;
    log_carina = 0;
    tube_removed = 0;

    display.showNumberDecEx(0, sevseg_dotMask, 1);
    turnOnTimer1();

    state = state_idle;
    break;
  }
}

/* ISR ---------------------------------------------------------------------- */
// timer ISR
ISR(TIMER1_OVF_vect)
{
  TCNT1 = timer1_counter; // preload timer

  if (state == state_clock)
  {
    clock.millis++;
  }
}

/* USER DEFINED FNS --------------------------------------------------------- */
void initL0Xs()
{
  /* setting address */
  // reset all L0X
  digitalWrite(L0X1_XSHUT_PIN, LOW);
  digitalWrite(L0X2_XSHUT_PIN, LOW);
  delay(10);
  digitalWrite(L0X1_XSHUT_PIN, HIGH);
  digitalWrite(L0X2_XSHUT_PIN, HIGH);
  delay(10);

  // shutdown L0X2
  digitalWrite(L0X2_XSHUT_PIN, LOW);
  delay(10);

  // init L0X1
  lox1.setAddress(L0X1_ADDR);
  lox1.setTimeout(500);
  lox1.init();

  // wake up L0X2
  digitalWrite(L0X2_XSHUT_PIN, HIGH);
  delay(10);

  // init L0X2
  lox2.setAddress(L0X2_ADDR);
  lox2.setTimeout(500);
  lox2.init();
}

uint16_t getRange(VL53L0X lox)
{
  uint16_t range = 9999;
  range = lox.readRangeSingleMillimeters();

  return range;
}

void turnOnTimer1()
{
  // enable timer interrupt
  noInterrupts();
  TIMSK1 |= (1 << TOIE1);
  interrupts();
}

void turnOffTimer1()
{
  noInterrupts();
  TIMSK1 &= ~(1 << TOIE1); // turn off timer interrupt
  interrupts();
}

void buzz(uint8_t state)
{
  digitalWrite(BUZZER_PIN, state);
}

void buzzBeep(uint16_t duration)
{
  buzz(1);
  delay(duration);
  buzz(0);
}

void repeatBeep(uint16_t times)
{
  for (int i = 0; i < times; ++i)
  {
    buzzBeep(100);
    delay(100);
  }
}
