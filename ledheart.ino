/*
* Project:	Voice controlled LED heart
* Author:	  Aditya K.
* Date:		  1/10/2017
* License:	MIT
*
*/


#include <SoftwareSerial.h>
#include "SimpleVR.h"

#define MAX_PWM 4095
#define MIN_PWM 0

#define DATA  PD5 // 5 <-- IDE pins
#define CLOCK PD6 // 6
#define LATCH PD7 // 7

//  Pull OE pin HIGH to disable TLC
#define OE    PB0 // 8

// Toggle pin HIGH to turn off TLC
#define TLC_DISABLE() (PORTB |= bit(OE))

// Toggle pin LOW to turn on TLC
#define TLC_ENABLE()  (PORTB &= ~bit(OE))

#define DATA_HIGH()   (PORTD |= bit(DATA))
#define DATA_LOW()    (PORTD &= ~bit(DATA))

#define CLOCK_HIGH()  ((PORTD |= bit(CLOCK)))
#define CLOCK_LOW()   ((PORTD &= ~bit(CLOCK)))


#define LATCH_ENABLE()  ((PORTD |= bit(LATCH)))
#define LATCH_DISABLE() ((PORTD &= ~bit(LATCH)))

// Speech commands 

#define ILU     (0x1)    // I LOVE YOU
#define ILUT    (0x2)    // I LOVE YOU TOO
#define ME_TOO  (0x3)    // ME TOO
#define YES     (0x4)    // YES

#define ILUBAF  (0x6)    // I LOVE YOU BUT AS A FRIEND
#define SORRY   (0x7)    // I AM SORRY
#define INST    (0x9)    // I NEED SOME TIME

uint16_t pwm_buffer[24];
uint8_t receive[10];

static bool reply = false;
unsigned long start;
const uint16_t wait  = 8000;


VR Heart(2, 3);

//TLC5947 related functions
void init_tlc() {
  memset(pwm_buffer, 0, 24);
  pinMode(DATA, OUTPUT);
  pinMode(CLOCK, OUTPUT);
  pinMode(LATCH, OUTPUT);
  pinMode(OE, OUTPUT);
  LATCH_DISABLE();
  TLC_DISABLE();
}

inline void tlc_set_pwm(uint16_t chan, uint16_t pwm) {
  pwm_buffer[chan] = pwm;
}

inline void tlc_clear_buff(void) {
  for (uint8_t i = 0; i < 24; i++)
    pwm_buffer[i] = 0;
}


void tlc_write(void) {
  LATCH_DISABLE();

  for (int t = 23; t >= 0; t--) {
    for (int b = 11; b >= 0; b--) {
      CLOCK_LOW();
      if (pwm_buffer[t] & (1 << b)) {
        DATA_HIGH();
      } else {
        DATA_LOW();
      }
      CLOCK_HIGH();
    }
  }
  CLOCK_LOW();
  LATCH_ENABLE();
  LATCH_DISABLE();
}

// code reference: https://docs.micropython.org/en/latest/pyboard/pyboard/tutorial/fading_led.html
void heartbeat_effect() {

  uint16_t curr_steps = 0;
  uint16_t step_incr = 117; // total of 35 steps: 117*35  = 4095

  while (1) {
    for (uint8_t i = 0; i < 24; i++) {
      tlc_set_pwm(i, curr_steps);
    }
    tlc_write();
    delay(45);
    curr_steps += step_incr;

    if (curr_steps > MAX_PWM) {
      curr_steps = MAX_PWM;
      step_incr *= -1;
    }
    else if (curr_steps < MIN_PWM) {
      curr_steps = MIN_PWM;
      step_incr *= -1;
    }
  }
}


void heart_effect() {
  uint8_t left_led;
  uint8_t right_led;
  for (left_led = 0, right_led = 23; left_led <= 12 && right_led >= 12; left_led++, right_led--) {
    tlc_set_pwm(left_led, MAX_PWM);
    tlc_set_pwm(right_led, MAX_PWM);
    tlc_write();
    delay(45);
  }
}

void recognizer() {
  int ret;
  uint16_t voice = 0;
  ret = Heart.recognize(receive, 45);
  if (ret > 0) {
    voice += receive[0];
    voice <<= 8;
    voice += receive[1];
  }

  switch (voice) {
    case ILU:
      if (!reply) {
        // We're here for the first time.
        reply = true;
        start = millis();
        break;
      }
      else {
        // We've been here before, this means recognizer misinterpreted "I love you too" as "I love you"
        // so run LED effects
        heart_effect();
        delay(1000);
        heartbeat_effect();
        break;
      }

    case ILUT:
      if (!reply) {
        // We're here for the first time.
        // this time recognizer missinterpreted "I love you" as "I love you too"
        reply = true;
        start = millis();
        break;
      }
    case YES:
    case ME_TOO:
      if (reply) {
        heart_effect();
        delay(1000);
        heartbeat_effect();
      }
      break;

    case SORRY:
      if (reply) {
        // :( better luck next time.
      }
      break;
    case ILUBAF:
      if (reply) {
        // you just got friendzoned. :(
      }
      break;
    case INST:
      if (reply) {
        // we have some hopes
        tlc_clear_buff();
        tlc_write();
        heart_effect();
      }
  }
  if (reply) {
    if (millis() - start > wait) { // wait for 8 seconds
      reply = 0;
      // Timeout
      // TODO: What to do?
    }
  }
}

void setup() {
  Heart.begin(9600);
  Heart.setGroup(1);
  Heart.setEnable(true);

  init_tlc();
  tlc_clear_buff();
  tlc_write();
  TLC_ENABLE();
}

void loop() {
  recognizer();
}
