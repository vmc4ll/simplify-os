#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usb_keyboard.h"

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

uint8_t matrix[8][8]= {
/*B0*/  {KEY_TILDE, KEY_Q, 0, 0, KEY_W, KEY_E, KEY_R, KEY_T}, 
/*B1*/  {KEY_SLASH, KEY_A, 0, 0, KEY_S, KEY_D, KEY_F, KEY_G}, 
/*B2*/  {KEY_LEFT_BRACE, KEY_Z, 0, 0, KEY_X, KEY_C, KEY_V, KEY_B}, 
/*B3*/  {0 /*Fn*/, KEY_ENTER, 0, 0,  KEY_TAB, KEY_ESC, KEY_SPACE},

/*B6*/  {KEY_Y, KEY_U, 0, 0, KEY_I, KEY_O, KEY_P, KEY_EQUAL}, 
/*B5*/  {KEY_H, KEY_J, 0, 0, KEY_K, KEY_L, KEY_SEMICOLON, KEY_QUOTE}, 
/*B4*/  {KEY_N, KEY_M, 0, 0, KEY_COMMA, KEY_PERIOD, KEY_SLASH, KEY_RIGHT_BRACE}, 
/*D7*/  {KEY_SPACE, KEY_ESC, 0, 0,  KEY_TAB, KEY_ENTER, 0 /*Fn*/}
};

int main(void)
{
  uint8_t state=0, prev_state[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  setup_pins_and_usb();

  while (1) {
    PORTB = 0b11111110;
    _delay_ms(5);
    state = PINF;
    read_row(state, prev_state[0], matrix[0]);
    prev_state[0] = state;
    PORTB = 0b11111101;
    _delay_ms(5);
    state = PINF;
    read_row(state, prev_state[1], matrix[1]);
    prev_state[1] = state;
    PORTB = 0b11111011;
    _delay_ms(5);
    state = PINF;
    read_row(state, prev_state[2], matrix[2]);
    prev_state[2] = state;
    PORTB = 0b11110111;
    _delay_ms(5);
    state = PINF;
    read_row(state, prev_state[3], matrix[3]);
    prev_state[3] = state;
    PORTB = 0b10111111;
    _delay_ms(5);
    state = PINF;
    read_row(state, prev_state[4], matrix[4]);
    prev_state[4] = state;
    PORTB = 0b11011111;
    _delay_ms(5);
    state = PINF;
    read_row(state, prev_state[5], matrix[5]);
    prev_state[5] = state;
    PORTB = 0b11101111;
    _delay_ms(5);
    state = PINF;
    read_row(state, prev_state[6], matrix[6]);
    prev_state[6] = state;
    PORTB = 0b11111111:
    PORTD = 0b00000000;
    _delay_ms(5);
    state = PINF;
    read_row(state, prev_state[7], matrix[7]);
    prev_state[7] = state;
    PORTD = 0b10000000;
  }
}

void read_row(uint8_t state, uint8_t prev_state, uint8_t keys[]) {
  uint8_t mask, i;

  mask = 1;
  for (i=0; i<8; i++) {
    if(((state & mask) >> i) == 0 && ((prev_state & mask) >> i) == 1 ) {
      usb_keyboard_press(keys[i], 0);
    }
    mask = mask << 1;
  }
}

void setup_pins_and_usb() {
  CPU_PRESCALE(0);
  DDRB = 0b01111111;
  DDRD = 0b10000000;
  PORTB = 0b01111111;
  PORTD = 0b10000000;
  DDRF = 0x00;
  PORTF = 0xFF;
  usb_init();
  while (!usb_configured()) 
  _delay_ms(1000);
}
