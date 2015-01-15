#include "arduino_simple.h"

void digitalWrite(uint8_t pin, uint8_t state)
{
  uint8_t maska;
  volatile uint8_t *p;
  
  maska = (1 << pinMap[pin-1][1]);
  
  switch (pinMap[pin-1][0]) {
  case 1: //PORTB
    p = &PORTB;
    break;
  case 2: //PORTC
    p = &PORTC;
    break;
  case 3: //PORTD
    p = &PORTD;
    break;
  }
  
  if (state == HIGH) {
    *p = *p | maska;
  }
  else {
    *p = *p & (~maska);
  }
}

uint8_t digitalRead(uint8_t pin)
{
  uint8_t maska;
  volatile uint8_t *p;
  
  maska = (1 << pinMap[pin-1][1]);
  
  switch (pinMap[pin-1][0]) {
  case 1: //PORTB
    p = &PINB;
    break;
  case 2: //PORTC
    p = &PINC;
    break;
  case 3: //PORTD
    p = &PIND;
    break;
  }
  
  return (*p & maska);
}

void pinMode(uint8_t pin, uint8_t mode)
{
  //create mask
  uint8_t maska;
  volatile uint8_t *p;
  
  maska = (1 << pinMap[pin-1][1]);
  
  switch (pinMap[pin-1][0]) {
  case 1: //DDRB
    p = &DDRB;
    break;
  case 2: //DDRC
    p = &DDRC;
    break;
  case 3: //DDRD
    p = &DDRD;
    break;
  }
  
  if (mode == OUTPUT) {
    *p = *p | maska;
  }
  else {
    *p = *p & (~maska);
  }
  
}
