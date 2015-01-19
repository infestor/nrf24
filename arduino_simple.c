#define __AVR_ATmega328__ 1
#include <stdio.h>
#include <avr/io.h>
#include <string.h>
#include "arduino_simple.h"

uint8_t volatile maska;
uint8_t volatile *p;
uint8_t volatile pinDef[2];

void digitalWrite(uint8_t volatile pin, uint8_t volatile state)
{
  memcpy((void*)&pinDef, pinMap[pin-1], 2);
  maska = (1 << pinDef[1]);
  
  switch (pinDef[0]) {
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

  if (p==0) return;
  if (state == HIGH) {
    *p = *p | maska;
  }
  else {
    *p = *p & (~maska);
  }
}

uint8_t digitalRead(uint8_t volatile pin)
{
  memcpy((void*)&pinDef, pinMap[pin-1], 2);
  maska = (1 << pinDef[1]); //pinMap[pin-1][1]);
  
  switch (pinDef[0]) { //(pinMap[pin-1][0]) {
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
  if (p==0) return(0);
  
  return (*p & maska);
}

void pinMode(uint8_t volatile pin, uint8_t volatile mode)
{
  //create mask
  memcpy((void*)&pinDef, pinMap[pin-1], 2);
  maska = (1 << pinDef[1]); //pinMap[pin-1][1]);
  
  switch (pinDef[0]) { //(pinMap[pin-1][0]) {
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

  if (p==0) return;
  if (mode == OUTPUT) {
    *p = *p | maska;
  }
  else {
    *p = *p & (~maska);
  }
  
}
