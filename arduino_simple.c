#include <stdio.h>
#include <avr/io.h>
#include <string.h>
#include "arduino_simple.h"

//nano board related pinout
//position in array - pin on arduino
//data in array [PORT, pin on port]
//PORTB=1; PORTC=2; PORTD=3; supply=9
//                            1      2      3      4      5      6      7      8      9
//                            10     11     12     13     14     15     16     17     18
//                            19     20     21     22     23     24     25     26     27
//                            28     29     30     31     32
const uint8_t pinMap[][2] = {{3,3}, {3,4}, {9,9}, {9,9}, {9,9}, {9,9}, {1,6}, {1,7}, {3,5}, \
                             {3,6}, {3,7}, {1,0}, {1,1}, {1,2}, {1,3}, {1,4}, {1,5}, {9,9}, \
							               {9,9}, {9,9}, {9,9}, {9,9}, {2,0}, {2,1}, {2,2}, {2,3}, {2,4}, \
							               {2,5}, {2,6}, {3,0}, {3,1}, {3,2} };
                             
                            
uint8_t volatile maska;
uint8_t volatile *p;
uint8_t *pinDef0, *pinDef1;

void digitalWrite(uint8_t volatile pin, uint8_t volatile state)
{
  //memcpy((void*)&pinDef, pinMap[pin-1], 2);
  pinDef0 = (uint8_t*)&pinMap[pin-1][0];
  pinDef1 = (uint8_t*)&pinMap[pin-1][1];
  if (*pinDef0 == 9) return; //supply or other non I/O pin
  maska = (1 << *pinDef1);
  
  switch (*pinDef0) {
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
  //memcpy((void*)&pinDef, pinMap[pin-1], 2);
  pinDef0 = (uint8_t*)&pinMap[pin-1][0];
  pinDef1 = (uint8_t*)&pinMap[pin-1][1];
  if (*pinDef0 == 9) return 0; //supply or other non I/O pin
  maska = (1 << *pinDef1); //pinMap[pin-1][1]);
  
  switch (*pinDef0) { //(pinMap[pin-1][0]) {
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
  //memcpy((void*)&pinDef, pinMap[pin-1], 2);
  pinDef0 = (uint8_t*)&pinMap[pin-1][0];
  pinDef1 = (uint8_t*)&pinMap[pin-1][1];
  if (*pinDef0 == 9) return; //supply or other non I/O pin
  maska = (1 << *pinDef1); //pinMap[pin-1][1]);
  
  switch (*pinDef0) { //(pinMap[pin-1][0]) {
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
