//library (header) which should simulate some arduino pin related functions
//on arduino nano board with atmega328, but without using arduino api..
//This is for specific simple tasks, where the arduino overhead is not necessary

#ifndef __ARDUINO_SIMPLE_H__
#define __ARDUINO_SIMPLE_H__

//#include <avr/io.h>

#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1

//arduino board pin index
#define PIN_SS 14
#define PIN_MOSI 15
#define PIN_MISO 16
#define PIN_SCK 17

void digitalWrite(uint8_t volatile pin, uint8_t volatile state);
void pinMode(uint8_t volatile pin, uint8_t volatile mode);
uint8_t digitalRead(uint8_t volatile pin);

#endif //__ARDUINO_SIMPLE_H__
