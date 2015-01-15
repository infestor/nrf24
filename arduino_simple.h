//library (header) which should simulate some arduino pin related functions
//on arduino nano board with atmega328, but without using arduino api..
//This is for specific simple tasks, where the arduino overhead is not necessary

#ifndef __ARDUINO_SIMPLE_H__
#define __ARDUINO_SIMPLE_H__

#include <avr/io.h>
//#include <inttypes.h>

#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1

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
//arduino board pin index
#define PIN_SS 14
#define PIN_MOSI 15
#define PIN_MISO 16
#define PIN_SCK 17

void digitalWrite(uint8_t pin, uint8_t state);
void pinMode(uint8_t pin, uint8_t mode);
uint8_t digitalRead(uint8_t pin);

#endif //__ARDUINO_SIMPLE_H__
