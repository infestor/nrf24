// Library SPIlib
// for hardware SPI of Atmega328 on arduino Nano board
// should have same interface as standard Arduino SPI library, but not using arduino
// Jan Benes (2015) infestor@centrum.cz

#ifndef __SPILIB_H__
#define __SPILIB_H__

#include <stdint.h>

#define SPI_MODE0 0
#define SPI_2XCLOCK_MASK 4
                     
class SPIlib {
public:

  SPIlib();

  void begin(void);
  uint8_t transfer(uint8_t data);
  void transfer(void *buf, uint8_t count);
  void end(void);
  void setDataMode(uint8_t);
  void setClockDivider(uint8_t);


};

extern SPIlib SPI;

#endif  //__SPILIB_H__
