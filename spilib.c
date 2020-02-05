
#include <avr/io.h>
#include "spilib.h"

__attribute__((used)) SPIlib SPI = SPIlib();

// Initialize SPI Master Device (without interrupt)
SPIlib::SPIlib(void)
{
  //nothing to do so far
}

void SPIlib::begin(void)
{
    // Set MOSI, SCK as Output
    DDRB |= (1<<3)|(1<<5);
    //PORTB |= (1<<4);
 	//pinMode(PIN_MOSI, OUTPUT);
	//pinMode(PIN_SCK, OUTPUT);
	//digitalWrite(PIN_MISO, HIGH); //will connect pull up resistor
   
    // Enable SPI, Set as Master
    //Prescaler: Fosc/2, Enable Interrupts
    SPCR = (1<<SPE)|(1<<MSTR);
	SPSR = 1; //1
}

//Function to send and receive data for both master and slave
uint8_t SPIlib::transfer (uint8_t data)
{
    // Load data into the buffer
    SPDR = data;
 
    //Wait until transmission complete
    while(!(SPSR & (1 << SPIF)))
	;
 
    // Return received data
    return(SPDR);
}

void SPIlib::transfer (void *buf, uint8_t count) {
    if (count == 0) return;
    uint8_t out, in;
    uint8_t *p = (uint8_t *)buf;
    SPDR = *p;
    while (--count > 0) {
      out = *(p + 1);
      while (!(SPSR & _BV(SPIF))) ;
      in = SPDR;
      SPDR = out;
      *p++ = in;
    }
    while (!(SPSR & _BV(SPIF))) ;
    *p = SPDR;
}
  
void SPIlib::end(void)
{
    // Set MOSI, SCK as Input
 	//pinMode(PIN_MOSI, INPUT);
	//pinMode(PIN_SCK, INPUT);
  DDRB &= ~((1<<3)|(1<<5));
 
    // disable SPI
    SPCR = 0;
	SPSR = 0;
}

void SPIlib::setDataMode(uint8_t)
{
}

void SPIlib::setClockDivider(uint8_t)
{
	
}
