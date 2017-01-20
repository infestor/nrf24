#pragma GCC diagnostic ignored "-Wwrite-strings"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h> 
#include <avr/sleep.h>
#include <stdio.h>
#include <string.h>
#include "Mirf.h"
#include "Mirf_nRF24L01.h"

#define DEV_ADDR 1

mirfPacket volatile inPacket;
mirfPacket volatile outPacket;
uint8_t volatile citac;
uint8_t volatile uartPos = 0;
//uint8_t volatile uartBufEmpty = 1;
uint8_t volatile uartIncoming = 0;
uint8_t volatile awaitingResult;
uint8_t volatile timerInterruptTriggered = 0;

char buff[35];

typedef union {
  uint16_t uint;
  struct {
    uint8_t lsb;
    uint8_t msb;
  };
} IntUnion;

void main() __attribute__ ((noreturn));

//IntUnion volatile adcVal;

//======================================================
ISR(TIMER0_COMPA_vect) {
	timerInterruptTriggered++;
    citac++;
}

ISR(BADISR_vect) { //just for case
  __asm__("nop\n\t");
}

ISR(USART_RX_vect)
{
	uint8_t inp = UDR0;

	if (uartIncoming == 0)
	{
		if (inp == 254)
		{
			uartIncoming = 1;
			//uartBufEmpty = 1;
			uartPos = 0;
			citac = 0;
		}
	}
	else
	{
		if (uartPos == sizeof(mirfPacket)) //incoming packet is longer than allowed
		{
			uartIncoming = 0;
		}
		else
		{
			((char*)&outPacket)[uartPos] = inp;
			uartPos++;
		}
	}

}
//==========================================================

void USART_Transmit( char *data, uint8_t len )
{
  for (uint8_t i=0; i < len; i++)
  {
    /* Wait for empty transmit buffer */
    while ( !( UCSR0A & (1<<UDRE0)) );
    /* Put data into buffer, sends the data */
    UDR0 = data[i];
  }
}

#ifdef _DEBUG_
void debug(void)
{
  uint8_t i;
  uint8_t volatile rr;
  uint8_t adr[5];
  
  cli();
  for (i=0; i< 10; i++)
  {
    Mirf.readRegister(i, (uint8_t*)&rr, 1);
    sprintf(buff, "%d:%d,", i, rr);
    USART_Transmit(buff, strlen(buff) );
  }
  sprintf(buff, "\n");
  USART_Transmit(buff, strlen(buff) );
  
  Mirf.readRegister(TX_ADDR, (uint8_t*)&adr, 5);
  sprintf(buff, "TX: 0x%2x%2x%2x%2x%2x\n", adr[0], adr[1], adr[2], adr[3], adr[4]);
  USART_Transmit(buff, strlen(buff) );

  Mirf.readRegister(RX_ADDR_P0, (uint8_t*)&adr, 5);
  sprintf(buff, "RX: 0x%2x%2x%2x%2x%2x\n", adr[0], adr[1], adr[2], adr[3], adr[4]);
  USART_Transmit(buff, strlen(buff) );
  
  sei();
}
#endif

void setup()
{
  //configure uart0  (57600, 8bits, no parity, 1 stop bit)
  UBRR0H = 0;
  UBRR0L = 16;
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
  UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0);

  //start radio    
  Mirf.init();
  Mirf.setDevAddr(DEV_ADDR);
  Mirf.config(); 
  Mirf.powerUpRx();

#ifdef _DEBUG_
  Mirf.csnLow();
  uint8_t a = Mirf.spi->transfer(R_REGISTER | ( REGISTER_MASK & 5 ));
  uint8_t b = Mirf.spi->transfer(255);
  uint8_t c = Mirf.spi->transfer(255);
  Mirf.csnHi();
  sprintf(buff, "po: %d, %d, %d\n", a, b, c); //DEBUG
  USART_Transmit(buff, strlen(buff) );
#endif

  //timer0 10ms period, interrupt enable
  //prescaler 1024, count to 156
  OCR0A = 156;
  OCR0B = 170;
  TCCR0A = 2;
  TCCR0B = 5;
  TIMSK0 = 2;

  //disable unused peripherials
  PRR = ( _BV(PRTWI) | _BV(PRTIM1) | _BV(PRTIM2) ) ;
  
  sei();
}

void main(void)
{
 wdt_disable();
 setup();

 memset((void*)&outPacket, 0, sizeof(mirfPacket) );

 USART_Transmit("COMM MASTER\n", 12 );

 //------------------------------------
 while(1)
 {

	 // handle periodic checking of MIRF
	 // originally it was located right in the ISR handling function
	 // but maybe it will be easier when it will be here
	 // and this also is possible even when processor is sleeping after each cycle of this WHILE loop
	 // because the timer ISR wakes it so it comes here right after that
	 if (timerInterruptTriggered > 0)
	 {
		 timerInterruptTriggered = 0;
		 Mirf.handleRxLoop();
		 Mirf.handleTxLoop();
	 }

	 //zpracovat prichozi packet
	 if (Mirf.inPacketReady)
	 {
		 if ( (!uartIncoming) && (!awaitingResult) )
		 {
			 Mirf.readPacket((mirfPacket*)&inPacket);
			 //send whole packet through usart
			 USART_Transmit((char*)&inPacket, sizeof(mirfPacket));

			 //sprintf(buff, "RX:%d,TX:%d,T:%d,C:%d,P:%d\n", inPacket.rxAddr, inPacket.txAddr, inPacket.type, inPacket.counter, inPacket.payload[0]);
			 //USART_Transmit(buff, strlen(buff) );
		 }
	 }

	 if ( (uartIncoming) && (uartPos == (sizeof(mirfPacket)-1)) ) //whole packet is received
	 {
		 uint8_t res = Mirf.sendPacket((mirfPacket*)&outPacket);
		 uartIncoming = 0;

		 if (res == 0) //packet was not transmitted
		 {
			 USART_Transmit("ER", 2);
			 awaitingResult = 0;
		 }
		 else //packet was sent, we will wait for sendResult and send it to usart
		 {
			 awaitingResult = 1;
			 //run tx loop earlier than in timer ISR to speed up sending,
			 //and since handling of rx/tx functions was moved away from ISR function
			 //we dont have to care about disabling the interrupt during invocation of this here
			 Mirf.handleTxLoop();
		 }
	 }

	 if (awaitingResult && (Mirf.sendResult != PROCESSING) ) //we are waiting for send result and it changed from in queue to some other value
	 {
		 if (Mirf.sendResult == SUCCESS) {
			 USART_Transmit("OK", 2);
		 }
		 else if (Mirf.sendResult == TIMEOUT) {
			 USART_Transmit("TO", 2);
		 }
		 else if (Mirf.sendResult == MAX_ATTEMPTS) {
			 USART_Transmit("MA", 2);
		 }

		 awaitingResult = 0;
	 }

	 if ((uartIncoming) && (citac > 1) ) //timeout receiving whole packet
	 {
		 uartIncoming = 0; //reset receiving
	 }

 }

}
